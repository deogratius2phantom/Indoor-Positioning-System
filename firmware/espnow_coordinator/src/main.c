#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_log.h"

static const char *TAG = "coordinator";

/* ================================================================
   Configuration — must match the sniffer nodes
   ================================================================ */

#define REPORT_CHANNEL         1       /* channel used for all ESP-NOW comms  */
#define MAX_NODES              8       /* maximum simultaneous sniffer nodes   */
#define MAX_DEVICES_NODE       64      /* maximum tracked devices per node     */
#define QUEUE_DEPTH            64      /* incoming report queue depth          */
#define STALE_MS               60000   /* evict device entries unseen for 60 s */
#define DISPLAY_PERIOD_MS      10000   /* print aggregated table every 10 s    */
#define SYNC_PERIOD_MS           30000  /* steady-state sync cadence            */
#define SYNC_STARTUP_PERIOD_MS   3000   /* rapid cadence during first 60 s      */
#define SYNC_STARTUP_DURATION_MS 60000  /* duration of the startup burst phase  */
#define SYNC_BCAST_INTERVAL_MS   500    /* gap between repeated epoch broadcasts */
#define SYNC_BCAST_COUNT         14

/* ================================================================
   Trilateration Configuration
   ================================================================ */

/* RSSI (dBm) measured at exactly 1 metre from a transmitter.
   Calibrate this: place a phone 1 m from one node, check avg_rssi,
   use that value here.  Typical indoor range: -40 to -55 dBm.      */
#define RSSI_REF               -50

/* Indoor path-loss exponent.
   2.0 = free space;  2.7 = open office;  3.0-3.5 = walls/obstacles */
#define PATH_LOSS_EXP          3.0f

/* Cross-node device table capacity */
#define MAX_TRILAT_DEVICES     32

/* Run a trilateration pass this often */
#define TRILAT_PERIOD_MS       5000

/* Accept observations whose window_id differs by at most this much.
   Set to 2 to handle nodes that sync on different broadcasts within
   the same epoch (up to 7 s apart → at most 1 window difference +
   one extra slot of margin).                                         */
#define WINDOW_TOLERANCE       2

/* Exact number of nodes for the algebraic 3-node solver */
#define NUM_NODES_EXPECTED     3

/* ================================================================
   Node Physical Positions
   ---------------------------------------------------------------
   Replace the placeholder MACs with the MAC printed by each sniffer
   node at boot ("Sniffer node MAC: XX:XX:XX:XX:XX:XX").
   Set x_m / y_m to the node's physical position in metres,
   measured from a common room origin (e.g. one corner of the room).

   Example layout (bird's-eye view):

       Node 3 (2.5, 4.0)
           *

       *               *
   Node 1 (0, 0)   Node 2 (5, 0)   ← origin wall
   ================================================================ */

typedef struct {
    uint8_t mac[6];
    float   x_m;
    float   y_m;
} node_position_t;

static const node_position_t NODE_POSITIONS[NUM_NODES_EXPECTED] = {
    /* { MAC bytes },                   X (m)   Y (m) */
    { {0x00,0x00,0x00,0x00,0x00,0x01},  0.0f,   0.0f  }, /* Node 1 — origin     */
    { {0x00,0x00,0x00,0x00,0x00,0x02},  5.0f,   0.0f  }, /* Node 2 — X-axis     */
    { {0x00,0x00,0x00,0x00,0x00,0x03},  2.5f,   4.0f  }, /* Node 3 — far wall   */
};

/* ================================================================
   Message types — MUST match espnow_sniffer_node
   ================================================================ */

#define MSG_RSSI_REPORT  0x01
#define MSG_SYNC_EPOCH   0x02
#define MSG_SYNC_ACK     0x03
#define MSG_SYNC_REQUEST 0x04   /* Node → coordinator: "please send a sync epoch now" */

/* ================================================================
   Wire structures (packed — shared between coordinator & sniffer)
   ================================================================ */

/* Coordinator → all nodes (broadcast) */
typedef struct {
    uint8_t  msg_type;       /* MSG_SYNC_EPOCH                              */
    uint32_t epoch_id;
    uint32_t coord_T1_ms;    /* captured ONCE per epoch, same in all bcast  */
} __attribute__((packed)) sync_epoch_t;

/* Node → coordinator */
typedef struct {
    uint8_t  msg_type;       /* MSG_SYNC_ACK                                */
    uint32_t epoch_id;
    uint32_t coord_T1_ms;
    uint32_t node_T2_ms;
    uint32_t node_T3_ms;
} __attribute__((packed)) sync_ack_t;

/* Node → coordinator: on-demand sync request */
typedef struct {
    uint8_t  msg_type;       /* MSG_SYNC_REQUEST                            */
} __attribute__((packed)) sync_request_t;

/* Node → coordinator (per-device RSSI snapshot) */
typedef struct {
    uint8_t  msg_type;            /* MSG_RSSI_REPORT                        */
    uint8_t  sender_mac[6];
    uint8_t  target_mac[6];
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;
    uint32_t window_id;
    uint32_t node_timestamp_ms;
} __attribute__((packed)) rssi_report_t;

/* ================================================================
   Internal coordinator structures
   ================================================================ */

/* Per-device record inside a node's table */
typedef struct {
    uint8_t  mac[6];
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;
    uint32_t corrected_ts_ms;
    uint32_t window_id;
} coord_device_t;

/* Per-node state */
typedef struct {
    uint8_t        node_mac[6];
    coord_device_t devices[MAX_DEVICES_NODE];
    uint8_t        device_count;
    uint32_t       last_report_ms;
    uint32_t       total_reports;
    int32_t        clock_offset_ms;
    uint32_t       last_sync_ms;
    bool           sync_valid;
} coord_node_t;

/* Queue envelope for SYNC_ACK */
typedef struct {
    uint8_t    src_mac[6];
    sync_ack_t ack;
} ack_envelope_t;

/* ================================================================
   Trilateration cross-node structures
   ================================================================ */

/* One sighting of a device from a single sniffer node */
typedef struct {
    uint8_t  node_mac[6];
    int8_t   rssi;
    float    distance_m;
    uint32_t window_id;
} node_observation_t;

/* Aggregated entry: all node observations for one target device,
   plus the computed (x, y) estimate once trilateration succeeds.  */
typedef struct {
    uint8_t            target_mac[6];
    char               ssid[33];
    uint8_t            is_random;
    node_observation_t obs[MAX_NODES];
    uint8_t            obs_count;
    float              est_x_m;
    float              est_y_m;
    bool               position_valid;
    uint32_t           last_updated_ms;
} trilat_device_t;

/* ================================================================
   Globals
   ================================================================ */

static coord_node_t      s_nodes[MAX_NODES];
static uint8_t           s_node_count = 0;
static trilat_device_t   s_trilat_devices[MAX_TRILAT_DEVICES];
static uint8_t           s_trilat_count = 0;
static SemaphoreHandle_t s_table_mutex;
static QueueHandle_t     s_report_queue;
static QueueHandle_t     s_ack_queue;
static volatile uint32_t s_epoch_id = 0;
static TaskHandle_t      s_sync_task_handle = NULL;   /* for on-demand wake-up */

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* ================================================================
   Helper: find or register a node in the node table
   ================================================================ */

static coord_node_t *find_or_create_node(const uint8_t *mac)
{
    for (int i = 0; i < s_node_count; i++) {
        if (memcmp(s_nodes[i].node_mac, mac, 6) == 0)
            return &s_nodes[i];
    }
    if (s_node_count >= MAX_NODES) {
        ESP_LOGW(TAG, "Node table full — ignoring new node");
        return NULL;
    }
    coord_node_t *n = &s_nodes[s_node_count++];
    memset(n, 0, sizeof(coord_node_t));
    memcpy(n->node_mac, mac, 6);
    ESP_LOGI(TAG, "New node registered: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return n;
}

/* ================================================================
   Trilateration helpers
   ================================================================ */

/* Log-distance path loss: d = 10 ^ ((RSSI_ref - RSSI) / (10 * n))
   Clamped to [0.1 m, 50 m] to prevent solver instability.           */
static float rssi_to_distance(int8_t rssi)
{
    float exponent = ((float)RSSI_REF - (float)rssi) / (10.0f * PATH_LOSS_EXP);
    float d = powf(10.0f, exponent);
    if (d < 0.1f)  d = 0.1f;
    if (d > 50.0f) d = 50.0f;
    return d;
}

/* Exact algebraic trilateration for 3 nodes using Cramer's rule.
   Subtracts circle 1 from circles 2 and 3 to yield 2 linear eqs:
     A·x + B·y = C
     D·x + E·y = F
   Solve: det = A·E − B·D;  x = (C·E − B·F)/det;  y = (A·F − C·D)/det
   Returns false when nodes are collinear (det ≈ 0).                  */
static bool trilaterate_2d(float x1, float y1, float d1,
                           float x2, float y2, float d2,
                           float x3, float y3, float d3,
                           float *out_x, float *out_y)
{
    float A = 2.0f * (x1 - x2);
    float B = 2.0f * (y1 - y2);
    float C = d2*d2 - d1*d1 - x2*x2 + x1*x1 - y2*y2 + y1*y1;

    float D = 2.0f * (x1 - x3);
    float E = 2.0f * (y1 - y3);
    float F = d3*d3 - d1*d1 - x3*x3 + x1*x1 - y3*y3 + y1*y1;

    float det = A * E - B * D;
    if (fabsf(det) < 1e-6f) return false;

    *out_x = (C * E - B * F) / det;
    *out_y = (A * F - C * D) / det;
    return true;
}

/* ================================================================
   ESP-NOW receive callback — Wi-Fi task context; enqueue only.
   ================================================================ */

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
{
    if (len < 1) return;
    BaseType_t woken = pdFALSE;

    switch (data[0]) {
        case MSG_RSSI_REPORT:
            if (len == (int)sizeof(rssi_report_t)) {
                rssi_report_t report;
                memcpy(&report, data, sizeof(rssi_report_t));
                memcpy(report.sender_mac, recv_info->src_addr, 6);
                xQueueSendFromISR(s_report_queue, &report, &woken);
            }
            break;
        case MSG_SYNC_ACK:
            if (len == (int)sizeof(sync_ack_t)) {
                ack_envelope_t env;
                memcpy(env.src_mac, recv_info->src_addr, 6);
                memcpy(&env.ack, data, sizeof(sync_ack_t));
                xQueueSendFromISR(s_ack_queue, &env, &woken);
            }
            break;
        case MSG_SYNC_REQUEST:
            /* A node is asking for an immediate sync broadcast.
               Wake sync_task early — it will start a new epoch right away. */
            if (s_sync_task_handle) {
                vTaskNotifyGiveFromISR(s_sync_task_handle, &woken);
            }
            break;
        default:
            break;
    }
    portYIELD_FROM_ISR(woken);
}

/* ================================================================
   Aggregator task — dequeues RSSI reports, updates per-node tables
   ================================================================ */

static void aggregator_task(void *pvParameters)
{
    rssi_report_t report;
    while (1) {
        if (xQueueReceive(s_report_queue, &report, portMAX_DELAY) != pdTRUE)
            continue;

        uint32_t recv_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        coord_node_t *node = find_or_create_node(report.sender_mac);
        if (!node) { xSemaphoreGive(s_table_mutex); continue; }

        node->last_report_ms = recv_ms;
        node->total_reports++;

        uint32_t corrected_ts = node->sync_valid
            ? (uint32_t)((int32_t)report.node_timestamp_ms - node->clock_offset_ms)
            : recv_ms;

        coord_device_t *dev = NULL;
        for (int i = 0; i < node->device_count; i++) {
            if (memcmp(node->devices[i].mac, report.target_mac, 6) == 0) {
                dev = &node->devices[i];
                break;
            }
        }
        if (!dev) {
            if (node->device_count >= MAX_DEVICES_NODE) {
                xSemaphoreGive(s_table_mutex);
                continue;
            }
            dev = &node->devices[node->device_count++];
            memset(dev, 0, sizeof(coord_device_t));
            memcpy(dev->mac, report.target_mac, 6);
        }

        dev->avg_rssi        = report.avg_rssi;
        dev->best_channel    = report.best_channel;
        dev->pkt_count       = report.pkt_count;
        dev->is_random       = report.is_random;
        dev->corrected_ts_ms = corrected_ts;
        dev->window_id       = report.window_id;
        if (report.ssid[0] != '\0')
            strncpy(dev->ssid, report.ssid, 32);

        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   Sync-processor task — dequeues SYNC_ACKs, computes clock offsets
   ================================================================ */

static void sync_processor_task(void *pvParameters)
{
    ack_envelope_t env;
    while (1) {
        if (xQueueReceive(s_ack_queue, &env, portMAX_DELAY) != pdTRUE)
            continue;

        uint32_t T4 = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t T1 = env.ack.coord_T1_ms;
        uint32_t T2 = env.ack.node_T2_ms;
        uint32_t T3 = env.ack.node_T3_ms;

        int32_t rtt_ms    = (int32_t)(T4 - T1) - (int32_t)(T3 - T2);
        if (rtt_ms < 0) rtt_ms = 0;
        int32_t offset_ms = (int32_t)(T2 - T1) - (rtt_ms / 2);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);
        coord_node_t *node = find_or_create_node(env.src_mac);
        if (node) {
            node->clock_offset_ms = offset_ms;
            node->last_sync_ms    = T4;
            node->sync_valid      = true;
            ESP_LOGI(TAG, "Sync  %02X:%02X:%02X:%02X:%02X:%02X  "
                          "offset=%+ld ms  rtt=%ld ms",
                     env.src_mac[0], env.src_mac[1], env.src_mac[2],
                     env.src_mac[3], env.src_mac[4], env.src_mac[5],
                     (long)offset_ms, (long)rtt_ms);
        }
        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   Sync task — periodically broadcasts SYNC_EPOCH to all nodes
   ================================================================ */

static void sync_task(void *pvParameters)
{
    /* Store task handle so the recv callback can wake us on demand. */
    s_sync_task_handle = xTaskGetCurrentTaskHandle();
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    while (1) {
        s_epoch_id++;

        /* Capture T1_base ONCE for the entire epoch.  All SYNC_BCAST_COUNT
           retransmissions carry this same value so every sniffer — regardless
           of which broadcast it catches — anchors to the same coordinator
           reference time, giving aligned window_id values at all nodes.     */
        uint32_t T1_base = (uint32_t)(esp_timer_get_time() / 1000ULL);
        ESP_LOGI(TAG, "Sync epoch %lu  T1_base=%lu ms",
                 (unsigned long)s_epoch_id, (unsigned long)T1_base);

        for (int i = 0; i < SYNC_BCAST_COUNT; i++) {
            sync_epoch_t epoch = {
                .msg_type    = MSG_SYNC_EPOCH,
                .epoch_id    = s_epoch_id,
                .coord_T1_ms = T1_base,          /* same for every broadcast */
            };
            esp_now_send(BROADCAST_MAC, (const uint8_t *)&epoch, sizeof(sync_epoch_t));
            vTaskDelay(pdMS_TO_TICKS(SYNC_BCAST_INTERVAL_MS));
        }

        /* Startup burst: use a short 3 s cadence for the first 60 s so nodes
           that boot at any point during that window sync quickly.  After the
           startup phase, fall back to the steady-state 30 s cadence.        */
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000ULL) - start_ms;
        uint32_t period  = (elapsed < SYNC_STARTUP_DURATION_MS)
                           ? SYNC_STARTUP_PERIOD_MS : SYNC_PERIOD_MS;
        ESP_LOGI(TAG, "Sync done. Next in %lu ms  [%s]",
                 (unsigned long)period,
                 elapsed < SYNC_STARTUP_DURATION_MS ? "startup burst" : "steady state");

        /* Sleep for the calculated period, but wake immediately if any node
           sends MSG_SYNC_REQUEST (vTaskNotifyGiveFromISR in recv callback). */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(period));
    }
}

/* ================================================================
   Trilateration task
   Runs every TRILAT_PERIOD_MS:
     1. Rebuilds the cross-node view from s_nodes[].
     2. For each device seen by all NUM_NODES_EXPECTED nodes within
        WINDOW_TOLERANCE, looks up node physical positions and calls
        the algebraic solver.
     3. Stores (est_x_m, est_y_m) in s_trilat_devices[] for the
        display task to print.
   ================================================================ */

static void trilateration_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000));    /* wait for first reports to arrive */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TRILAT_PERIOD_MS));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        /* ---- Step 1: rebuild cross-node view from s_nodes[] ---- */
        s_trilat_count = 0;
        memset(s_trilat_devices, 0, sizeof(s_trilat_devices));

        for (int n = 0; n < s_node_count; n++) {
            coord_node_t *node = &s_nodes[n];
            for (int d = 0; d < node->device_count; d++) {
                coord_device_t *dev = &node->devices[d];

                /* find or create a trilateration entry for this target MAC */
                trilat_device_t *te = NULL;
                for (int t = 0; t < s_trilat_count; t++) {
                    if (memcmp(s_trilat_devices[t].target_mac, dev->mac, 6) == 0) {
                        te = &s_trilat_devices[t];
                        break;
                    }
                }
                if (!te) {
                    if (s_trilat_count >= MAX_TRILAT_DEVICES) continue;
                    te = &s_trilat_devices[s_trilat_count++];
                    memset(te, 0, sizeof(trilat_device_t));
                    memcpy(te->target_mac, dev->mac, 6);
                    te->is_random = dev->is_random;
                    if (dev->ssid[0] != '\0')
                        strncpy(te->ssid, dev->ssid, 32);
                }

                /* upsert this node's observation — keep the most recent window */
                bool updated = false;
                for (int o = 0; o < te->obs_count; o++) {
                    if (memcmp(te->obs[o].node_mac, node->node_mac, 6) == 0) {
                        if (dev->window_id >= te->obs[o].window_id) {
                            te->obs[o].rssi       = dev->avg_rssi;
                            te->obs[o].distance_m = rssi_to_distance(dev->avg_rssi);
                            te->obs[o].window_id  = dev->window_id;
                        }
                        updated = true;
                        break;
                    }
                }
                if (!updated && te->obs_count < MAX_NODES) {
                    node_observation_t *obs = &te->obs[te->obs_count++];
                    memcpy(obs->node_mac, node->node_mac, 6);
                    obs->rssi       = dev->avg_rssi;
                    obs->distance_m = rssi_to_distance(dev->avg_rssi);
                    obs->window_id  = dev->window_id;
                }
            }
        }

        /* ---- Step 2: run trilateration for eligible devices ---- */
        int solved = 0;

        for (int t = 0; t < s_trilat_count; t++) {
            trilat_device_t *te = &s_trilat_devices[t];
            te->position_valid = false;

            if (te->obs_count < NUM_NODES_EXPECTED) continue;

            /* All observations must fall within WINDOW_TOLERANCE of each other */
            uint32_t min_wid = te->obs[0].window_id;
            uint32_t max_wid = te->obs[0].window_id;
            for (int o = 1; o < te->obs_count; o++) {
                if (te->obs[o].window_id < min_wid) min_wid = te->obs[o].window_id;
                if (te->obs[o].window_id > max_wid) max_wid = te->obs[o].window_id;
            }
            if ((max_wid - min_wid) > WINDOW_TOLERANCE) continue;

            /* Look up the physical (x, y) for each reporting node */
            float px[3], py[3], pd[3];
            bool all_found = true;
            for (int o = 0; o < NUM_NODES_EXPECTED; o++) {
                bool found = false;
                for (int p = 0; p < NUM_NODES_EXPECTED; p++) {
                    if (memcmp(NODE_POSITIONS[p].mac, te->obs[o].node_mac, 6) == 0) {
                        px[o] = NODE_POSITIONS[p].x_m;
                        py[o] = NODE_POSITIONS[p].y_m;
                        pd[o] = te->obs[o].distance_m;
                        found = true;
                        break;
                    }
                }
                if (!found) { all_found = false; break; }
            }
            if (!all_found) continue;

            /* Run algebraic solver */
            float ex, ey;
            if (trilaterate_2d(px[0], py[0], pd[0],
                               px[1], py[1], pd[1],
                               px[2], py[2], pd[2],
                               &ex, &ey)) {
                te->est_x_m         = ex;
                te->est_y_m         = ey;
                te->position_valid  = true;
                te->last_updated_ms = now_ms;
                solved++;
            }
        }

        xSemaphoreGive(s_table_mutex);

        if (solved > 0) {
            ESP_LOGI(TAG, "Trilateration: %d device(s) positioned this cycle", solved);
        }
    }
}

/* ================================================================
   Display task — prints per-node RSSI table + trilateration results
   ================================================================ */

static void display_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        /* Evict stale device entries */
        for (int n = 0; n < s_node_count; n++) {
            coord_node_t *node = &s_nodes[n];
            int i = 0;
            while (i < node->device_count) {
                if ((now_ms - node->devices[i].corrected_ts_ms) > STALE_MS)
                    node->devices[i] = node->devices[--node->device_count];
                else
                    i++;
            }
        }

        printf("\n"
               "╔══════════════════════════════════════════════════════════════════╗\n"
               "║  ESP-NOW Coordinator  [%8lu ms]  %d node(s)  epoch=%lu\n"
               "╚══════════════════════════════════════════════════════════════════╝\n",
               (unsigned long)now_ms, s_node_count, (unsigned long)s_epoch_id);

        /* ---- Per-node RSSI table ---- */
        for (int n = 0; n < s_node_count; n++) {
            coord_node_t *node = &s_nodes[n];

            printf("\n  ┌─ Node %d  %02X:%02X:%02X:%02X:%02X:%02X"
                   "  reports=%lu  sync=%-7s  offset=%+ld ms\n",
                   n + 1,
                   node->node_mac[0], node->node_mac[1], node->node_mac[2],
                   node->node_mac[3], node->node_mac[4], node->node_mac[5],
                   (unsigned long)node->total_reports,
                   node->sync_valid ? "OK" : "pending",
                   node->sync_valid ? (long)node->clock_offset_ms : 0L);

            if (node->device_count == 0) {
                printf("  │  (no devices tracked yet)\n");
                continue;
            }

            printf("  │  %-3s  %-17s  %-4s  %-3s  %-32s  %-8s  %-6s  %-6s\n",
                   "#", "Device MAC", "RND", "Ch", "WiFi SSID",
                   "AvgRSSI", "WinID", "Pkts");
            printf("  │  ---  -----------------  ----  ---"
                   "  --------------------------------  --------  ------  ------\n");

            for (int d = 0; d < node->device_count; d++) {
                coord_device_t *dev = &node->devices[d];
                printf("  │  %-3d  %02X:%02X:%02X:%02X:%02X:%02X  %-4s  %3d"
                       "  %-32s  %4d dBm  %-6lu  %lu\n",
                       d + 1,
                       dev->mac[0], dev->mac[1], dev->mac[2],
                       dev->mac[3], dev->mac[4], dev->mac[5],
                       dev->is_random ? "[R]" : "   ",
                       dev->best_channel,
                       dev->ssid[0] ? dev->ssid : "<hidden>",
                       dev->avg_rssi,
                       (unsigned long)dev->window_id,
                       (unsigned long)dev->pkt_count);
            }
        }

        /* ---- Trilateration results ---- */
        int positioned = 0;
        for (int t = 0; t < s_trilat_count; t++) {
            if (s_trilat_devices[t].position_valid) positioned++;
        }

        printf("\n"
               "  ┌─ Trilateration  (RSSI_REF=%d dBm  n=%.1f)"
               "  %d tracked  %d positioned\n",
               RSSI_REF, PATH_LOSS_EXP, s_trilat_count, positioned);

        if (s_trilat_count == 0) {
            printf("  │  (waiting for devices seen by all %d nodes in same window)\n",
                   NUM_NODES_EXPECTED);
        } else {
            printf("  │  %-3s  %-17s  %-4s  %-32s  %-10s  %-10s  %-5s\n",
                   "#", "Device MAC", "RND", "SSID", "X (m)", "Y (m)", "Nodes");
            printf("  │  ---  -----------------  ----"
                   "  --------------------------------  ----------  ----------  -----\n");

            for (int t = 0; t < s_trilat_count; t++) {
                trilat_device_t *te = &s_trilat_devices[t];
                if (te->position_valid) {
                    printf("  │  %-3d  %02X:%02X:%02X:%02X:%02X:%02X  %-4s"
                           "  %-32s  %10.2f  %10.2f  %d\n",
                           t + 1,
                           te->target_mac[0], te->target_mac[1], te->target_mac[2],
                           te->target_mac[3], te->target_mac[4], te->target_mac[5],
                           te->is_random ? "[R]" : "   ",
                           te->ssid[0] ? te->ssid : "<hidden>",
                           te->est_x_m, te->est_y_m,
                           te->obs_count);
                } else {
                    printf("  │  %-3d  %02X:%02X:%02X:%02X:%02X:%02X  %-4s"
                           "  %-32s  %-10s  %-10s  %d/%d\n",
                           t + 1,
                           te->target_mac[0], te->target_mac[1], te->target_mac[2],
                           te->target_mac[3], te->target_mac[4], te->target_mac[5],
                           te->is_random ? "[R]" : "   ",
                           te->ssid[0] ? te->ssid : "<hidden>",
                           "---", "---",
                           te->obs_count, NUM_NODES_EXPECTED);
                }
            }
        }

        printf("\n"
               "════════════════════════════════════════════════════════════════════\n\n");

        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   app_main
   ================================================================ */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_table_mutex  = xSemaphoreCreateMutex();
    s_report_queue = xQueueCreate(QUEUE_DEPTH, sizeof(rssi_report_t));
    s_ack_queue    = xQueueCreate(MAX_NODES * 2, sizeof(ack_envelope_t));

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(REPORT_CHANNEL, WIFI_SECOND_CHAN_NONE);

    uint8_t self_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, self_mac);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    esp_now_peer_info_t bcast_peer = {};
    memcpy(bcast_peer.peer_addr, BROADCAST_MAC, 6);
    bcast_peer.channel = REPORT_CHANNEL;
    bcast_peer.ifidx   = WIFI_IF_STA;
    bcast_peer.encrypt = false;
    esp_now_add_peer(&bcast_peer);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP-NOW Coordinator ready");
    ESP_LOGI(TAG, " MAC     : %02X:%02X:%02X:%02X:%02X:%02X",
             self_mac[0], self_mac[1], self_mac[2],
             self_mac[3], self_mac[4], self_mac[5]);
    ESP_LOGI(TAG, " Channel : %d (fixed)", REPORT_CHANNEL);
    ESP_LOGI(TAG, " Sync    : every %d s  (%d broadcasts x %d ms)",
             SYNC_PERIOD_MS / 1000, SYNC_BCAST_COUNT, SYNC_BCAST_INTERVAL_MS);
    ESP_LOGI(TAG, " Trilat  : RSSI_REF=%d dBm  n=%.1f  nodes=%d",
             RSSI_REF, PATH_LOSS_EXP, NUM_NODES_EXPECTED);
    ESP_LOGI(TAG, " -> Update NODE_POSITIONS[] with node MACs and room coordinates");
    ESP_LOGI(TAG, "========================================");

    xTaskCreate(aggregator_task,     "aggregator", 4096, NULL, 5, NULL);
    xTaskCreate(sync_processor_task, "sync_proc",  4096, NULL, 5, NULL);
    xTaskCreate(sync_task,           "sync",       2048, NULL, 4, NULL);
    xTaskCreate(trilateration_task,  "trilat",     4096, NULL, 3, NULL);
    xTaskCreate(display_task,        "display",    4096, NULL, 2, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
