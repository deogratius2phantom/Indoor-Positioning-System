#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
#define DISPLAY_PERIOD_MS      5000    /* emit RSSI table every 5 s            */
#define SYNC_PERIOD_MS           30000  /* steady-state sync cadence            */
#define SYNC_STARTUP_PERIOD_MS   3000   /* rapid cadence during first 60 s      */
#define SYNC_STARTUP_DURATION_MS 60000  /* duration of the startup burst phase  */
#define SYNC_BCAST_INTERVAL_MS   500    /* gap between repeated epoch broadcasts */
#define SYNC_BCAST_COUNT         14

/* Trilateration is now performed in the Python visualizer.
   The coordinator only collects RSSI data and emits RSSI| serial lines. */

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
   Globals
   ================================================================ */

static coord_node_t      s_nodes[MAX_NODES];
static uint8_t           s_node_count = 0;
static SemaphoreHandle_t s_table_mutex;
static QueueHandle_t     s_report_queue;
static QueueHandle_t     s_ack_queue;
static volatile uint32_t s_epoch_id = 0;
static TaskHandle_t      s_sync_task_handle = NULL;   /* for on-demand wake-up */

/* MACs of sniffer nodes discovered via ESP-NOW (sync ack or RSSI report).
   Protected by s_table_mutex.  Emitted as DISC| lines for the visualizer. */
static uint8_t           s_disc_macs[MAX_NODES][6];
static int               s_num_disc = 0;

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* ================================================================
   Helper: record a newly-seen sniffer MAC and emit DISC| serial line.
   Safe to call from any task context (takes its own mutex slice).
   ================================================================ */

static void register_disc_mac(const uint8_t *mac)
{
    bool is_new = false;
    xSemaphoreTake(s_table_mutex, portMAX_DELAY);
    bool found = false;
    for (int i = 0; i < s_num_disc; i++) {
        if (memcmp(s_disc_macs[i], mac, 6) == 0) { found = true; break; }
    }
    if (!found && s_num_disc < MAX_NODES) {
        memcpy(s_disc_macs[s_num_disc++], mac, 6);
        is_new = true;
    }
    xSemaphoreGive(s_table_mutex);
    if (is_new) {
        printf("DISC|%02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        ESP_LOGI(TAG, "Discovered sniffer: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

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
        register_disc_mac(report.sender_mac);
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
            /* Machine-readable sync status for the visualizer terminal */
            printf("SYNC|%02X:%02X:%02X:%02X:%02X:%02X|%+ld|%ld\n",
                   env.src_mac[0], env.src_mac[1], env.src_mac[2],
                   env.src_mac[3], env.src_mac[4], env.src_mac[5],
                   (long)offset_ms, (long)rtt_ms);
        }
        xSemaphoreGive(s_table_mutex);
        register_disc_mac(env.src_mac);
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
   UART command task — reads lines from stdin (USB-Serial/JTAG = Python
   visualizer serial port).  Recognised commands:
     CMD:SYNC  — wake sync_task immediately for an on-demand sync broadcast
   ================================================================ */

static void uart_cmd_task(void *pvParameters)
{
    char buf[32];
    while (1) {
        if (fgets(buf, sizeof(buf), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        /* strip trailing \r\n */
        size_t len = strlen(buf);
        while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n'))
            buf[--len] = '\0';

        if (strcmp(buf, "CMD:SYNC") == 0) {
            ESP_LOGI(TAG, "CMD:SYNC received — triggering immediate sync");
            printf("CMD_ACK:SYNC\n");
            fflush(stdout);
            if (s_sync_task_handle) {
                xTaskNotifyGive(s_sync_task_handle);
            }
        }
        /* Unknown commands are silently ignored */
    }
}

/* ================================================================
   Display task — prints per-node RSSI table and emits machine-readable lines
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

        printf("\n"
               "════════════════════════════════════════════════════════════════════\n\n");

        /* ---- Machine-readable lines consumed by the Python visualizer ----
         * Format:
         *   RSSI|<device_mac>|<is_random>|<ssid>|<node_mac>|<avg_rssi>|<window_id>
         *   DISC|<MAC>            — discovered sniffer node MAC (re-emitted every cycle)
         *   ---FRAME END---
         * Python performs trilateration using node positions set by dragging.
         * Python splits on '|' with maxsplit=6 so SSIDs may contain '|'.  */
        for (int d = 0; d < s_num_disc; d++) {
            printf("DISC|%02X:%02X:%02X:%02X:%02X:%02X\n",
                   s_disc_macs[d][0], s_disc_macs[d][1], s_disc_macs[d][2],
                   s_disc_macs[d][3], s_disc_macs[d][4], s_disc_macs[d][5]);
        }
        for (int n = 0; n < s_node_count; n++) {
            coord_node_t *node = &s_nodes[n];
            for (int d = 0; d < node->device_count; d++) {
                coord_device_t *dev = &node->devices[d];
                printf("RSSI|%02X:%02X:%02X:%02X:%02X:%02X"
                       "|%d|%s"
                       "|%02X:%02X:%02X:%02X:%02X:%02X"
                       "|%d|%lu\n",
                       dev->mac[0], dev->mac[1], dev->mac[2],
                       dev->mac[3], dev->mac[4], dev->mac[5],
                       dev->is_random ? 1 : 0,
                       dev->ssid[0] ? dev->ssid : "<hidden>",
                       node->node_mac[0], node->node_mac[1], node->node_mac[2],
                       node->node_mac[3], node->node_mac[4], node->node_mac[5],
                       dev->avg_rssi,
                       (unsigned long)dev->window_id);
            }
        }
        printf("---FRAME END---\n");

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
    ESP_LOGI(TAG, " RSSI emit: every %d s  — trilateration runs in Python visualizer",
             DISPLAY_PERIOD_MS / 1000);
    ESP_LOGI(TAG, "========================================");

    xTaskCreate(aggregator_task,     "aggregator", 4096, NULL, 5, NULL);
    xTaskCreate(sync_processor_task, "sync_proc",  4096, NULL, 5, NULL);
    xTaskCreate(sync_task,           "sync",       2048, NULL, 4, NULL);
    xTaskCreate(display_task,        "display",    4096, NULL, 2, NULL);
    xTaskCreate(uart_cmd_task,       "uart_cmd",   2048, NULL, 3, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
