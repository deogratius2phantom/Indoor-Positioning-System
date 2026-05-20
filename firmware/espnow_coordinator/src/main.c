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
#define DISPLAY_PERIOD_MS      10000   /* print aggregated table every 10 s    */
#define SYNC_PERIOD_MS         30000   /* start a new sync cycle every 30 s    */
#define SYNC_BCAST_INTERVAL_MS 500     /* gap between repeated epoch broadcasts */
/* 14 broadcasts × 500 ms = 7 s — covers one full 13-channel hop cycle (6.5 s)
   so every hopping node is guaranteed to receive at least one SYNC_EPOCH.    */
#define SYNC_BCAST_COUNT       14

/* ================================================================
   Message types — MUST match espnow_sniffer_node
   ================================================================ */

#define MSG_RSSI_REPORT  0x01
#define MSG_SYNC_EPOCH   0x02
#define MSG_SYNC_ACK     0x03

/* ================================================================
   Wire structures (packed — shared between coordinator & sniffer)
   ================================================================ */

/* Coordinator → all nodes (broadcast) */
typedef struct {
    uint8_t  msg_type;       /* MSG_SYNC_EPOCH                              */
    uint32_t epoch_id;       /* monotonically increasing counter            */
    uint32_t coord_T1_ms;    /* coordinator clock at the moment of send     */
} __attribute__((packed)) sync_epoch_t;

/* Node → coordinator */
typedef struct {
    uint8_t  msg_type;       /* MSG_SYNC_ACK                                */
    uint32_t epoch_id;       /* echoed from SYNC_EPOCH                      */
    uint32_t coord_T1_ms;    /* echoed coordinator T1                       */
    uint32_t node_T2_ms;     /* node local clock at reception of epoch      */
    uint32_t node_T3_ms;     /* node local clock at moment ACK is sent      */
} __attribute__((packed)) sync_ack_t;

/* Node → coordinator (per-device RSSI snapshot) */
typedef struct {
    uint8_t  msg_type;            /* MSG_RSSI_REPORT                        */
    uint8_t  sender_mac[6];       /* sniffer node's WiFi STA MAC            */
    uint8_t  target_mac[6];       /* sniffed device MAC                     */
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;           /* 1 = locally-administered (random) MAC  */
    uint32_t window_id;           /* epoch-aligned reporting window counter  */
    uint32_t node_timestamp_ms;   /* node local clock when window closed     */
} __attribute__((packed)) rssi_report_t;

/* ================================================================
   Internal coordinator structures
   ================================================================ */

/* Per-device record stored at the coordinator (one entry per node) */
typedef struct {
    uint8_t  mac[6];
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;
    uint32_t corrected_ts_ms; /* node timestamp normalised to coordinator time */
    uint32_t window_id;
} coord_device_t;

/* Per-node state: one sniffer + everything it has reported */
typedef struct {
    uint8_t        node_mac[6];
    coord_device_t devices[MAX_DEVICES_NODE];
    uint8_t        device_count;
    uint32_t       last_report_ms;
    uint32_t       total_reports;
    /* --- time-sync state --- */
    int32_t        clock_offset_ms; /* node_time − coord_time; apply: coord = node − offset */
    uint32_t       last_sync_ms;    /* coordinator time when sync last completed */
    bool           sync_valid;      /* true once at least one sync round-trip done */
} coord_node_t;

/* Internal envelope to carry SYNC_ACK + source MAC through the queue */
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
static QueueHandle_t     s_report_queue; /* rssi_report_t items  */
static QueueHandle_t     s_ack_queue;    /* ack_envelope_t items */
static volatile uint32_t s_epoch_id = 0;

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
                /* Trust the frame's source address over the self-reported MAC */
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

        default:
            break;
    }

    portYIELD_FROM_ISR(woken);
}

/* ================================================================
   Aggregator task — dequeues RSSI reports, updates device tables
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

        /* Normalise the node's timestamp to coordinator time using the stored
           clock offset.  Falls back to coordinator reception time if no sync
           has completed yet for this node.                                    */
        uint32_t corrected_ts = node->sync_valid
            ? (uint32_t)((int32_t)report.node_timestamp_ms - node->clock_offset_ms)
            : recv_ms;

        /* Find or insert the device entry inside this node's table */
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

        dev->avg_rssi       = report.avg_rssi;
        dev->best_channel   = report.best_channel;
        dev->pkt_count      = report.pkt_count;
        dev->is_random      = report.is_random;
        dev->corrected_ts_ms = corrected_ts;
        dev->window_id      = report.window_id;
        if (report.ssid[0] != '\0')
            strncpy(dev->ssid, report.ssid, 32);

        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   Sync-processor task — dequeues SYNC_ACKs, computes per-node offsets
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

        /* NTP offset formula:
             RTT    = (T4 − T1) − (T3 − T2)   — round-trip excluding node processing
             offset = T2 − T1 − RTT/2          — node_time − coord_time
           To convert a node timestamp to coordinator time:
             coord_time = node_time − offset                                   */
        int32_t rtt_ms    = (int32_t)(T4 - T1) - (int32_t)(T3 - T2);
        if (rtt_ms < 0) rtt_ms = 0;                       /* sanity clamp     */
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
    /* Brief startup delay — let Wi-Fi and nodes stabilise first */
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        s_epoch_id++;
        ESP_LOGI(TAG, "Sync cycle start  epoch_id=%lu", (unsigned long)s_epoch_id);

        /* Repeat the broadcast over one full channel-hop cycle (13 ch × 500 ms = 6.5 s)
           so that every hopping node is guaranteed to receive at least one epoch.
           T1 is refreshed each iteration so the timestamp stays accurate.            */
        for (int i = 0; i < SYNC_BCAST_COUNT; i++) {
            sync_epoch_t epoch = {
                .msg_type    = MSG_SYNC_EPOCH,
                .epoch_id    = s_epoch_id,
                .coord_T1_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
            };
            esp_now_send(BROADCAST_MAC, (const uint8_t *)&epoch, sizeof(sync_epoch_t));
            vTaskDelay(pdMS_TO_TICKS(SYNC_BCAST_INTERVAL_MS));
        }

        ESP_LOGI(TAG, "Sync cycle done.  Next in %d s", SYNC_PERIOD_MS / 1000);
        vTaskDelay(pdMS_TO_TICKS(SYNC_PERIOD_MS));
    }
}

/* ================================================================
   Display task — prints per-node device table every DISPLAY_PERIOD_MS
   ================================================================ */

static void display_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        /* Evict stale device entries from every node's table */
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

        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   app_main
   ================================================================ */

void app_main(void)
{
    /* NVS required by Wi-Fi driver */
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

    /* Fixed to the reporting channel — coordinator never hops */
    esp_wifi_set_channel(REPORT_CHANNEL, WIFI_SECOND_CHAN_NONE);

    uint8_t self_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, self_mac);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    /* Broadcast peer — required to send SYNC_EPOCH to all nodes at once */
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
    ESP_LOGI(TAG, " Sync    : every %d s  (%d broadcasts × %d ms)",
             SYNC_PERIOD_MS / 1000, SYNC_BCAST_COUNT, SYNC_BCAST_INTERVAL_MS);
    ESP_LOGI(TAG, " -> Copy this MAC into COORDINATOR_MAC in sniffer main.c");
    ESP_LOGI(TAG, "========================================");

    xTaskCreate(aggregator_task,     "aggregator", 4096, NULL, 5, NULL);
    xTaskCreate(sync_processor_task, "sync_proc",  4096, NULL, 5, NULL);
    xTaskCreate(sync_task,           "sync",       2048, NULL, 4, NULL);
    xTaskCreate(display_task,        "display",    4096, NULL, 3, NULL);

    /* Keep app_main alive — all work is done in tasks */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
