#include <stdio.h>
#include <string.h>
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

#define REPORT_CHANNEL    1      /* channel sniffer nodes use for ESP-NOW   */
#define MAX_NODES         8      /* maximum simultaneous sensor nodes        */
#define MAX_DEVICES_NODE  64     /* maximum tracked devices per node          */
#define QUEUE_DEPTH       64     /* incoming report queue depth               */
#define STALE_MS          60000  /* evict node entries unseen for 60 s        */
#define DISPLAY_PERIOD_MS 10000  /* print aggregated table every 10 s         */

/* ================================================================
   Data structures — rssi_report_t MUST match the sniffer's definition
   ================================================================ */

/* Payload received over ESP-NOW from each sniffer node */
typedef struct {
    uint8_t  sender_mac[6];  /* sniffer node's WiFi STA MAC              */
    uint8_t  target_mac[6];  /* sniffed device MAC                        */
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;      /* 1 = locally-administered (randomised) MAC */
} __attribute__((packed)) rssi_report_t;

/* Per-device record stored at the coordinator (one per node) */
typedef struct {
    uint8_t  mac[6];
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;
    uint32_t last_update_ms;
} coord_device_t;

/* Per-node state: a sniffer's MAC + everything it has reported */
typedef struct {
    uint8_t        node_mac[6];
    coord_device_t devices[MAX_DEVICES_NODE];
    uint8_t        device_count;
    uint32_t       last_report_ms;
    uint32_t       total_reports;
} coord_node_t;

static coord_node_t      s_nodes[MAX_NODES];
static uint8_t           s_node_count = 0;
static SemaphoreHandle_t s_table_mutex;
static QueueHandle_t     s_report_queue;

/* ================================================================
   ESP-NOW receive callback — runs in the Wi-Fi task context.
   Keep it short: just enqueue the report.
   ================================================================ */

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
{
    if (len != (int)sizeof(rssi_report_t)) return; /* ignore malformed packets */

    rssi_report_t report;
    memcpy(&report, data, sizeof(rssi_report_t));

    /* Trust the frame's source address rather than what the sender self-reports */
    memcpy(report.sender_mac, recv_info->src_addr, 6);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_report_queue, &report, &woken);
    portYIELD_FROM_ISR(woken);
}

/* ================================================================
   Helpers
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
   Aggregator task — dequeues reports and updates the device tables
   ================================================================ */

static void aggregator_task(void *pvParameters)
{
    rssi_report_t report;
    while (1) {
        if (xQueueReceive(s_report_queue, &report, portMAX_DELAY) != pdTRUE)
            continue;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        coord_node_t *node = find_or_create_node(report.sender_mac);
        if (node == NULL) {
            xSemaphoreGive(s_table_mutex);
            continue;
        }

        node->last_report_ms = now_ms;
        node->total_reports++;

        /* Find or insert the device entry inside this node's table */
        coord_device_t *dev = NULL;
        for (int i = 0; i < node->device_count; i++) {
            if (memcmp(node->devices[i].mac, report.target_mac, 6) == 0) {
                dev = &node->devices[i];
                break;
            }
        }
        if (dev == NULL) {
            if (node->device_count >= MAX_DEVICES_NODE) {
                xSemaphoreGive(s_table_mutex);
                continue; /* node's device table full; discard */
            }
            dev = &node->devices[node->device_count++];
            memset(dev, 0, sizeof(coord_device_t));
            memcpy(dev->mac, report.target_mac, 6);
        }

        /* Update with latest snapshot from the sniffer node */
        dev->avg_rssi       = report.avg_rssi;
        dev->best_channel   = report.best_channel;
        dev->pkt_count      = report.pkt_count;
        dev->is_random      = report.is_random;
        dev->last_update_ms = now_ms;
        if (report.ssid[0] != '\0')
            strncpy(dev->ssid, report.ssid, 32);

        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   Display task — prints a per-node device table every DISPLAY_PERIOD_MS
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
                if ((now_ms - node->devices[i].last_update_ms) > STALE_MS)
                    node->devices[i] = node->devices[--node->device_count];
                else
                    i++;
            }
        }

        /* ---- Print aggregated report ---- */
        printf("\n"
               "╔══════════════════════════════════════════════════════════════════╗\n"
               "║    ESP-NOW Coordinator  [%8lu ms]  %d node(s) active         ║\n"
               "╚══════════════════════════════════════════════════════════════════╝\n",
               (unsigned long)now_ms, s_node_count);

        for (int n = 0; n < s_node_count; n++) {
            coord_node_t *node = &s_nodes[n];

            printf("\n  ┌─ Node %d  MAC: %02X:%02X:%02X:%02X:%02X:%02X"
                   "  reports-rx: %lu  devices: %d\n",
                   n + 1,
                   node->node_mac[0], node->node_mac[1], node->node_mac[2],
                   node->node_mac[3], node->node_mac[4], node->node_mac[5],
                   (unsigned long)node->total_reports,
                   node->device_count);

            if (node->device_count == 0) {
                printf("  │  (no devices tracked yet)\n");
                continue;
            }

            printf("  │  %-3s  %-17s  %-4s  %-3s  %-32s  %-8s  %-6s\n",
                   "#", "Device MAC", "RND", "Ch", "WiFi SSID", "AvgRSSI", "Pkts");
            printf("  │  ---  -----------------  ----  ---"
                   "  --------------------------------  --------  ------\n");

            for (int d = 0; d < node->device_count; d++) {
                coord_device_t *dev = &node->devices[d];
                printf("  │  %-3d  %02X:%02X:%02X:%02X:%02X:%02X  %-4s  %3d"
                       "  %-32s  %4d dBm  %lu\n",
                       d + 1,
                       dev->mac[0], dev->mac[1], dev->mac[2],
                       dev->mac[3], dev->mac[4], dev->mac[5],
                       dev->is_random ? "[R]" : "   ",
                       dev->best_channel,
                       dev->ssid[0] ? dev->ssid : "<hidden>",
                       dev->avg_rssi,
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

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    /* Fix to the reporting channel — no hopping on the coordinator */
    esp_wifi_set_channel(REPORT_CHANNEL, WIFI_SECOND_CHAN_NONE);

    uint8_t self_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, self_mac);

    /* Initialise ESP-NOW; no peers need to be added to receive broadcasts */
    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP-NOW Coordinator ready");
    ESP_LOGI(TAG, " MAC  : %02X:%02X:%02X:%02X:%02X:%02X",
             self_mac[0], self_mac[1], self_mac[2],
             self_mac[3], self_mac[4], self_mac[5]);
    ESP_LOGI(TAG, " Channel : %d (fixed)", REPORT_CHANNEL);
    ESP_LOGI(TAG, " -> Copy this MAC into COORDINATOR_MAC");
    ESP_LOGI(TAG, "    in the sniffer node's main.c");
    ESP_LOGI(TAG, "========================================");

    xTaskCreate(aggregator_task, "aggregator", 4096, NULL, 5, NULL);
    xTaskCreate(display_task,    "display",    4096, NULL, 3, NULL);

    /* Keep app_main alive — all work is done in tasks */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
