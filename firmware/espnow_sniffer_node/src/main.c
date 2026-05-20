#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_mac.h"

/* ================================================================
   Phase 1 — Data structures
   ================================================================ */

/* ---- ESP-NOW configuration ---- */
/* Channel used for ESP-NOW reporting (all nodes must agree) */
#define REPORT_CHANNEL  1
/* Coordinator MAC — replace with actual coordinator MAC once known.
   Default 0xFF*6 = broadcast, works for single-node testing.        */
static const uint8_t COORDINATOR_MAC[6] = {0x0C,0x4E,0xA0,0x6F,0x61,0x94};

#define MAX_DEVICES   64
#define QUEUE_DEPTH   32
#define MGMT_HDR_LEN  24      /* fixed part of 802.11 management header */
#define STALE_MS      30000   /* evict entries unseen for 30 s           */
#define RSSI_REF_DBM  (-40)   /* reference RSSI at 1 m                  */
#define PATH_LOSS_N   2.5f    /* indoor path loss exponent               */

typedef struct {
    uint8_t  mac[6];
    bool     is_random;       /* locally-administered MAC (randomised)   */
    char     ssid[33];        /* from Probe Req / Beacon IE              */
    int8_t   rssi_last;
    int8_t   rssi_min;
    int8_t   rssi_max;
    int32_t  rssi_sum;
    uint32_t pkt_count;
    uint32_t last_seen_ms;
    uint8_t  best_channel;
} device_entry_t;

typedef struct {
    uint8_t mac[6];
    int8_t  rssi;
    uint8_t channel;
    uint8_t ftype;
    uint8_t fsubtype;
    char    ssid[33];
} pkt_info_t;

/* Payload sent over ESP-NOW to the coordinator */
typedef struct {
    uint8_t  sender_mac[6];   /* this sensor node's WiFi STA MAC       */
    uint8_t  target_mac[6];   /* sniffed device MAC                     */
    int8_t   avg_rssi;
    uint8_t  best_channel;
    uint32_t pkt_count;
    char     ssid[33];
    uint8_t  is_random;
} __attribute__((packed)) rssi_report_t;

static device_entry_t s_devices[MAX_DEVICES];
static uint8_t        s_device_count = 0;
static SemaphoreHandle_t s_table_mutex;
static QueueHandle_t     s_pkt_queue;
static uint8_t           s_self_mac[6];      /* populated in app_main  */
static volatile bool     s_hopper_pause = false;

/* ================================================================
   802.11 frame structures
   ================================================================ */

typedef struct {
    uint8_t version        : 2;
    uint8_t type           : 2;
    uint8_t subtype        : 4;
    uint8_t to_ds          : 1;
    uint8_t from_ds        : 1;
    uint8_t more_frag      : 1;
    uint8_t retry          : 1;
    uint8_t pwr_mgmt       : 1;
    uint8_t more_data      : 1;
    uint8_t protected_frame: 1;
    uint8_t order          : 1;
} __attribute__((packed)) frame_ctrl_t;

typedef struct {
    frame_ctrl_t frame_ctrl;
    uint16_t     duration;
    uint8_t      addr1[6];
    uint8_t      addr2[6];
    uint8_t      addr3[6];
    uint16_t     seq_ctrl;
} __attribute__((packed)) ieee80211_hdr_t;

/* ================================================================
   Phase 2 — Promiscuous callback (ISR-context: no printf, no alloc)
   ================================================================ */

static void parse_ssid_ie(const uint8_t *payload, uint16_t payload_len, char *out_ssid)
{
    out_ssid[0] = '\0';
    if (payload_len <= MGMT_HDR_LEN) return;

    const uint8_t *ie     = payload + MGMT_HDR_LEN;
    uint16_t       offset = MGMT_HDR_LEN;

    /* Beacons have an 8-byte timestamp + 2-byte interval + 2-byte capability
       before the IEs; Probe Requests go straight to IEs after the fixed header */
    /* We handle both: search for IE tag 0 (SSID) within first 128 bytes */
    uint16_t limit = (payload_len < (uint16_t)(MGMT_HDR_LEN + 128))
                     ? payload_len : (uint16_t)(MGMT_HDR_LEN + 128);

    while (offset + 2 <= limit) {
        uint8_t tag = ie[0];
        uint8_t len = ie[1];
        if (offset + 2 + len > payload_len) break;
        if (tag == 0 && len > 0 && len <= 32) {
            memcpy(out_ssid, &ie[2], len);
            out_ssid[len] = '\0';
            return;
        }
        ie     += 2 + len;
        offset += 2 + len;
    }
}

static void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t pkt_type)
{
    if (pkt_type != WIFI_PKT_MGMT && pkt_type != WIFI_PKT_DATA) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const ieee80211_hdr_t        *hdr = (const ieee80211_hdr_t *)pkt->payload;

    /* Skip ESP-NOW action frames (type=0, subtype=13) to avoid logging
       our own reports and coordinator replies as tracked devices */
    if (hdr->frame_ctrl.type == 0 && (hdr->frame_ctrl.subtype & 0x0F) == 13) return;

    pkt_info_t info;
    info.ftype    = hdr->frame_ctrl.type;
    info.fsubtype = hdr->frame_ctrl.subtype & 0x0F;
    info.rssi     = pkt->rx_ctrl.rssi;
    info.channel  = pkt->rx_ctrl.channel;
    info.ssid[0]  = '\0';
    memcpy(info.mac, hdr->addr2, 6);

    /* Extract SSID from Probe Request (4) and Beacon (8) */
    if (info.ftype == 0 && (info.fsubtype == 4 || info.fsubtype == 8)) {
        parse_ssid_ie(pkt->payload, pkt->rx_ctrl.sig_len, info.ssid);
    }

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_pkt_queue, &info, &woken);
    /* Drop silently if queue full — never block in callback */
    portYIELD_FROM_ISR(woken);
}

/* ================================================================
   Phase 3 — Packet processor task
   ================================================================ */

static void packet_processor_task(void *pvParameters)
{
    pkt_info_t info;
    while (1) {
        if (xQueueReceive(s_pkt_queue, &info, portMAX_DELAY) != pdTRUE) continue;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        /* Search for existing entry */
        device_entry_t *entry = NULL;
        for (int i = 0; i < s_device_count; i++) {
            if (memcmp(s_devices[i].mac, info.mac, 6) == 0) {
                entry = &s_devices[i];
                break;
            }
        }

        /* Insert new entry if not found and table not full */
        if (entry == NULL && s_device_count < MAX_DEVICES) {
            entry = &s_devices[s_device_count++];
            memset(entry, 0, sizeof(device_entry_t));
            memcpy(entry->mac, info.mac, 6);
            entry->is_random  = (info.mac[0] & 0x02) != 0;
            entry->rssi_min   = info.rssi;
            entry->rssi_max   = info.rssi;
        }

        if (entry != NULL) {
            entry->rssi_last    = info.rssi;
            entry->rssi_sum    += info.rssi;
            entry->pkt_count++;
            entry->last_seen_ms = now_ms;
            if (info.rssi < entry->rssi_min) entry->rssi_min = info.rssi;
            if (info.rssi > entry->rssi_max) entry->rssi_max = info.rssi;
            if (info.channel > 0)            entry->best_channel = info.channel;
            if (info.ssid[0] != '\0')        strncpy(entry->ssid, info.ssid, 32);
        }

        xSemaphoreGive(s_table_mutex);
    }
}

/* ================================================================
   Phase 4 — Reporter task
   ================================================================ */

static void device_reporter_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        /* Evict stale entries (compact the array) */
        int i = 0;
        while (i < s_device_count) {
            if ((now_ms - s_devices[i].last_seen_ms) > STALE_MS) {
                s_devices[i] = s_devices[--s_device_count];
            } else {
                i++;
            }
        }

        printf("\n===== Device Tracker  [%lu ms]  %d device(s) =====\n",
               (unsigned long)now_ms, s_device_count);
        printf("Node  %02X:%02X:%02X:%02X:%02X:%02X  "
               "-> Coordinator %02X:%02X:%02X:%02X:%02X:%02X (ch %d)\n",
               s_self_mac[0], s_self_mac[1], s_self_mac[2],
               s_self_mac[3], s_self_mac[4], s_self_mac[5],
               COORDINATOR_MAC[0], COORDINATOR_MAC[1], COORDINATOR_MAC[2],
               COORDINATOR_MAC[3], COORDINATOR_MAC[4], COORDINATOR_MAC[5],
               REPORT_CHANNEL);
        printf("%-3s %-17s %-4s %-3s %-32s %-8s %-5s %-5s %-7s\n",
               "#", "MAC", "RND", "Ch", "WiFi SSID",
               "AvgRSSI", "Min", "Max", "~Dist");
        printf("---  -----------------  ----  ---  "
               "--------------------------------  "
               "--------  -----  -----  -------\n");

        /* Snapshot entries for ESP-NOW send (done outside mutex) */
        static rssi_report_t reports[MAX_DEVICES];
        int report_count = 0;

        for (int j = 0; j < s_device_count; j++) {
            device_entry_t *e = &s_devices[j];
            int8_t avg_rssi   = (int8_t)(e->rssi_sum / (int32_t)e->pkt_count);

            float dist = powf(10.0f,
                              (float)(RSSI_REF_DBM - avg_rssi) / (10.0f * PATH_LOSS_N));

            printf("%-3d %02X:%02X:%02X:%02X:%02X:%02X  %-4s  %3d  %-32s  "
                   "%4d dBm  %4d   %4d  ~%.1fm\n",
                   j + 1,
                   e->mac[0], e->mac[1], e->mac[2],
                   e->mac[3], e->mac[4], e->mac[5],
                   e->is_random ? "[R]" : "   ",
                   e->best_channel,
                   e->ssid[0] ? e->ssid : "<hidden>",
                   avg_rssi,
                   e->rssi_min,
                   e->rssi_max,
                   dist);

            /* Build report for this entry */
            rssi_report_t *r = &reports[report_count++];
            memcpy(r->sender_mac, s_self_mac, 6);
            memcpy(r->target_mac, e->mac, 6);
            r->avg_rssi    = avg_rssi;
            r->best_channel = e->best_channel;
            r->pkt_count   = e->pkt_count;
            r->is_random   = e->is_random ? 1 : 0;
            strncpy(r->ssid, e->ssid, 32);
            r->ssid[32] = '\0';
        }
        printf("=================================================\n\n");

        xSemaphoreGive(s_table_mutex);

        /* ---- ESP-NOW reporting window ---- */
        /* Pause hopper, switch to report channel, send all snapshots */
        if (report_count > 0) {
            s_hopper_pause = true;
            esp_wifi_set_channel(REPORT_CHANNEL, WIFI_SECOND_CHAN_NONE);
            vTaskDelay(pdMS_TO_TICKS(10)); /* settle */

            for (int k = 0; k < report_count; k++) {
                esp_now_send(COORDINATOR_MAC,
                             (const uint8_t *)&reports[k],
                             sizeof(rssi_report_t));
                vTaskDelay(pdMS_TO_TICKS(2)); /* avoid flooding coordinator */
            }

            s_hopper_pause = false;
        }
    }
}

/* ================================================================
   Channel hopper
   ================================================================ */

static void channel_hop_task(void *pvParameters)
{
    uint8_t channel = 1;
    while (1) {
        if (!s_hopper_pause) {
            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
            channel = (channel % 13) + 1;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ================================================================
   Phase 5 — app_main
   ================================================================ */

void app_main(void)
{
    /* NVS required by WiFi driver */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Create synchronisation primitives */
    s_table_mutex = xSemaphoreCreateMutex();
    s_pkt_queue   = xQueueCreate(QUEUE_DEPTH, sizeof(pkt_info_t));

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);   /* STA required for ESP-NOW */
    esp_wifi_start();

    /* Read this node's own MAC address for use in reports */
    esp_wifi_get_mac(WIFI_IF_STA, s_self_mac);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);

    /* ---- ESP-NOW init ---- */
    esp_now_init();
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, COORDINATOR_MAC, 6);
    peer.channel = REPORT_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    printf("[*] WiFi device tracker started\n");
    printf("[*] Node MAC : %02X:%02X:%02X:%02X:%02X:%02X\n",
           s_self_mac[0], s_self_mac[1], s_self_mac[2],
           s_self_mac[3], s_self_mac[4], s_self_mac[5]);
    printf("[*] Coordinator: %02X:%02X:%02X:%02X:%02X:%02X  (ch %d)\n",
           COORDINATOR_MAC[0], COORDINATOR_MAC[1], COORDINATOR_MAC[2],
           COORDINATOR_MAC[3], COORDINATOR_MAC[4], COORDINATOR_MAC[5],
           REPORT_CHANNEL);
    printf("[*] Hopping channels 1-13, reporting every 5 s\n\n");

    xTaskCreate(packet_processor_task, "pkt_proc",  4096, NULL, 5, NULL);
    xTaskCreate(device_reporter_task,  "reporter",  4096, NULL, 3, NULL);
    xTaskCreate(channel_hop_task,      "ch_hop",    2048, NULL, 4, NULL);

    /* Keep app_main alive — all work done in tasks */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

