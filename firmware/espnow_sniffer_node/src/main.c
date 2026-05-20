#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

/* ================================================================
   Configuration
   ================================================================ */

#define REPORT_CHANNEL    1        /* channel used for all ESP-NOW comms        */
#define REPORT_PERIOD_MS  5000     /* duration of each reporting window (5 s)   */
#define MAX_DEVICES       64
#define QUEUE_DEPTH       32
#define SYNC_QUEUE_DEPTH  4
#define MGMT_HDR_LEN      24
#define STALE_MS          30000
#define RSSI_REF_DBM      (-40)
#define PATH_LOSS_N       2.5f

/* Coordinator MAC — replace with actual coordinator MAC once known. */
static const uint8_t COORDINATOR_MAC[6] = {0x0C, 0x4E, 0xA0, 0x6F, 0x61, 0x94};

/* ================================================================
   RGB LED Configuration  (ESP32-C3-DevKitM-1: WS2812 on GPIO 8)
   ================================================================ */

#define LED_GPIO          8
#define LED_RMT_RES_HZ    10000000   /* 10 MHz = 100 ns/tick, suits WS2812 */

/*  Steady colours (dim — easy on the eyes)
    WS2812 wire order is G-R-B; set_led() handles the swap.           */
#define LED_STEADY_R_RED   50    /* red   = no sync yet                  */
#define LED_STEADY_G_RED   0
#define LED_STEADY_B_RED   0

#define LED_STEADY_R_BLUE  0    /* blue  = sync valid, idle              */
#define LED_STEADY_G_BLUE  0
#define LED_STEADY_B_BLUE  50

/*  Flash colours (brighter, short duration)                           */
#define LED_FLASH_R_GREEN  0    /* green = sync epoch received  (500 ms) */
#define LED_FLASH_G_GREEN  100
#define LED_FLASH_B_GREEN  0

#define LED_FLASH_R_WHITE  80   /* white = ESP-NOW packet sent  (150 ms) */
#define LED_FLASH_G_WHITE  80
#define LED_FLASH_B_WHITE  80

#define LED_FLASH_SYNC_MS  500
#define LED_FLASH_TX_MS    150

/* ================================================================
   Message types — MUST match espnow_coordinator
   ================================================================ */

#define MSG_RSSI_REPORT  0x01
#define MSG_SYNC_EPOCH   0x02
#define MSG_SYNC_ACK     0x03

/* ================================================================
   Wire structures (packed — shared between coordinator & sniffer)
   ================================================================ */

typedef struct {
    uint8_t  msg_type;
    uint32_t epoch_id;
    uint32_t coord_T1_ms;
} __attribute__((packed)) sync_epoch_t;

typedef struct {
    uint8_t  msg_type;
    uint32_t epoch_id;
    uint32_t coord_T1_ms;
    uint32_t node_T2_ms;
    uint32_t node_T3_ms;
} __attribute__((packed)) sync_ack_t;

typedef struct {
    uint8_t  msg_type;
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
   Internal sniffer structures
   ================================================================ */

typedef struct {
    uint8_t  mac[6];
    bool     is_random;
    char     ssid[33];
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

typedef struct {
    sync_epoch_t epoch;
    uint32_t     recv_T2_ms;
} epoch_envelope_t;

/* ================================================================
   LED command type
   ================================================================ */

typedef enum {
    LED_CMD_NO_SYNC    = 0,   /* base → red  (no sync received yet)       */
    LED_CMD_SYNC_VALID,       /* base → blue (sync valid, normal op)       */
    LED_CMD_FLASH_SYNC,       /* green flash for LED_FLASH_SYNC_MS         */
    LED_CMD_FLASH_TX,         /* white flash for LED_FLASH_TX_MS           */
} led_cmd_t;

/* ================================================================
   Globals
   ================================================================ */

static device_entry_t    s_devices[MAX_DEVICES];
static uint8_t           s_device_count = 0;
static SemaphoreHandle_t s_table_mutex;
static QueueHandle_t     s_pkt_queue;
static QueueHandle_t     s_sync_queue;
static QueueHandle_t     s_led_queue;
static uint8_t           s_self_mac[6];
static volatile bool     s_hopper_pause = false;

static volatile uint32_t s_epoch_anchor_ms = 0;
static volatile uint32_t s_epoch_id        = 0;
static volatile bool     s_epoch_valid     = false;

/* RMT handles — initialised in app_main, used by set_led() */
static rmt_channel_handle_t s_led_chan    = NULL;
static rmt_encoder_handle_t s_led_encoder = NULL;

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
   SSID IE parser
   ================================================================ */

static void parse_ssid_ie(const uint8_t *payload, uint16_t payload_len, char *out_ssid)
{
    out_ssid[0] = '\0';
    if (payload_len <= MGMT_HDR_LEN) return;

    const uint8_t *ie     = payload + MGMT_HDR_LEN;
    uint16_t       offset = MGMT_HDR_LEN;
    uint16_t       limit  = (payload_len < (uint16_t)(MGMT_HDR_LEN + 128))
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

/* ================================================================
   LED helpers
   ================================================================ */

/* Write one colour to the onboard WS2812.
   WS2812 wire order is G-R-B; this function accepts plain R-G-B.   */
static void set_led(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_chan || !s_led_encoder) return;
    uint8_t grb[3] = {g, r, b};
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    rmt_transmit(s_led_chan, s_led_encoder, grb, sizeof(grb), &tx_cfg);
    rmt_tx_wait_all_done(s_led_chan, pdMS_TO_TICKS(50));
}

/* LED task — owns all LED state; driven by s_led_queue.
   Keeps a "base" colour (red or blue) and handles timed colour
   flashes, reverting to base once the flash duration expires.       */
static void led_task(void *pvParameters)
{
    led_cmd_t cmd;
    uint8_t base_r = LED_STEADY_R_RED;
    uint8_t base_g = LED_STEADY_G_RED;
    uint8_t base_b = LED_STEADY_B_RED;
    uint32_t flash_end_ms = 0;
    bool flashing = false;

    set_led(base_r, base_g, base_b);   /* start red — no sync yet */

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* Expire any active flash */
        if (flashing && now_ms >= flash_end_ms) {
            flashing = false;
            set_led(base_r, base_g, base_b);
        }

        /* Block until next command or until the flash needs to expire */
        TickType_t wait;
        if (flashing) {
            uint32_t rem = flash_end_ms - (uint32_t)(esp_timer_get_time() / 1000ULL);
            wait = pdMS_TO_TICKS(rem < 10 ? 10 : rem);
        } else {
            wait = portMAX_DELAY;
        }

        if (xQueueReceive(s_led_queue, &cmd, wait) != pdTRUE) continue;

        now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        switch (cmd) {

            case LED_CMD_NO_SYNC:
                base_r = LED_STEADY_R_RED;
                base_g = LED_STEADY_G_RED;
                base_b = LED_STEADY_B_RED;
                if (!flashing) set_led(base_r, base_g, base_b);
                break;

            case LED_CMD_SYNC_VALID:
                base_r = LED_STEADY_R_BLUE;
                base_g = LED_STEADY_G_BLUE;
                base_b = LED_STEADY_B_BLUE;
                if (!flashing) set_led(base_r, base_g, base_b);
                break;

            case LED_CMD_FLASH_SYNC:
                /* green flash — always overrides a TX flash */
                set_led(LED_FLASH_R_GREEN, LED_FLASH_G_GREEN, LED_FLASH_B_GREEN);
                flash_end_ms = now_ms + LED_FLASH_SYNC_MS;
                flashing = true;
                break;

            case LED_CMD_FLASH_TX:
                /* white flash — only if no longer flash is already running */
                if (!flashing) {
                    set_led(LED_FLASH_R_WHITE, LED_FLASH_G_WHITE, LED_FLASH_B_WHITE);
                    flash_end_ms = now_ms + LED_FLASH_TX_MS;
                    flashing = true;
                }
                break;
        }
    }
}

/* ================================================================
   Promiscuous callback (Wi-Fi task context — no alloc, no printf)
   ================================================================ */

static void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t pkt_type)
{
    if (pkt_type != WIFI_PKT_MGMT && pkt_type != WIFI_PKT_DATA) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const ieee80211_hdr_t        *hdr = (const ieee80211_hdr_t *)pkt->payload;

    /* Skip ESP-NOW action frames so we don't log our own transmissions */
    if (hdr->frame_ctrl.type == 0 && (hdr->frame_ctrl.subtype & 0x0F) == 13) return;

    pkt_info_t info;
    info.ftype    = hdr->frame_ctrl.type;
    info.fsubtype = hdr->frame_ctrl.subtype & 0x0F;
    info.rssi     = pkt->rx_ctrl.rssi;
    info.channel  = pkt->rx_ctrl.channel;
    info.ssid[0]  = '\0';
    memcpy(info.mac, hdr->addr2, 6);

    if (info.ftype == 0 && (info.fsubtype == 4 || info.fsubtype == 8))
        parse_ssid_ie(pkt->payload, pkt->rx_ctrl.sig_len, info.ssid);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_pkt_queue, &info, &woken);
    portYIELD_FROM_ISR(woken);
}

/* ================================================================
   ESP-NOW receive callback (Wi-Fi task context — enqueue only)
   ================================================================ */

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
{
    if (len < 1) return;

    if (data[0] == MSG_SYNC_EPOCH && len == (int)sizeof(sync_epoch_t)) {
        epoch_envelope_t env;
        memcpy(&env.epoch, data, sizeof(sync_epoch_t));
        env.recv_T2_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(s_sync_queue, &env, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ================================================================
   ESP-NOW send callback — fires after every successful TX.
   Signals the LED task to show a brief white flash.
   ================================================================ */

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
    led_cmd_t cmd = LED_CMD_FLASH_TX;
    xQueueSend(s_led_queue, &cmd, 0);   /* non-blocking; task context */
}

/* ================================================================
   Packet processor task
   ================================================================ */

static void packet_processor_task(void *pvParameters)
{
    pkt_info_t info;
    while (1) {
        if (xQueueReceive(s_pkt_queue, &info, portMAX_DELAY) != pdTRUE) continue;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        device_entry_t *entry = NULL;
        for (int i = 0; i < s_device_count; i++) {
            if (memcmp(s_devices[i].mac, info.mac, 6) == 0) {
                entry = &s_devices[i];
                break;
            }
        }

        if (entry == NULL && s_device_count < MAX_DEVICES) {
            entry = &s_devices[s_device_count++];
            memset(entry, 0, sizeof(device_entry_t));
            memcpy(entry->mac, info.mac, 6);
            entry->is_random = (info.mac[0] & 0x02) != 0;
            entry->rssi_min  = info.rssi;
            entry->rssi_max  = info.rssi;
        }

        if (entry) {
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
   Sync handler task — processes SYNC_EPOCH, sends SYNC_ACK,
   updates LED to show sync status.
   ================================================================ */

static void sync_handler_task(void *pvParameters)
{
    epoch_envelope_t env;
    uint32_t         last_processed_epoch = 0;

    while (1) {
        if (xQueueReceive(s_sync_queue, &env, portMAX_DELAY) != pdTRUE) continue;

        /* Skip duplicate retransmissions of the same epoch */
        if (env.epoch.epoch_id == last_processed_epoch) continue;
        last_processed_epoch = env.epoch.epoch_id;

        uint32_t T2 = env.recv_T2_ms;

        s_epoch_anchor_ms = T2;
        s_epoch_id        = env.epoch.epoch_id;
        s_epoch_valid     = true;

        /* ---- LED: update base to blue, then flash green ---- */
        led_cmd_t led_blue  = LED_CMD_SYNC_VALID;
        led_cmd_t led_flash = LED_CMD_FLASH_SYNC;
        xQueueSend(s_led_queue, &led_blue,  0);
        xQueueSend(s_led_queue, &led_flash, 0);

        /* Switch to reporting channel and send ACK */
        s_hopper_pause = true;
        esp_wifi_set_channel(REPORT_CHANNEL, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(5));

        sync_ack_t ack = {
            .msg_type    = MSG_SYNC_ACK,
            .epoch_id    = env.epoch.epoch_id,
            .coord_T1_ms = env.epoch.coord_T1_ms,
            .node_T2_ms  = T2,
            .node_T3_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };
        esp_now_send(COORDINATOR_MAC, (const uint8_t *)&ack, sizeof(sync_ack_t));
        /* espnow_send_cb will fire here and flash white for the ACK TX */

        s_hopper_pause = false;

        printf("[sync] Epoch %lu  T1(coord)=%lu ms  T2(local)=%lu ms  ACK sent\n",
               (unsigned long)env.epoch.epoch_id,
               (unsigned long)env.epoch.coord_T1_ms,
               (unsigned long)T2);
    }
}

/* ================================================================
   Device reporter task — epoch-aligned RSSI reports
   ================================================================ */

static void device_reporter_task(void *pvParameters)
{
    while (1) {
        uint32_t wait_ms = REPORT_PERIOD_MS;

        if (s_epoch_valid) {
            uint32_t now     = (uint32_t)(esp_timer_get_time() / 1000ULL);
            uint32_t elapsed = (now - s_epoch_anchor_ms) % REPORT_PERIOD_MS;
            wait_ms          = REPORT_PERIOD_MS - elapsed;
            if (wait_ms < 100) wait_ms += REPORT_PERIOD_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(wait_ms));

        uint32_t now_ms    = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t window_id = s_epoch_valid
            ? (now_ms - s_epoch_anchor_ms) / REPORT_PERIOD_MS
            : 0;

        xSemaphoreTake(s_table_mutex, portMAX_DELAY);

        int i = 0;
        while (i < s_device_count) {
            if ((now_ms - s_devices[i].last_seen_ms) > STALE_MS)
                s_devices[i] = s_devices[--s_device_count];
            else
                i++;
        }

        printf("\n===== Device Tracker  [%lu ms]  window=%lu  %d device(s) =====\n",
               (unsigned long)now_ms, (unsigned long)window_id, s_device_count);
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

        static rssi_report_t reports[MAX_DEVICES];
        int report_count = 0;

        for (int j = 0; j < s_device_count; j++) {
            device_entry_t *e   = &s_devices[j];
            int8_t avg_rssi     = (int8_t)(e->rssi_sum / (int32_t)e->pkt_count);
            float  dist         = powf(10.0f,
                                       (float)(RSSI_REF_DBM - avg_rssi) /
                                       (10.0f * PATH_LOSS_N));

            printf("%-3d %02X:%02X:%02X:%02X:%02X:%02X  %-4s  %3d  %-32s  "
                   "%4d dBm  %4d   %4d  ~%.1fm\n",
                   j + 1,
                   e->mac[0], e->mac[1], e->mac[2],
                   e->mac[3], e->mac[4], e->mac[5],
                   e->is_random ? "[R]" : "   ",
                   e->best_channel,
                   e->ssid[0] ? e->ssid : "<hidden>",
                   avg_rssi, e->rssi_min, e->rssi_max, dist);

            rssi_report_t *r     = &reports[report_count++];
            r->msg_type          = MSG_RSSI_REPORT;
            memcpy(r->sender_mac, s_self_mac, 6);
            memcpy(r->target_mac, e->mac, 6);
            r->avg_rssi          = avg_rssi;
            r->best_channel      = e->best_channel;
            r->pkt_count         = e->pkt_count;
            r->is_random         = e->is_random ? 1 : 0;
            r->window_id         = window_id;
            r->node_timestamp_ms = now_ms;
            strncpy(r->ssid, e->ssid, 32);
            r->ssid[32]          = '\0';
        }
        printf("=================================================\n\n");

        xSemaphoreGive(s_table_mutex);

        /* ---- ESP-NOW reporting window ----
           espnow_send_cb fires after each send and flashes white.    */
        if (report_count > 0) {
            s_hopper_pause = true;
            esp_wifi_set_channel(REPORT_CHANNEL, WIFI_SECOND_CHAN_NONE);
            vTaskDelay(pdMS_TO_TICKS(10));

            for (int k = 0; k < report_count; k++) {
                esp_now_send(COORDINATOR_MAC,
                             (const uint8_t *)&reports[k],
                             sizeof(rssi_report_t));
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            s_hopper_pause = false;
        }
    }
}

/* ================================================================
   Channel hopper task
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
   app_main
   ================================================================ */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_table_mutex = xSemaphoreCreateMutex();
    s_pkt_queue   = xQueueCreate(QUEUE_DEPTH,      sizeof(pkt_info_t));
    s_sync_queue  = xQueueCreate(SYNC_QUEUE_DEPTH, sizeof(epoch_envelope_t));
    s_led_queue   = xQueueCreate(8,                sizeof(led_cmd_t));

    /* ---- RMT + WS2812 init ---- */
    rmt_tx_channel_config_t tx_chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz     = LED_RMT_RES_HZ,
        .trans_queue_depth = 4,
    };
    rmt_new_tx_channel(&tx_chan_cfg, &s_led_chan);

    led_strip_encoder_config_t enc_cfg = { .resolution = LED_RMT_RES_HZ };
    rmt_new_led_strip_encoder(&enc_cfg, &s_led_encoder);
    rmt_enable(s_led_chan);

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    esp_wifi_get_mac(WIFI_IF_STA, s_self_mac);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);
    esp_now_register_send_cb(espnow_send_cb);   /* TX indicator */

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, COORDINATOR_MAC, 6);
    peer.channel = REPORT_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    printf("[*] Sniffer node started\n");
    printf("[*] Node MAC    : %02X:%02X:%02X:%02X:%02X:%02X\n",
           s_self_mac[0], s_self_mac[1], s_self_mac[2],
           s_self_mac[3], s_self_mac[4], s_self_mac[5]);
    printf("[*] Coordinator : %02X:%02X:%02X:%02X:%02X:%02X  (ch %d)\n",
           COORDINATOR_MAC[0], COORDINATOR_MAC[1], COORDINATOR_MAC[2],
           COORDINATOR_MAC[3], COORDINATOR_MAC[4], COORDINATOR_MAC[5],
           REPORT_CHANNEL);
    printf("[*] Hopping channels 1-13, reporting every %d s (epoch-aligned)\n",
           REPORT_PERIOD_MS / 1000);
    printf("[*] LED: red=no-sync | green-flash=sync-rcvd | blue=synced | white-flash=tx\n");
    printf("[*] Awaiting first sync epoch from coordinator...\n\n");

    xTaskCreate(led_task,              "led",         2048, NULL, 7, NULL);
    xTaskCreate(packet_processor_task, "pkt_proc",    4096, NULL, 5, NULL);
    xTaskCreate(sync_handler_task,     "sync_handler",2048, NULL, 6, NULL);
    xTaskCreate(device_reporter_task,  "reporter",    4096, NULL, 3, NULL);
    xTaskCreate(channel_hop_task,      "ch_hop",      2048, NULL, 4, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
