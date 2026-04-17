#include <WiFi.h>
#include "esp_wifi.h"

// Callback function that processes every captured packet
void sniffer_callback(void* buf, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    wifi_pkt_rx_ctrl_t ctrl = (wifi_pkt_rx_ctrl_t)pkt->rx_ctrl;

    // We only care about Management frames (like Probe Requests) or Data frames
    // This filters out noise like ACK or Beacon frames from routers
    uint8_t *payload = pkt->payload;
    
    // Extract MAC Address (Source Address is typically at offset 10 for most frames)
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             payload[10], payload[11], payload[12], payload[13], payload[14], payload[15]);

    // Print to Serial for testing: MAC | RSSI | Channel
    Serial.printf("%s,%d,%d\n", macStr, ctrl.rssi, ctrl.channel);
}

void setup() {
    Serial.begin(115200);

    // 1. Initialize WiFi in "Null" mode (not connected to an AP)
    WiFi.mode(WIFI_MODE_NULL);
    esp_wifi_start();

    // 2. Configure Promiscuous Mode
    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&sniffer_callback);
    esp_wifi_set_promiscuous(true);

    Serial.println("Sniffer Started. Output format: MAC,RSSI,CHANNEL");
}

void loop() {
    // 3. Channel Hopping
    // WiFi devices spread across channels 1-13. We must cycle through them.
    for (int ch = 1; ch <= 13; ch++) {
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        delay(100); // Spend 100ms on each channel
    }
}