#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>

// Update these values for your local network.
constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
constexpr char NODE_ID[] = "node_1";

IPAddress serverIp(192, 168, 1, 100);
constexpr uint16_t serverPort = 5005;

WiFiUDP udp;

constexpr uint8_t kMinChannel = 1;
constexpr uint8_t kMaxChannel = 13;
constexpr uint32_t kHopIntervalMs = 200;
constexpr size_t kQueueSize = 64;

struct SniffRecord {
  int8_t rssi;
  uint8_t channel;
  uint8_t mac[6];
  uint32_t tsMs;
};

volatile SniffRecord queueBuffer[kQueueSize];
volatile size_t queueHead = 0;
volatile size_t queueTail = 0;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t currentChannel = kMinChannel;
uint32_t lastHop = 0;

bool enqueueRecord(const SniffRecord &record) {
  bool pushed = false;
  portENTER_CRITICAL(&queueMux);
  const size_t nextHead = (queueHead + 1) % kQueueSize;
  if (nextHead != queueTail) {
    queueBuffer[queueHead] = record;
    queueHead = nextHead;
    pushed = true;
  }
  portEXIT_CRITICAL(&queueMux);
  return pushed;
}

bool dequeueRecord(SniffRecord &record) {
  bool popped = false;
  portENTER_CRITICAL(&queueMux);
  if (queueTail != queueHead) {
    record = queueBuffer[queueTail];
    queueTail = (queueTail + 1) % kQueueSize;
    popped = true;
  }
  portEXIT_CRITICAL(&queueMux);
  return popped;
}

void IRAM_ATTR onPacketSniffed(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
    return;
  }

  auto *packet = reinterpret_cast<wifi_promiscuous_pkt_t *>(buf);
  SniffRecord record{};
  record.rssi = packet->rx_ctrl.rssi;
  record.channel = packet->rx_ctrl.channel;
  record.tsMs = millis();

  // Source MAC starts at byte 10 in IEEE 802.11 headers.
  const uint8_t *payload = packet->payload;
  for (size_t i = 0; i < 6; ++i) {
    record.mac[i] = payload[10 + i];
  }

  enqueueRecord(record);
}

String formatMac(const uint8_t mac[6]) {
  char macBuffer[18];
  snprintf(macBuffer,
           sizeof(macBuffer),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],
           mac[1],
           mac[2],
           mac[3],
           mac[4],
           mac[5]);
  return String(macBuffer);
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t startMs = millis();
  constexpr uint32_t kConnectTimeoutMs = 15000;

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(500);
    if (millis() - startMs >= kConnectTimeoutMs) {
      Serial.println("WiFi connection timeout. Restarting...");
      ESP.restart();
    }
  }
  Serial.println("WiFi connected.");
}

void enablePromiscuousMode() {
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(&onPacketSniffed);
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(true);
}

void sendUdpRecord(const SniffRecord &record) {
  String payload = "{";
  payload += "\"node_id\":\"" + String(NODE_ID) + "\",";
  payload += "\"mac\":\"" + formatMac(record.mac) + "\",";
  payload += "\"rssi\":" + String(record.rssi) + ",";
  payload += "\"channel\":" + String(record.channel) + ",";
  payload += "\"ts_ms\":" + String(record.tsMs);
  payload += "}";

  udp.beginPacket(serverIp, serverPort);
  udp.print(payload);
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  connectWifi();
  udp.begin(0);
  enablePromiscuousMode();
  lastHop = millis();
}

void loop() {
  SniffRecord record{};
  while (dequeueRecord(record)) {
    sendUdpRecord(record);
  }

  const uint32_t now = millis();
  if (now - lastHop >= kHopIntervalMs) {
    currentChannel = (currentChannel >= kMaxChannel) ? kMinChannel : (currentChannel + 1);
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    lastHop = now;
  }
}
