#pragma once

#include <Arduino.h>

#ifndef IPS_NODE_ID
#define IPS_NODE_ID "node_1"
#endif

#ifndef IPS_SERIAL_BAUD_RATE
#define IPS_SERIAL_BAUD_RATE 921600
#endif

namespace sniffer_config {
constexpr char kNodeId[] = IPS_NODE_ID;
constexpr uint32_t kSerialBaudRate = IPS_SERIAL_BAUD_RATE;
constexpr uint8_t kMinChannel = 1;
constexpr uint8_t kMaxChannel = 13;
constexpr uint32_t kHopIntervalMs = 200;
constexpr size_t kQueueSize = 128;
constexpr uint32_t kSerialWarmupDelayMs = 1500;
}  // namespace sniffer_config
