# Indoor Positioning System

## File Tree

```text
.
├── docs/
│   └── architecture.md
├── firmware/
│   └── esp32_sniffer/
│       ├── include/
│       │   └── sniffer_config.h
│       ├── src/
│       │   └── main.cpp
│       └── platformio.ini
├── server/
│   ├── config.yaml
│   ├── main.py
│   └── ips_server/
│       ├── __init__.py
│       ├── config.py
│       ├── kalman.py
│       ├── processing.py
│       └── trilateration.py
├── requirements.txt
├── LICENSE
└── README.md
```

## Project Overview

This repository contains a modular Indoor Positioning System (IPS):
- **4 ESP32 sniffer nodes** capture nearby MAC addresses and RSSI in WiFi promiscuous mode.
- Nodes stream newline-delimited JSON to a **Raspberry Pi 4** over dedicated **hardware serial/USB links**.
- The Python server aggregates readings in a **200 ms sliding window**, solves position with **trilateration**, and smooths coordinates using a basic **Kalman filter**.

This transport keeps the ESP32 radio free for channel hopping and avoids the single-radio conflict of trying to sniff and stay associated to WiFi at the same time.

## Raspberry Pi Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python server/main.py
```

Edit the serial device paths, baud rate, processing window, and anchor coordinates in `server/config.yaml` before starting the server.

## ESP32 Firmware Build

The ESP32 firmware is a standalone PlatformIO project in `firmware/esp32_sniffer/`.

```bash
cd firmware/esp32_sniffer
pio run -e esp32-c3
pio run -e esp32-s3
pio run -e esp32-c6
```

Available environments:

- `esp32-c3`
- `esp32-s3`
- `esp32-c6`

To override the node ID at build time, pass a build flag such as:

```bash
pio run -e esp32-s3 --project-option="build_flags=-DARDUINO_USB_CDC_ON_BOOT=1 -DCORE_DEBUG_LEVEL=1 -DIPS_NODE_ID=\\\"node_2\\\""
```

## ESP32 Sniffer Entry Point

Main firmware is located at:

`/firmware/esp32_sniffer/src/main.cpp`

It includes:
- Promiscuous packet callback (MAC + RSSI capture)
- JSON line framing over hardware serial/USB
- Channel hopping on channels 1-13
- Build-time node ID / baud-rate overrides via macros in `include/sniffer_config.h`

## Core Python Trilateration Logic

Core solver is located at:

`/server/ips_server/trilateration.py`

It uses `scipy.optimize.least_squares` and a log-distance RSSI model to estimate `(x, y, z)` from multi-node RSSI windows.

## Configuration

Edit node anchors and serial transport settings in:

`/server/config.yaml`
