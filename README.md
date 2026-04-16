# Indoor Positioning System

## File Tree

```text
.
├── docs/
│   └── architecture.md
├── firmware/
│   └── esp32_sniffer/
│       └── esp32_sniffer.ino
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
- Nodes send readings to a **Raspberry Pi 4** over UDP.
- The Python server aggregates readings in a **200 ms sliding window**, solves position with **trilateration**, and smooths coordinates using a basic **Kalman filter**.

## Installation (Raspberry Pi)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python server/main.py
```

## ESP32 Sniffer Entry Point

Main firmware is located at:

`/firmware/esp32_sniffer/esp32_sniffer.ino`

It includes:
- WiFi station connection
- Promiscuous packet callback (MAC + RSSI capture)
- UDP forwarding to Raspberry Pi
- Channel hopping on channels 1-13

## Core Python Trilateration Logic

Core solver is located at:

`/server/ips_server/trilateration.py`

It uses `scipy.optimize.least_squares` and a log-distance RSSI model to estimate `(x, y, z)` from multi-node RSSI windows.

## Configuration

Edit node anchors and UDP bind settings in:

`/server/config.yaml`
