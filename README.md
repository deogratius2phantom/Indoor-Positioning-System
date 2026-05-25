# Indoor Positioning System

A WiFi-based indoor positioning system using ESP32 microcontrollers and trilateration. Three sniffer nodes sniff WiFi probe-request frames in promiscuous mode, report RSSI data to a coordinator node over ESP-NOW, and a desktop visualizer performs trilateration and plots device positions in real time.

---

## System Architecture

![System Architecture](docs/IPS%20block%20diagram.png)

---

## Repository Structure

```text
.
├── firmware/
│   ├── espnow_coordinator/       # Coordinator node firmware (ESP-IDF)
│   │   ├── src/main.c
│   │   └── platformio.ini        # Targets: ESP32-C3, ESP32, ESP32-S3
│   └── espnow_sniffer_node/      # Sniffer node firmware (ESP-IDF)
│       ├── src/main.c
│       ├── src/led_strip_encoder.c
│       └── platformio.ini        # Target: ESP32-C3
├── tools/
│   └── visualizer/
│       ├── plot_positions.py     # Tkinter visualizer (main app)
│       ├── ips_visualizer.spec   # PyInstaller build spec
│       └── requirements.txt
├── .github/
│   └── workflows/
│       └── build_visualizer.yml  # CI: builds macOS .app + Windows .exe
├── docs/
│   └── architecture.md
└── README.md
```

---

## Hardware

| Role | Board | Count |
|------|-------|-------|
| Sniffer node | ESP32-C3-DevKitM-1 | 3 |
| Coordinator | ESP32-C3-DevKitM-1 / DOIT ESP32 DevKit V1 / ESP32-S3-DevKitC-1 | 1 |
| Host computer | Any Mac / Windows / Linux PC | 1 |

---

## How It Works

### 1. Sniffer Nodes
Each sniffer node operates in **WiFi promiscuous mode**, cycling through channels 1–13 to capture probe-request frames from nearby WiFi devices. For each detected device it tracks:
- Device MAC address
- Per-channel RSSI (strongest channel wins)
- A 5-second **window ID** derived from the coordinator's clock

Every 5 seconds, each node sends an `MSG_RSSI_REPORT` packet via **ESP-NOW** to the coordinator containing all observed devices and their RSSI values.

### 2. Clock Synchronisation (NTP-style)
The coordinator periodically broadcasts a `MSG_SYNC_EPOCH` containing a timestamp `T1`. Each sniffer records `T2` (receive time) and replies with a `MSG_SYNC_ACK` containing `T2` and `T3` (captured before the channel-switch overhead). The coordinator computes:

```
rtt    = (T4 - T1) - (T3 - T2)
offset = (T2 - T1) - rtt / 2
```

Sniffers apply `offset` so their `window_id = (local_ms - offset) / 5000` aligns with coordinator time. This ensures RSSI reports from all nodes for the same physical moment land in the same window, maximising trilateration accuracy.

### 3. Coordinator
The coordinator:
- Receives and aggregates RSSI reports from all sniffer nodes
- Manages the clock-sync broadcast (rapid 3 s cadence for 60 s on boot, then 30 s steady-state)
- Emits a machine-readable serial protocol over USB at 115200 baud
- Accepts `CMD:SYNC\n` over serial to trigger an immediate sync broadcast

### 4. Serial Protocol

```
DISC|<node_mac>
RSSI|<device_mac>|<is_random>|<ssid>|<node_mac>|<avg_rssi>|<window_id>
SYNC|<node_mac>|<offset_ms>|<rtt_ms>
---FRAME END---
CMD_ACK:SYNC
```

### 5. Visualizer
A Tkinter desktop app reads the serial stream, performs **trilateration** (Nelder-Mead via scipy) per device, and renders a live 2-D floor-plan canvas with:
- Draggable, lockable anchor nodes (positioned in metres)
- Live device positions updated every 400 ms
- Sync status badges per node (green ⟳ / red ⚠)
- Scrollable serial terminal showing human-readable coordinator output
- ⚡ **Sync clocks** button (sends `CMD:SYNC` to coordinator)
- ↺ **Refresh plot** button

---

## Getting Started

### Flash the Coordinator

```bash
cd firmware/espnow_coordinator

# ESP32-C3
~/.platformio/penv/bin/pio run -e esp32-c3-devkitm-1 --target upload

# DOIT ESP32 DevKit V1
~/.platformio/penv/bin/pio run -e esp32doit-devkit-v1 --target upload

# ESP32-S3
~/.platformio/penv/bin/pio run -e esp32-s3-devkitc-1 --target upload
```

Note the MAC address printed on the serial monitor, then set `COORDINATOR_MAC` in the sniffer firmware.

### Flash the Sniffer Nodes

```bash
# Edit COORDINATOR_MAC in firmware/espnow_sniffer_node/src/main.c first
cd firmware/espnow_sniffer_node
~/.platformio/penv/bin/pio run -e esp32-c3-devkitm-1 --target upload
```

Flash all three sniffer nodes the same way (port changes automatically with each board).

### Run the Visualizer (from source)

```bash
cd tools/visualizer
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python plot_positions.py --port /dev/cu.usbmodemXXXX  # omit --port for GUI picker
```

---

## Desktop App (No Python Required)

Pre-built binaries are available on the [Releases page](../../releases):

| Platform | Download |
|----------|----------|
| 🍎 macOS | `IPS-Visualizer-macOS.zip` — unzip and double-click `IPS Visualizer.app` |
| 🪟 Windows 10/11 | `IPS-Visualizer-Windows.exe` — double-click to run |

On first launch a **port-picker dialog** appears — select your coordinator's serial port and click **Connect**.

> **macOS note:** If Gatekeeper blocks the app, right-click → Open → Open.

---

## Building the Desktop App Locally

```bash
cd tools/visualizer
pip install pyinstaller
pyinstaller ips_visualizer.spec --distpath dist --workpath build --noconfirm
# macOS: dist/IPS Visualizer.app
# Windows: dist/IPS Visualizer.exe
```

The GitHub Actions workflow (`.github/workflows/build_visualizer.yml`) builds both platforms automatically on every `v*` tag push.

---

## Trilateration Model

The visualizer uses a **log-distance path-loss model**:

```
distance = 10 ^ ((RSSI_ref - RSSI) / (10 × n))
```

Default parameters (adjustable via sliders in the UI):
- `RSSI_ref = -40 dBm` (reference RSSI at 1 m)
- `n = 2.5` (path-loss exponent — increase for walls/obstacles)

Position is estimated by minimising the sum of squared distance residuals using **scipy's Nelder-Mead** solver.

---

## License

See [LICENSE](LICENSE).

