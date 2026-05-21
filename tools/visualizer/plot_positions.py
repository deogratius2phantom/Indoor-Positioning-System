#!/usr/bin/env python3
"""
Indoor Positioning System — Real-time Serial Visualizer
========================================================
Reads machine-readable lines from the coordinator serial output and plots
device positions on a live floor-plan using matplotlib.

Line formats (emitted by coordinator display_task):
  NODE|<idx>|<MAC>|<x_m>|<y_m>
  POS|<MAC>|<is_random>|<x_m>|<y_m>|<ts_ms>|<ssid>
  ---FRAME END---

Usage:
  python plot_positions.py                        # uses default port
  python plot_positions.py /dev/cu.usbmodem101    # macOS / Linux
  python plot_positions.py COM3                   # Windows
"""

import sys
import time
import threading
import serial
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

# ── Configuration ──────────────────────────────────────────────────────────────
DEFAULT_PORT = "/dev/cu.usbmodem101"
BAUD_RATE    = 115200
STALE_SECS   = 45        # remove a device dot if unseen for this long
UPDATE_MS    = 1000      # plot refresh interval

PORT = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

# ── Shared state (written by reader thread, read by plot thread) ───────────────
_lock   = threading.Lock()
_nodes  = {}    # { "1": {"mac": str, "x": float, "y": float} }
_devices = {}   # { mac_str: {"ssid": str, "x": float, "y": float,
                #              "random": bool, "ts": float} }

# ── Serial reader thread ───────────────────────────────────────────────────────
def _serial_reader():
    while True:
        try:
            with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
                print(f"[visualizer] connected to {PORT} @ {BAUD_RATE}")
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()

                    if line.startswith("NODE|"):
                        # NODE|<idx>|<MAC>|<x>|<y>
                        parts = line.split("|")
                        if len(parts) == 5:
                            try:
                                idx = parts[1]
                                mac = parts[2]
                                x   = float(parts[3])
                                y   = float(parts[4])
                                with _lock:
                                    _nodes[idx] = {"mac": mac, "x": x, "y": y}
                            except ValueError:
                                pass

                    elif line.startswith("POS|"):
                        # POS|<MAC>|<is_random>|<x>|<y>|<ts_ms>|<ssid...>
                        parts = line.split("|", 6)
                        if len(parts) == 7:
                            try:
                                mac     = parts[1]
                                is_rand = int(parts[2]) == 1
                                x       = float(parts[3])
                                y       = float(parts[4])
                                ssid    = parts[6]
                                with _lock:
                                    _devices[mac] = {
                                        "ssid":   ssid,
                                        "x":      x,
                                        "y":      y,
                                        "random": is_rand,
                                        "ts":     time.time(),
                                    }
                            except ValueError:
                                pass

                    elif line == "---FRAME END---":
                        now = time.time()
                        with _lock:
                            stale = [m for m, d in _devices.items()
                                     if now - d["ts"] > STALE_SECS]
                            for m in stale:
                                del _devices[m]

        except serial.SerialException as e:
            print(f"[visualizer] serial error: {e}  — retrying in 3 s")
            time.sleep(3)

# ── Plot setup ─────────────────────────────────────────────────────────────────
_BG_OUTER  = "#1a1a2e"
_BG_INNER  = "#16213e"
_COL_NODE  = "#00d4ff"   # sniffer anchor triangles
_COL_KNOWN = "#ffd93d"   # device with stable MAC
_COL_RAND  = "#ff6b6b"   # device with randomized MAC
_COL_TEXT  = "#e0e0e0"
_COL_GRID  = "#2a2a4a"

fig, ax = plt.subplots(figsize=(9, 8))
fig.patch.set_facecolor(_BG_OUTER)
fig.canvas.manager.set_window_title("IPS Visualizer")

def _update(_frame):
    ax.clear()
    ax.set_facecolor(_BG_INNER)
    ax.set_title("Indoor Positioning System — Live", color=_COL_TEXT,
                 fontsize=13, fontweight="bold", pad=12)
    ax.set_xlabel("X (m)", color=_COL_TEXT)
    ax.set_ylabel("Y (m)", color=_COL_TEXT)
    ax.tick_params(colors=_COL_TEXT, labelsize=9)
    for spine in ax.spines.values():
        spine.set_edgecolor("#444466")
    ax.grid(True, color=_COL_GRID, linestyle="--", linewidth=0.6, zorder=0)

    with _lock:
        nodes_snap   = dict(_nodes)
        devices_snap = dict(_devices)

    # ── Draw sniffer node anchors ──────────────────────────────────────────────
    for idx, node in nodes_snap.items():
        ax.plot(node["x"], node["y"], "^",
                color=_COL_NODE, markersize=16, zorder=5,
                markeredgecolor="#003344", markeredgewidth=1.2)
        ax.annotate(
            f"Node {idx}\n{node['mac'][-8:]}",
            (node["x"], node["y"]),
            textcoords="offset points", xytext=(10, 8),
            color=_COL_NODE, fontsize=8.5, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2", facecolor=_BG_INNER,
                      edgecolor=_COL_NODE, alpha=0.7),
        )

    # ── Draw positioned devices ────────────────────────────────────────────────
    for mac, dev in devices_snap.items():
        age   = time.time() - dev["ts"]
        alpha = max(0.3, 1.0 - age / STALE_SECS)
        color = _COL_RAND if dev["random"] else _COL_KNOWN

        ax.plot(dev["x"], dev["y"], "o",
                color=color, markersize=11, alpha=alpha, zorder=4,
                markeredgecolor=_BG_INNER, markeredgewidth=1.0)

        short_ssid = (dev["ssid"][:14] + "…") if len(dev["ssid"]) > 14 else dev["ssid"]
        label = f"{short_ssid}\n{mac[-8:]}"
        ax.annotate(
            label,
            (dev["x"], dev["y"]),
            textcoords="offset points", xytext=(7, 6),
            color=color, fontsize=7, alpha=alpha,
        )

    # ── Draw distance circles from each node to each device (optional) ────────
    # Uncomment if you want range rings:
    # for mac, dev in devices_snap.items():
    #     for idx, node in nodes_snap.items():
    #         d = ((dev["x"]-node["x"])**2 + (dev["y"]-node["y"])**2)**0.5
    #         circle = plt.Circle((node["x"], node["y"]), d,
    #                             fill=False, color="#333355", linewidth=0.5, zorder=1)
    #         ax.add_patch(circle)

    # ── Legend ─────────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(color=_COL_NODE,  label="Sniffer node anchor"),
        mpatches.Patch(color=_COL_KNOWN, label="Device (stable MAC)"),
        mpatches.Patch(color=_COL_RAND,  label="Device (random MAC)"),
    ]
    leg = ax.legend(handles=legend_handles, loc="upper right",
                    facecolor=_BG_OUTER, edgecolor="#444466",
                    labelcolor=_COL_TEXT, fontsize=9)

    # Status line
    n_dev = len(devices_snap)
    status = (f"{n_dev} device{'s' if n_dev != 1 else ''} positioned  |  "
              f"{len(nodes_snap)} node{'s' if len(nodes_snap) != 1 else ''} configured  |  "
              f"stale timeout {STALE_SECS}s")
    fig.text(0.5, 0.01, status, ha="center", va="bottom",
             color="#666688", fontsize=8)

    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.25)

# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    reader = threading.Thread(target=_serial_reader, daemon=True)
    reader.start()

    ani = FuncAnimation(fig, _update, interval=UPDATE_MS, cache_frame_data=False)
    plt.tight_layout(rect=[0, 0.03, 1, 1])
    plt.show()
