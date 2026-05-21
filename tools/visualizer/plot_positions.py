#!/usr/bin/env python3
"""
Indoor Positioning System — Real-time Serial Visualizer
========================================================
Left panel : live scatter plot of trilaterated device positions.
Right panel: embedded serial terminal showing raw coordinator output.

Line formats parsed from coordinator display_task:
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
from collections import deque
import serial
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

# ── Configuration ──────────────────────────────────────────────────────────────
DEFAULT_PORT   = "/dev/cu.usbmodem101"
BAUD_RATE      = 115200
STALE_SECS     = 45        # remove a device dot if unseen for this long
UPDATE_MS      = 1000      # plot refresh interval
TERMINAL_LINES = 38        # max lines kept in the serial terminal panel
TERMINAL_COLS  = 62        # characters per line before truncation

PORT = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

# ── Shared state (written by reader thread, read by plot thread) ───────────────
_lock         = threading.Lock()
_nodes        = {}                          # { "1": {"mac", "x", "y"} }
_devices      = {}                          # { mac: {"ssid","x","y","random","ts"} }
_serial_lines = deque(maxlen=TERMINAL_LINES)  # raw terminal lines

# ── Serial reader thread ───────────────────────────────────────────────────────
def _serial_reader():
    while True:
        try:
            with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
                print(f"[visualizer] connected to {PORT} @ {BAUD_RATE}")
                with _lock:
                    _serial_lines.append(f"── connected to {PORT} ──")
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()

                    # Always append to terminal buffer
                    with _lock:
                        _serial_lines.append(line)

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
            msg = f"── serial error: {e} — retry in 3s ──"
            print(f"[visualizer] {msg}")
            with _lock:
                _serial_lines.append(msg)
            time.sleep(3)

# ── Plot setup ─────────────────────────────────────────────────────────────────
_BG_OUTER  = "#1a1a2e"
_BG_INNER  = "#16213e"
_BG_TERM   = "#0d0d14"
_COL_NODE  = "#00d4ff"   # sniffer anchor triangles
_COL_KNOWN = "#ffd93d"   # device with stable MAC
_COL_RAND  = "#ff6b6b"   # device with randomized MAC
_COL_TEXT  = "#e0e0e0"
_COL_GRID  = "#2a2a4a"
_COL_TERM  = "#39ff14"   # terminal text (neon green)
_COL_TERM_DIM = "#1a6b08"  # dimmer colour for older lines

fig = plt.figure(figsize=(17, 8))
fig.patch.set_facecolor(_BG_OUTER)
fig.canvas.manager.set_window_title("IPS Visualizer")

gs = fig.add_gridspec(1, 2, width_ratios=[3, 2], wspace=0.04,
                      left=0.05, right=0.97, top=0.93, bottom=0.06)
ax_plot = fig.add_subplot(gs[0])
ax_term = fig.add_subplot(gs[1])

def _update(_frame):
    # ── LEFT: position scatter plot ───────────────────────────────────────────
    ax_plot.clear()
    ax_plot.set_facecolor(_BG_INNER)
    ax_plot.set_title("Indoor Positioning System — Live", color=_COL_TEXT,
                      fontsize=12, fontweight="bold", pad=10)
    ax_plot.set_xlabel("X (m)", color=_COL_TEXT)
    ax_plot.set_ylabel("Y (m)", color=_COL_TEXT)
    ax_plot.tick_params(colors=_COL_TEXT, labelsize=9)
    for spine in ax_plot.spines.values():
        spine.set_edgecolor("#444466")
    ax_plot.grid(True, color=_COL_GRID, linestyle="--", linewidth=0.6, zorder=0)

    with _lock:
        nodes_snap   = dict(_nodes)
        devices_snap = dict(_devices)
        term_snap    = list(_serial_lines)

    # Draw sniffer node anchors
    for idx, node in nodes_snap.items():
        ax_plot.plot(node["x"], node["y"], "^",
                     color=_COL_NODE, markersize=16, zorder=5,
                     markeredgecolor="#003344", markeredgewidth=1.2)
        ax_plot.annotate(
            f"Node {idx}\n{node['mac'][-8:]}",
            (node["x"], node["y"]),
            textcoords="offset points", xytext=(10, 8),
            color=_COL_NODE, fontsize=8.5, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2", facecolor=_BG_INNER,
                      edgecolor=_COL_NODE, alpha=0.7),
        )

    # Draw positioned devices
    for mac, dev in devices_snap.items():
        age   = time.time() - dev["ts"]
        alpha = max(0.3, 1.0 - age / STALE_SECS)
        color = _COL_RAND if dev["random"] else _COL_KNOWN

        ax_plot.plot(dev["x"], dev["y"], "o",
                     color=color, markersize=11, alpha=alpha, zorder=4,
                     markeredgecolor=_BG_INNER, markeredgewidth=1.0)

        short_ssid = (dev["ssid"][:14] + "…") if len(dev["ssid"]) > 14 else dev["ssid"]
        ax_plot.annotate(
            f"{short_ssid}\n{mac[-8:]}",
            (dev["x"], dev["y"]),
            textcoords="offset points", xytext=(7, 6),
            color=color, fontsize=7, alpha=alpha,
        )

    legend_handles = [
        mpatches.Patch(color=_COL_NODE,  label="Sniffer node anchor"),
        mpatches.Patch(color=_COL_KNOWN, label="Device (stable MAC)"),
        mpatches.Patch(color=_COL_RAND,  label="Device (random MAC)"),
    ]
    ax_plot.legend(handles=legend_handles, loc="upper right",
                   facecolor=_BG_OUTER, edgecolor="#444466",
                   labelcolor=_COL_TEXT, fontsize=9)

    n_dev = len(devices_snap)
    fig.texts.clear()
    fig.text(0.5, 0.01,
             (f"{n_dev} device{'s' if n_dev != 1 else ''} positioned  |  "
              f"{len(nodes_snap)} node{'s' if len(nodes_snap) != 1 else ''} configured  |  "
              f"stale timeout {STALE_SECS}s"),
             ha="center", va="bottom", color="#666688", fontsize=8)

    ax_plot.set_aspect("equal", adjustable="datalim")
    ax_plot.margins(0.25)

    # ── RIGHT: serial terminal ────────────────────────────────────────────────
    ax_term.clear()
    ax_term.set_facecolor(_BG_TERM)
    ax_term.set_xticks([])
    ax_term.set_yticks([])
    for spine in ax_term.spines.values():
        spine.set_edgecolor("#00ff4133")
    ax_term.set_title("Serial Monitor", color=_COL_TERM,
                      fontsize=10, fontweight="bold", pad=6)

    # Render lines bottom-to-top so newest is at the bottom
    total = len(term_snap)
    line_height = 1.0 / (TERMINAL_LINES + 1)
    for i, txt in enumerate(term_snap):
        # Truncate and escape any chars that confuse text rendering
        truncated = txt[:TERMINAL_COLS]
        # Fade older lines slightly
        age_frac  = i / max(total - 1, 1)
        color = _COL_TERM if age_frac > 0.6 else _COL_TERM_DIM
        y_pos = (i + 0.5) * line_height
        ax_term.text(
            0.01, y_pos, truncated,
            transform=ax_term.transAxes,
            va="center", ha="left",
            fontfamily="monospace", fontsize=7,
            color=color,
        )


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    reader = threading.Thread(target=_serial_reader, daemon=True)
    reader.start()

    ani = FuncAnimation(fig, _update, interval=UPDATE_MS, cache_frame_data=False)
    plt.show()
