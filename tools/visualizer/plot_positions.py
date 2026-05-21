#!/usr/bin/env python3
"""
Indoor Positioning System — Real-time Serial Visualizer
========================================================
Left panel : live scatter plot of trilaterated device positions.
             Node anchors are draggable — drag to set position,
             then click "Send Positions" to push coords to the coordinator.
Right panel: embedded serial terminal showing raw coordinator output.

Serial protocol (to coordinator):
  SET_NODE <1-3> <x_m> <y_m>\\n   →   ACK SET_NODE <idx> <x> <y>

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
from matplotlib.widgets import Button
import matplotlib.patches as mpatches

# ── Configuration ──────────────────────────────────────────────────────────────
DEFAULT_PORT   = "/dev/cu.usbmodem101"
BAUD_RATE      = 115200
STALE_SECS     = 45
UPDATE_MS      = 500          # faster refresh for drag responsiveness
TERMINAL_LINES = 36
TERMINAL_COLS  = 62
DRAG_FRAC      = 0.07         # pick radius as fraction of visible axis range

PORT = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

# ── Thread-safe shared state ───────────────────────────────────────────────────
_lock       = threading.Lock()
_write_lock = threading.Lock()

_nodes   = {}   # {idx_str: {"mac": str, "x": float, "y": float}}   ← firmware positions
_devices = {}   # {mac_str: {"ssid", "x", "y", "random", "ts"}}
_serial_lines = deque(maxlen=TERMINAL_LINES)

# Node override state: {idx_str: {"x": float, "y": float,
#                                  "state": "default"|"pending"|"fixed"}}
# "default"  — using firmware position, nothing dragged yet
# "pending"  — dragged but SET_NODE not yet sent
# "fixed"    — SET_NODE sent AND coordinator ACK received
_node_overrides = {}

_ser_conn = None   # current serial.Serial instance (written by reader thread)

# ── Drag state (main-thread only) ─────────────────────────────────────────────
_drag = {"active": False, "node_idx": None}

# ── Serial helpers ─────────────────────────────────────────────────────────────
def _send_serial(text: str) -> bool:
    with _write_lock:
        if _ser_conn and _ser_conn.is_open:
            try:
                _ser_conn.write(text.encode())
                return True
            except serial.SerialException as e:
                print(f"[visualizer] write error: {e}")
    return False

def _serial_reader():
    global _ser_conn
    while True:
        try:
            ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
            with _write_lock:
                _ser_conn = ser
            with _lock:
                _serial_lines.append(f"── connected to {PORT} ──")
            print(f"[visualizer] connected to {PORT} @ {BAUD_RATE}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                with _lock:
                    _serial_lines.append(line)

                if line.startswith("NODE|"):
                    parts = line.split("|")
                    if len(parts) == 5:
                        try:
                            idx = parts[1]
                            with _lock:
                                _nodes[idx] = {
                                    "mac": parts[2],
                                    "x":   float(parts[3]),
                                    "y":   float(parts[4]),
                                }
                        except ValueError:
                            pass

                elif line.startswith("POS|"):
                    parts = line.split("|", 6)
                    if len(parts) == 7:
                        try:
                            mac = parts[1]
                            with _lock:
                                _devices[mac] = {
                                    "ssid":   parts[6],
                                    "x":      float(parts[3]),
                                    "y":      float(parts[4]),
                                    "random": int(parts[2]) == 1,
                                    "ts":     time.time(),
                                }
                        except ValueError:
                            pass

                elif line == "---FRAME END---":
                    now = time.time()
                    with _lock:
                        for m in [m for m, d in _devices.items()
                                  if now - d["ts"] > STALE_SECS]:
                            del _devices[m]

                elif line.startswith("ACK SET_NODE"):
                    # "ACK SET_NODE <idx> <x> <y>"
                    parts = line.split()
                    if len(parts) == 5:
                        idx = parts[2]
                        with _lock:
                            if idx in _node_overrides:
                                _node_overrides[idx]["state"] = "fixed"

        except serial.SerialException as e:
            with _write_lock:
                _ser_conn = None
            msg = f"── error: {e} — retry in 3 s ──"
            with _lock:
                _serial_lines.append(msg)
            print(f"[visualizer] {msg}")
            time.sleep(3)

# ── Colours ────────────────────────────────────────────────────────────────────
_BG_OUTER         = "#1a1a2e"
_BG_INNER         = "#16213e"
_BG_TERM          = "#0d0d14"
_COL_NODE_DEFAULT = "#00d4ff"   # cyan    — firmware position
_COL_NODE_PENDING = "#ff9500"   # orange  — dragged, not yet sent
_COL_NODE_FIXED   = "#39ff14"   # green   — coordinator confirmed
_COL_KNOWN        = "#ffd93d"   # yellow  — stable MAC device
_COL_RAND         = "#ff6b6b"   # red     — random MAC device
_COL_TEXT         = "#e0e0e0"
_COL_GRID         = "#2a2a4a"
_COL_TERM_HI      = "#39ff14"   # bright green
_COL_TERM_LO      = "#1a6b08"   # dim green

# ── Figure layout ──────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(17, 9))
fig.patch.set_facecolor(_BG_OUTER)
try:
    fig.canvas.manager.set_window_title("IPS Visualizer")
except Exception:
    pass

gs = fig.add_gridspec(
    1, 2, width_ratios=[3, 2],
    left=0.05, right=0.97, top=0.93, bottom=0.14, wspace=0.04
)
ax_plot = fig.add_subplot(gs[0])
ax_term = fig.add_subplot(gs[1])

# Buttons — placed below the left panel
_BTN_Y   = 0.03
_BTN_H   = 0.07
ax_b_send  = fig.add_axes([0.05, _BTN_Y, 0.24, _BTN_H])
ax_b_reset = fig.add_axes([0.31, _BTN_Y, 0.20, _BTN_H])

btn_send  = Button(ax_b_send,  "📡  Send Positions to Coordinator",
                   color="#0a2a0a", hovercolor="#1a5a1a")
btn_reset = Button(ax_b_reset, "↺  Reset to Firmware Positions",
                   color="#2a0a0a", hovercolor="#5a1a1a")

for btn, col in ((btn_send, _COL_NODE_FIXED), (btn_reset, _COL_RAND)):
    btn.label.set_color(col)
    btn.label.set_fontsize(9)
    btn.label.set_fontweight("bold")

def _on_send(_event):
    with _lock:
        pending = {k: dict(v) for k, v in _node_overrides.items()
                   if v["state"] == "pending"}
    if not pending:
        with _lock:
            _serial_lines.append("── no pending positions to send ──")
        return
    for idx in sorted(pending, key=int):
        ov  = pending[idx]
        cmd = f"SET_NODE {idx} {ov['x']:.3f} {ov['y']:.3f}\n"
        if _send_serial(cmd):
            with _lock:
                _serial_lines.append(f"→ {cmd.strip()}")
        else:
            with _lock:
                _serial_lines.append(f"✗ send failed (no connection)")

def _on_reset(_event):
    with _lock:
        _node_overrides.clear()
    with _lock:
        _serial_lines.append("── node positions reset to firmware values ──")

btn_send.on_clicked(_on_send)
btn_reset.on_clicked(_on_reset)

# ── Drag helpers ───────────────────────────────────────────────────────────────
def _display_pos(idx):
    """Return (x, y, state) for a node — overrides take priority."""
    with _lock:
        if idx in _node_overrides:
            ov = _node_overrides[idx]
            return ov["x"], ov["y"], ov["state"]
        if idx in _nodes:
            n = _nodes[idx]
            return n["x"], n["y"], "default"
    return None, None, "default"

def _pick_radius():
    try:
        xl, xr = ax_plot.get_xlim()
        yb, yt = ax_plot.get_ylim()
        return max(xr - xl, yt - yb) * DRAG_FRAC
    except Exception:
        return 0.5

def _on_press(event):
    if event.inaxes is not ax_plot or event.xdata is None:
        return
    with _lock:
        node_idxs = list(_nodes.keys())
    radius = _pick_radius()
    best_idx, best_d = None, radius
    for idx in node_idxs:
        x, y, _ = _display_pos(idx)
        if x is None:
            continue
        d = ((event.xdata - x) ** 2 + (event.ydata - y) ** 2) ** 0.5
        if d < best_d:
            best_d, best_idx = d, idx
    if best_idx is not None:
        _drag["active"]   = True
        _drag["node_idx"] = best_idx
        x, y, _ = _display_pos(best_idx)
        with _lock:
            _node_overrides[best_idx] = {
                "x": x, "y": y, "state": "pending"
            }

def _on_motion(event):
    if not _drag["active"] or event.inaxes is not ax_plot or event.xdata is None:
        return
    idx = _drag["node_idx"]
    if idx is not None:
        with _lock:
            if idx in _node_overrides:
                _node_overrides[idx]["x"] = event.xdata
                _node_overrides[idx]["y"] = event.ydata

def _on_release(event):
    _drag["active"]   = False
    _drag["node_idx"] = None

fig.canvas.mpl_connect("button_press_event",   _on_press)
fig.canvas.mpl_connect("motion_notify_event",  _on_motion)
fig.canvas.mpl_connect("button_release_event", _on_release)

# ── Animation update ───────────────────────────────────────────────────────────
def _update(_frame):
    with _lock:
        nodes_snap    = dict(_nodes)
        devices_snap  = dict(_devices)
        term_snap     = list(_serial_lines)
        overrides_snap = dict(_node_overrides)

    # ── LEFT: position plot ───────────────────────────────────────────────────
    ax_plot.clear()
    ax_plot.set_facecolor(_BG_INNER)
    ax_plot.set_title(
        "Indoor Positioning System — Live\n"
        "drag ▲ anchors to set node positions, then click Send",
        color=_COL_TEXT, fontsize=11, fontweight="bold", pad=8
    )
    ax_plot.set_xlabel("X (m)", color=_COL_TEXT)
    ax_plot.set_ylabel("Y (m)", color=_COL_TEXT)
    ax_plot.tick_params(colors=_COL_TEXT, labelsize=9)
    for spine in ax_plot.spines.values():
        spine.set_edgecolor("#444466")
    ax_plot.grid(True, color=_COL_GRID, linestyle="--", linewidth=0.6, zorder=0)

    # Node anchors
    for idx, node in nodes_snap.items():
        ov    = overrides_snap.get(idx)
        state = ov["state"] if ov else "default"
        x     = ov["x"]     if ov else node["x"]
        y     = ov["y"]     if ov else node["y"]

        if state == "fixed":
            color, suffix = _COL_NODE_FIXED,   " ✓"
        elif state == "pending":
            color, suffix = _COL_NODE_PENDING, " ●"
        else:
            color, suffix = _COL_NODE_DEFAULT, ""

        ax_plot.plot(x, y, "^", color=color, markersize=17, zorder=5,
                     markeredgecolor="#001020", markeredgewidth=1.2)
        ax_plot.annotate(
            f"Node {idx}{suffix}\n({x:.2f}, {y:.2f})",
            (x, y), textcoords="offset points", xytext=(11, 8),
            color=color, fontsize=8.5, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", facecolor=_BG_INNER,
                      edgecolor=color, alpha=0.85),
        )

    # Positioned devices
    for mac, dev in devices_snap.items():
        age   = time.time() - dev["ts"]
        alpha = max(0.3, 1.0 - age / STALE_SECS)
        color = _COL_RAND if dev["random"] else _COL_KNOWN
        ax_plot.plot(dev["x"], dev["y"], "o", color=color,
                     markersize=10, alpha=alpha, zorder=4,
                     markeredgecolor=_BG_INNER, markeredgewidth=0.8)
        short = (dev["ssid"][:13] + "…") if len(dev["ssid"]) > 13 else dev["ssid"]
        ax_plot.annotate(
            f"{short}\n{mac[-8:]}",
            (dev["x"], dev["y"]), textcoords="offset points", xytext=(6, 5),
            color=color, fontsize=7, alpha=alpha,
        )

    # Legend
    handles = [
        mpatches.Patch(color=_COL_NODE_DEFAULT, label="Node anchor (firmware pos)"),
        mpatches.Patch(color=_COL_NODE_PENDING, label="Node anchor (moved, unsent ●)"),
        mpatches.Patch(color=_COL_NODE_FIXED,   label="Node anchor (confirmed ✓)"),
        mpatches.Patch(color=_COL_KNOWN,        label="Device — stable MAC"),
        mpatches.Patch(color=_COL_RAND,         label="Device — random MAC"),
    ]
    ax_plot.legend(handles=handles, loc="upper right",
                   facecolor=_BG_OUTER, edgecolor="#444466",
                   labelcolor=_COL_TEXT, fontsize=8)
    ax_plot.set_aspect("equal", adjustable="datalim")
    ax_plot.margins(0.25)

    # Status bar
    n_pend = sum(1 for v in overrides_snap.values() if v["state"] == "pending")
    n_fix  = sum(1 for v in overrides_snap.values() if v["state"] == "fixed")
    fig.texts.clear()
    fig.text(
        0.5, 0.002,
        f"{len(devices_snap)} device(s) tracked  |  "
        f"{n_pend} position(s) pending  |  {n_fix} confirmed",
        ha="center", va="bottom", color="#666688", fontsize=8,
    )

    # ── RIGHT: serial terminal ────────────────────────────────────────────────
    ax_term.clear()
    ax_term.set_facecolor(_BG_TERM)
    ax_term.set_xticks([])
    ax_term.set_yticks([])
    for spine in ax_term.spines.values():
        spine.set_edgecolor("#00ff4122")
    ax_term.set_title("Serial Monitor", color=_COL_TERM_HI,
                       fontsize=10, fontweight="bold", pad=6)

    total = len(term_snap)
    lh    = 1.0 / (TERMINAL_LINES + 1)
    for i, txt in enumerate(term_snap):
        age_frac = i / max(total - 1, 1)
        col = _COL_TERM_HI if age_frac > 0.55 else _COL_TERM_LO
        ax_term.text(
            0.01, (i + 0.5) * lh, txt[:TERMINAL_COLS],
            transform=ax_term.transAxes,
            va="center", ha="left",
            fontfamily="monospace", fontsize=7,
            color=col,
        )

# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=_serial_reader, daemon=True).start()
    ani = FuncAnimation(fig, _update, interval=UPDATE_MS, cache_frame_data=False)
    plt.show()

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
