#!/usr/bin/env python3
"""
Indoor Positioning System — Real-time Serial Visualizer
========================================================
Left panel : live scatter plot of trilaterated device positions.
             Node anchors are draggable — drag to set position,
             then click "Send Positions" to push coords to the coordinator.
Right panel: embedded serial terminal showing raw coordinator output.

Serial protocol (to coordinator):
  SET_NODE <1-3> [<MAC>] <x_m> <y_m>\\n   ->   ACK SET_NODE <idx> [<MAC>] <x> <y>

Usage:
  python plot_positions.py                        # uses default port
  python plot_positions.py /dev/cu.usbmodem101    # macOS / Linux
  python plot_positions.py COM3                   # Windows
"""

import sys
import time
import re
import threading
from collections import deque

import serial
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox
import matplotlib.patches as mpatches

# ── Configuration ──────────────────────────────────────────────────────────────
DEFAULT_PORT   = "/dev/cu.usbmodem101"
BAUD_RATE      = 115200
STALE_SECS     = 45
UPDATE_MS      = 500          # faster refresh for drag responsiveness
TERMINAL_LINES = 36
TERMINAL_COLS  = 62
DRAG_FRAC      = 0.07         # pick radius as fraction of visible axis range
MAC_RE         = re.compile(r"^[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}$")

PORT = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

# ── Thread-safe shared state ───────────────────────────────────────────────────
_lock       = threading.Lock()
_write_lock = threading.Lock()

_nodes   = {}   # {idx_str: {"mac": str, "x": float, "y": float}}   <- firmware positions
_devices = {}   # {mac_str: {"ssid", "x", "y", "random", "ts"}}
_serial_lines = deque(maxlen=TERMINAL_LINES)

# Node override state: {idx_str: {"x": float, "y": float,
#                                  "state": "default"|"pending"|"fixed"}}
_node_overrides = {}

# MAC entries typed by the user in the TextBox widgets {idx_str: mac_str}
_mac_entries = {}

# MACs of sniffer nodes auto-discovered via DISC| serial lines (ordered)
_discovered_macs = []
_cycle_idx = {"1": 0, "2": 0, "3": 0}   # per-node cycle pointer

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

def _clean(s: str) -> str:
    """Strip ASCII control characters that confuse matplotlib's font renderer."""
    return re.sub(r"[\x00-\x08\x0b-\x1f\x7f]", "", s)

def _serial_reader():
    global _ser_conn
    while True:
        try:
            ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
            with _write_lock:
                _ser_conn = ser
            with _lock:
                _serial_lines.append(f"-- connected to {PORT} --")
            print(f"[visualizer] connected to {PORT} @ {BAUD_RATE}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                line = _clean(line)
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
                    # "ACK SET_NODE <idx> <MAC> <x> <y>"  (new)
                    # "ACK SET_NODE <idx> <x> <y>"         (old/compat)
                    parts = line.split()
                    if len(parts) >= 4:
                        idx = parts[2]
                        # 6-part = new format with MAC
                        if len(parts) == 6 and ":" in parts[3]:
                            with _lock:
                                if idx in _node_overrides:
                                    _node_overrides[idx]["state"] = "fixed"
                        elif len(parts) == 5:
                            with _lock:
                                if idx in _node_overrides:
                                    _node_overrides[idx]["state"] = "fixed"

                elif line.startswith("DISC|"):
                    # "DISC|AA:BB:CC:DD:EE:FF" — coordinator discovered a sniffer node
                    parts = line.split("|")
                    if len(parts) == 2 and MAC_RE.match(parts[1]):
                        mac = parts[1].upper()
                        if mac not in _discovered_macs:
                            _discovered_macs.append(mac)
                            slot = len(_discovered_macs)   # 1, 2, or 3
                            # auto-fill the corresponding TextBox if user hasn't typed
                            if slot <= 3 and _mac_boxes:
                                tb = _mac_boxes[slot - 1]
                                idx_str = str(slot)
                                # only auto-fill if TextBox still shows placeholder
                                cur = tb.text if hasattr(tb, "text") else ""
                                if cur in ("", "XX:XX:XX:XX:XX:XX"):
                                    tb.set_val(mac)
                                    _mac_entries[idx_str] = mac
                                    with _lock:
                                        _serial_lines.append(
                                            f"Auto-filled Node {slot} MAC: {mac}"
                                        )

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
fig = plt.figure(figsize=(17, 10))
fig.patch.set_facecolor(_BG_OUTER)
try:
    fig.canvas.manager.set_window_title("IPS Visualizer")
except Exception:
    pass

gs = fig.add_gridspec(
    1, 2, width_ratios=[3, 2],
    left=0.05, right=0.97, top=0.93, bottom=0.25, wspace=0.04
)
ax_plot = fig.add_subplot(gs[0])
ax_term = fig.add_subplot(gs[1])

# MAC address entry row (above buttons)
# Layout: [TextBox  ⟳] [TextBox  ⟳] [TextBox  ⟳]
_MAC_Y, _MAC_H = 0.14, 0.07
_TB_W, _CYC_W, _GAP = 0.13, 0.034, 0.002
_mac_axes = [
    fig.add_axes([0.05,                              _MAC_Y, _TB_W,  _MAC_H]),
    fig.add_axes([0.05 + _TB_W + _CYC_W + 0.015,    _MAC_Y, _TB_W,  _MAC_H]),
    fig.add_axes([0.05 + 2*(_TB_W + _CYC_W + 0.015), _MAC_Y, _TB_W, _MAC_H]),
]
_cyc_axes = [
    fig.add_axes([0.05 + _TB_W + _GAP,               _MAC_Y, _CYC_W, _MAC_H]),
    fig.add_axes([0.05 + _TB_W + _CYC_W + 0.015 + _TB_W + _GAP, _MAC_Y, _CYC_W, _MAC_H]),
    fig.add_axes([0.05 + 2*(_TB_W + _CYC_W + 0.015) + _TB_W + _GAP, _MAC_Y, _CYC_W, _MAC_H]),
]
_mac_boxes = []
for _i, _ax in enumerate(_mac_axes):
    _tb = TextBox(_ax, f" N{_i+1} MAC: ",
                  initial="XX:XX:XX:XX:XX:XX",
                  color="#060616", hovercolor="#0d0d2a")
    _tb.label.set_color(_COL_TEXT)
    _tb.label.set_fontsize(8)
    try:
        _tb.text_disp.set_color(_COL_NODE_DEFAULT)
        _tb.text_disp.set_fontfamily("monospace")
    except Exception:
        pass
    _mac_boxes.append(_tb)

def _make_mac_cb(idx_str, tb):
    def _cb(text):
        text = text.strip().upper()
        if MAC_RE.match(text):
            _mac_entries[idx_str] = text
            with _lock:
                _serial_lines.append(f"Node {idx_str} MAC stored: {text}")
        else:
            with _lock:
                _serial_lines.append(f"Invalid MAC '{text}' (need AA:BB:CC:DD:EE:FF)")
            tb.set_val("XX:XX:XX:XX:XX:XX")
    return _cb

for _i, _tb in enumerate(_mac_boxes):
    _tb.on_submit(_make_mac_cb(str(_i + 1), _tb))

def _make_cycle_cb(idx_str, tb):
    """Cycle through _discovered_macs for the given node slot."""
    def _cb(_event):
        if not _discovered_macs:
            with _lock:
                _serial_lines.append("No nodes discovered yet — power on sniffer nodes")
            return
        _cycle_idx[idx_str] = (_cycle_idx[idx_str] + 1) % len(_discovered_macs)
        mac = _discovered_macs[_cycle_idx[idx_str]]
        tb.set_val(mac)
        _mac_entries[idx_str] = mac
        with _lock:
            _serial_lines.append(f"Node {idx_str} -> {mac}")
    return _cb

_mac_cycle_btns = []
for _i, _cax in enumerate(_cyc_axes):
    _cb_btn = Button(_cax, "\u27f3", color="#0a0a2a", hovercolor="#1a1a4a")
    _cb_btn.label.set_color(_COL_NODE_DEFAULT)
    _cb_btn.label.set_fontsize(11)
    _cb_btn.on_clicked(_make_cycle_cb(str(_i + 1), _mac_boxes[_i]))
    _mac_cycle_btns.append(_cb_btn)

# Buttons row (below MAC row)
_BTN_Y, _BTN_H = 0.04, 0.07
ax_b_send  = fig.add_axes([0.05, _BTN_Y, 0.24, _BTN_H])
ax_b_reset = fig.add_axes([0.31, _BTN_Y, 0.20, _BTN_H])

btn_send  = Button(ax_b_send,  "Send Positions to Coordinator",
                   color="#0a2a0a", hovercolor="#1a5a1a")
btn_reset = Button(ax_b_reset, "Reset to Firmware Positions",
                   color="#2a0a0a", hovercolor="#5a1a1a")

for btn, col in ((btn_send, _COL_NODE_FIXED), (btn_reset, _COL_RAND)):
    btn.label.set_color(col)
    btn.label.set_fontsize(9)
    btn.label.set_fontweight("bold")

def _on_send(_event):
    with _lock:
        overrides = dict(_node_overrides)
        nodes     = dict(_nodes)

    # Send any node that has a pending position OR a typed MAC
    candidates = (
        {idx for idx, ov in overrides.items() if ov.get("state") == "pending"}
        | {idx for idx, mac in _mac_entries.items() if mac}
    )
    if not candidates:
        with _lock:
            _serial_lines.append("-- no pending positions or MACs to send --")
        return

    for idx in sorted(candidates, key=int):
        ov  = overrides.get(idx, {})
        x   = ov.get("x")
        y   = ov.get("y")
        if x is None:                        # no drag override — use firmware pos
            n = nodes.get(idx, {})
            x, y = n.get("x"), n.get("y")
        if x is None:
            with _lock:
                _serial_lines.append(f"Node {idx}: position unknown, skip")
            continue

        mac = _mac_entries.get(idx, "")
        if mac:
            cmd = f"SET_NODE {idx} {mac} {x:.3f} {y:.3f}\n"
        else:
            cmd = f"SET_NODE {idx} {x:.3f} {y:.3f}\n"

        if _send_serial(cmd):
            with _lock:
                _serial_lines.append(f"-> {cmd.strip()}")
        else:
            with _lock:
                _serial_lines.append("send failed (no connection)")

def _on_reset(_event):
    with _lock:
        _node_overrides.clear()
    _mac_entries.clear()
    _cycle_idx.update({"1": 0, "2": 0, "3": 0})
    for tb in _mac_boxes:
        tb.set_val("XX:XX:XX:XX:XX:XX")
    with _lock:
        _serial_lines.append("-- node positions and MACs reset --")

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
