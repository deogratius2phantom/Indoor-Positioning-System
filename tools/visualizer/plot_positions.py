#!/usr/bin/env python3
"""
Indoor Positioning System — Tkinter Visualizer
===============================================
Reads RSSI data from the coordinator node via serial, performs trilateration
with scipy, and shows anchor nodes + tracked devices on a live 2-D canvas.

Serial protocol (from coordinator):
  DISC|<node_mac>
  RSSI|<device_mac>|<is_random>|<ssid>|<node_mac>|<avg_rssi>|<window_id>
  ---FRAME END---

Run:
  python plot_positions.py [--port /dev/cu.usbmodemXXXX] [--baud 115200]
"""

import argparse
import re
import sys
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import ttk

try:
    from serial.tools import list_ports as _list_ports
    _HAS_LIST_PORTS = True
except ImportError:
    _HAS_LIST_PORTS = False

import numpy as np
import serial
from scipy.optimize import minimize

# ── Configuration ──────────────────────────────────────────────────────────────
DEFAULT_PORT      = "/dev/cu.usbmodem1101"
DEFAULT_BAUD      = 115200
STALE_SECS        = 45
UPDATE_MS         = 400       # canvas refresh rate
WINDOW_TOLERANCE  = 5         # max window_id spread across nodes for trilat
MAC_RE            = re.compile(r"^[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}$")

# Initial canvas size — grows when the window is resized
CANVAS_W, CANVAS_H = 620, 520
WORLD_X = (-1.0, 13.0)       # metres shown on x-axis
WORLD_Y = (-1.0,  9.0)       # metres shown on y-axis

# ── Colours ────────────────────────────────────────────────────────────────────
COL_BG            = "#1a1a2e"
COL_GRID          = "#2a2a4a"
COL_AXIS_LABEL    = "#555577"
COL_NODE_DEFAULT  = "#00d4ff"   # cyan   — default position
COL_NODE_DRAGGING = "#ff9500"   # amber  — being dragged
COL_NODE_FIXED    = "#39ff14"   # green  — position locked
COL_KNOWN         = "#ffd93d"   # yellow — stable MAC device
COL_RAND          = "#ff6b6b"   # red    — random MAC device
COL_TEXT          = "#e0e0e0"
COL_TERM_BG       = "#0d0d14"
COL_TERM_FG       = "#39ff14"

# ── Thread-safe shared state ───────────────────────────────────────────────────
_lock = threading.Lock()

# _nodes[idx_str] = {"mac": str, "x": float, "y": float, "state": str}
# state: "default" | "dragging" | "fixed"
_nodes = {
    "1": {"mac": "", "x": 0.0, "y": 0.0, "state": "default"},
    "2": {"mac": "", "x": 5.0, "y": 0.0, "state": "default"},
    "3": {"mac": "", "x": 2.5, "y": 4.0, "state": "default"},
}

# RSSI buffer: {device_mac: {node_mac: {"rssi", "window_id", "is_random", "ssid", "ts"}}}
_rssi_buf  = {}

# Trilaterated device positions: {device_mac: {"x", "y", "random", "ssid", "ts"}}
_devices   = {}

# Sniffer MACs auto-discovered via DISC| lines (in arrival order)
_discovered_macs = []

# Sync status per node MAC: {node_mac: {"offset_ms": int, "rtt_ms": int, "ts": float}}
_sync_status  = {}
# Timestamp of last diagnostic message — throttles warning spam
_last_diag_ts = 0.0

# Serial log — bounded deque; old entries auto-drop when full
_serial_lines = deque(maxlen=150)

# Per-frame RSSI report counter (reset each FRAME END, shown in frame summary)
_frame_rssi_count = 0

# Tunable parameters — read from sliders on every trilateration call
_rssi_ref    = -50.0
_path_loss_n =   3.0

# GUI root (set in _build_gui)
_root = None

# Active serial port object — shared so the GUI "Sync" button can write to it
_ser  = None

# ── Canvas / log widget handles (set in _build_gui) ───────────────────────────
_canvas   = None
_log_text = None

# ── Coordinate helpers ─────────────────────────────────────────────────────────
def _cw() -> int:
    return _canvas.winfo_width()  if (_canvas and _canvas.winfo_width()  > 1) else CANVAS_W

def _ch() -> int:
    return _canvas.winfo_height() if (_canvas and _canvas.winfo_height() > 1) else CANVAS_H

def _w2c(wx: float, wy: float):
    """World metres → canvas pixel (x, y)."""
    cw, ch = _cw(), _ch()
    cx = (wx - WORLD_X[0]) / (WORLD_X[1] - WORLD_X[0]) * cw
    cy = ch - (wy - WORLD_Y[0]) / (WORLD_Y[1] - WORLD_Y[0]) * ch
    return cx, cy

def _c2w(cx: float, cy: float):
    """Canvas pixel → world metres."""
    cw, ch = _cw(), _ch()
    wx = cx / cw * (WORLD_X[1] - WORLD_X[0]) + WORLD_X[0]
    wy = (ch - cy) / ch * (WORLD_Y[1] - WORLD_Y[0]) + WORLD_Y[0]
    return wx, wy

# ── Trilateration ──────────────────────────────────────────────────────────────
def _rssi_to_distance(rssi: float) -> float:
    exp = (_rssi_ref - rssi) / (10.0 * _path_loss_n)
    return max(0.1, min(50.0, 10.0 ** exp))

def _scipy_trilaterate(positions: np.ndarray, distances: np.ndarray):
    """Weighted Nelder-Mead least-squares trilateration. Returns (x, y) or None."""
    weights = 1.0 / np.maximum(distances, 0.1) ** 2
    x0 = float(np.average(positions[:, 0], weights=weights))
    y0 = float(np.average(positions[:, 1], weights=weights))

    def cost(p):
        dx = positions[:, 0] - p[0]
        dy = positions[:, 1] - p[1]
        return float(np.sum(weights * (np.sqrt(dx**2 + dy**2) - distances) ** 2))

    try:
        res = minimize(cost, [x0, y0], method="Nelder-Mead",
                       options={"maxiter": 1000, "xatol": 0.01, "fatol": 0.01})
        return res.x if (res.success or res.fun < 1.0) else None
    except Exception:
        return None

def _run_trilateration():
    """Called on every FRAME END from the serial thread."""
    global _rssi_ref, _path_loss_n, _last_diag_ts
    # Read slider values (Tkinter DoubleVars are thread-safe for .get())
    if _rssi_ref_var is not None:
        _rssi_ref    = _rssi_ref_var.get()
        _path_loss_n = _path_loss_var.get()

    with _lock:
        mac_pos   = {}
        macs_used = []
        for idx, node in _nodes.items():
            mac = node.get("mac", "").upper()
            if mac and MAC_RE.match(mac):
                macs_used.append(mac)
                mac_pos[mac] = (node["x"], node["y"])
        rssi_snap = {dm: dict(nm) for dm, nm in _rssi_buf.items()}

    now_ts = time.time()
    diag_due = (now_ts - _last_diag_ts) >= 10.0   # throttle diagnostics to 1/10 s

    # Detect duplicate node MACs
    dupes = [m for m in set(macs_used) if macs_used.count(m) > 1]
    if dupes and diag_due:
        ts = time.strftime("%H:%M:%S")
        for m in dupes:
            _serial_lines.append(
                f"[{ts}] ⚠ Duplicate node MAC {m} — click ⟳ on the duplicate slot to fix"
            )

    if len(mac_pos) < 3:
        if diag_due:
            ts = time.strftime("%H:%M:%S")
            _serial_lines.append(
                f"[{ts}] ⚠ Trilat skipped — only {len(mac_pos)} unique node MAC(s) "
                f"(need 3). Use ⟳ to cycle MACs or set manually and press ✓"
            )
            _last_diag_ts = now_ts
        return

    skipped_wid   = 0
    skipped_anch  = 0
    now = time.time()
    for dev_mac, node_obs in rssi_snap.items():
        valid = [(mac_pos[nm], obs)
                 for nm, obs in node_obs.items()
                 if nm in mac_pos]
        if len(valid) < 3:
            skipped_anch += 1
            continue
        wids = [v[1]["window_id"] for v in valid]
        if max(wids) - min(wids) > WINDOW_TOLERANCE:
            skipped_wid += 1
            continue

        positions = np.array([v[0] for v in valid], dtype=float)
        distances = np.array([_rssi_to_distance(v[1]["rssi"]) for v in valid], dtype=float)
        result    = _scipy_trilaterate(positions, distances)
        if result is None:
            continue

        ref_obs = valid[0][1]
        with _lock:
            _devices[dev_mac] = {
                "ssid":   ref_obs["ssid"],
                "x":      float(result[0]),
                "y":      float(result[1]),
                "random": ref_obs["is_random"],
                "ts":     now,
            }

    # Log skip reasons throttled to once every 10 s
    if diag_due and (skipped_anch > 0 or skipped_wid > 0):
        ts = time.strftime("%H:%M:%S")
        _serial_lines.append(
            f"[{ts}] ℹ Skipped: {skipped_anch} device(s) seen by <3 nodes, "
            f"{skipped_wid} filtered by window mismatch (tolerance={WINDOW_TOLERANCE})"
        )
        _last_diag_ts = now_ts

# ── Serial reader (background thread) ─────────────────────────────────────────
def _serial_reader(port: str, baud: int):
    global _ser
    while True:
        try:
            _ser = serial.Serial(port, baud, timeout=1)
            _serial_lines.append(f"── connected to {port} ──")
            _safe_after(lambda: _status_var.set(f"Connected — {port}"))

            while True:
                raw = _ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                line = re.sub(r"[\x00-\x08\x0b-\x1f\x7f]", "", line)
                _process_line(line)

        except serial.SerialException as e:
            _ser = None
            _serial_lines.append(f"── serial error: {e} — retry in 3 s ──")
            _safe_after(lambda: _status_var.set("Disconnected — retrying…"))
            time.sleep(3)
        except Exception as e:
            _ser = None
            _serial_lines.append(f"── error: {e} ──")
            time.sleep(3)

def _process_line(line: str):
    """Parse one line from the coordinator and route it appropriately.

    Structured protocol lines are parsed silently; a concise human-readable
    summary is appended to _serial_lines instead of the raw token string.
    All other coordinator output (ESP_LOGI, printf debug, errors) flows
    through unchanged.  Old entries are discarded automatically by the
    bounded deque (maxlen=150).
    """
    global _frame_rssi_count

    # ── Machine-readable: RSSI report ─────────────────────────────────────
    if line.startswith("RSSI|"):
        parts = line.split("|", 6)
        if len(parts) == 7:
            try:
                dev_mac  = parts[1].upper()
                is_rand  = int(parts[2]) == 1
                ssid     = parts[3]
                node_mac = parts[4].upper()
                rssi     = int(parts[5])
                wid      = int(parts[6])
                with _lock:
                    _rssi_buf.setdefault(dev_mac, {})[node_mac] = {
                        "rssi": rssi, "window_id": wid,
                        "is_random": is_rand, "ssid": ssid,
                        "ts": time.time(),
                    }
                _frame_rssi_count += 1
            except (ValueError, IndexError):
                pass
        return  # don't echo raw token line

    # ── Machine-readable: frame boundary ──────────────────────────────────
    elif line == "---FRAME END---":
        _run_trilateration()
        now = time.time()
        with _lock:
            n_dev = len(_devices)
            for m in [m for m, d in _devices.items() if now - d["ts"] > STALE_SECS]:
                del _devices[m]
            for dm in list(_rssi_buf):
                for nm in list(_rssi_buf[dm]):
                    if now - _rssi_buf[dm][nm]["ts"] > STALE_SECS:
                        del _rssi_buf[dm][nm]
                if not _rssi_buf[dm]:
                    del _rssi_buf[dm]
        ts = time.strftime("%H:%M:%S")
        _serial_lines.append(
            f"[{ts}] ── Frame: {_frame_rssi_count} RSSI reports  |  {n_dev} device(s) placed ──"
        )
        _frame_rssi_count = 0
        return

    # ── Machine-readable: node discovery ──────────────────────────────────
    elif line.startswith("DISC|"):
        parts = line.split("|")
        if len(parts) == 2 and MAC_RE.match(parts[1]):
            mac  = parts[1].upper()
            slot = None
            with _lock:
                if mac not in _discovered_macs:
                    _discovered_macs.append(mac)
                    slot = len(_discovered_macs)
                    if slot <= 3:
                        _nodes[str(slot)]["mac"] = mac
            if slot and slot <= 3:
                ts = time.strftime("%H:%M:%S")
                _serial_lines.append(f"[{ts}] 🔵 Node {slot} discovered: {mac}")
                _safe_after(lambda i=str(slot), m=mac: _mac_vars[i].set(m))
        return

    # ── Machine-readable: clock sync result ───────────────────────────────
    elif line.startswith("SYNC|"):
        parts = line.split("|")
        if len(parts) == 4 and MAC_RE.match(parts[1]):
            try:
                mac       = parts[1].upper()
                offset_ms = int(parts[2])
                rtt_ms    = int(parts[3])
                with _lock:
                    _sync_status[mac] = {
                        "offset_ms": offset_ms,
                        "rtt_ms":    rtt_ms,
                        "ts":        time.time(),
                    }
                ts = time.strftime("%H:%M:%S")
                _serial_lines.append(
                    f"[{ts}] ✔ Sync  {mac[-8:]}  offset={offset_ms:+d} ms  rtt={rtt_ms} ms"
                )
            except ValueError:
                pass
        return

    # ── Coordinator command acknowledgement ───────────────────────────────
    elif line.startswith("CMD_ACK:"):
        ts = time.strftime("%H:%M:%S")
        _serial_lines.append(f"[{ts}] ↩ {line}")
        return

    # ── Human-readable coordinator output (ESP_LOGI, printf, errors) ──────
    # Skip empty lines and pure whitespace to keep the log clean
    stripped = line.strip()
    if stripped:
        _serial_lines.append(stripped)


def _safe_after(fn):
    """Schedule fn() on the Tk main thread. Safe to call from any thread."""
    if _root:
        _root.after(0, fn)

# ── Canvas drawing ─────────────────────────────────────────────────────────────
_drag = {"idx": None}   # active drag state

def _draw_canvas():
    """Redraw canvas. Scheduled by _root.after() — always runs on main thread."""
    _canvas.delete("dyn")

    # Grid
    cw, ch = _cw(), _ch()
    for gx in range(int(WORLD_X[0]), int(WORLD_X[1]) + 1):
        cx, _ = _w2c(gx, 0)
        _canvas.create_line(cx, 0, cx, ch, fill=COL_GRID, tags="dyn")
    for gy in range(int(WORLD_Y[0]), int(WORLD_Y[1]) + 1):
        _, cy = _w2c(0, gy)
        _canvas.create_line(0, cy, cw, cy, fill=COL_GRID, tags="dyn")

    # Axis labels
    for gx in range(0, int(WORLD_X[1]) + 1, 2):
        cx, cy = _w2c(gx, WORLD_Y[0])
        _canvas.create_text(cx, cy - 10, text=f"{gx}m",
                            font=("Arial", 7), fill=COL_AXIS_LABEL, tags="dyn")
    for gy in range(0, int(WORLD_Y[1]) + 1, 2):
        cx, cy = _w2c(WORLD_X[0], gy)
        _canvas.create_text(cx + 16, cy, text=f"{gy}m",
                            font=("Arial", 7), fill=COL_AXIS_LABEL, tags="dyn")

    with _lock:
        nodes_snap   = {k: dict(v) for k, v in _nodes.items()}
        devices_snap = dict(_devices)
        sync_snap    = dict(_sync_status)

    # Draw tracked devices (circles)
    R = 9
    for dev_mac, d in devices_snap.items():
        cx, cy = _w2c(d["x"], d["y"])
        col = COL_RAND if d["random"] else COL_KNOWN
        _canvas.create_oval(cx-R, cy-R, cx+R, cy+R,
                            fill=col, outline="white", width=2, tags="dyn")
        label = dev_mac[-8:]
        _canvas.create_text(cx, cy - R - 7, text=label,
                            font=("Arial", 7), fill=col, tags="dyn")

    # Draw anchor nodes (triangles, draggable)
    T = 13
    for idx, node in nodes_snap.items():
        cx, cy = _w2c(node["x"], node["y"])
        col = {
            "default":  COL_NODE_DEFAULT,
            "dragging": COL_NODE_DRAGGING,
            "fixed":    COL_NODE_FIXED,
        }.get(node["state"], COL_NODE_DEFAULT)
        pts = [cx, cy - T, cx - T, cy + T, cx + T, cy + T]
        _canvas.create_polygon(pts, fill=col, outline="white", width=2, tags="dyn")
        mac_short = node["mac"][-5:] if node["mac"] else "??:??"
        _canvas.create_text(cx, cy + T + 9, text=f"N{idx} {mac_short}",
                            font=("Arial", 8, "bold"), fill=col, tags="dyn")
        # Sync badge: ✓ offset / ✗ no sync (stale > 60 s = grey)
        if node["mac"]:
            status = sync_snap.get(node["mac"].upper())
            if status and (time.time() - status["ts"]) < 60:
                badge = f"⟳ {status['offset_ms']:+d}ms"
                badge_col = "#00e676"   # green — recent sync
            else:
                badge = "⚠ no sync"
                badge_col = "#ff6b6b"   # red — never synced or stale
            _canvas.create_text(cx, cy + T + 20, text=badge,
                                font=("Arial", 7), fill=badge_col, tags="dyn")

    _root.after(UPDATE_MS, _draw_canvas)

# ── Drag ───────────────────────────────────────────────────────────────────────
def _nearest_node(cx, cy, threshold=22):
    with _lock:
        snap = {k: dict(v) for k, v in _nodes.items()}
    best, best_d = None, threshold
    for idx, node in snap.items():
        ncx, ncy = _w2c(node["x"], node["y"])
        d = ((cx - ncx)**2 + (cy - ncy)**2) ** 0.5
        if d < best_d:
            best, best_d = idx, d
    return best

def _on_press(event):
    idx = _nearest_node(event.x, event.y)
    if idx:
        _drag["idx"] = idx
        with _lock:
            _nodes[idx]["state"] = "dragging"

def _on_motion(event):
    idx = _drag.get("idx")
    if not idx:
        return
    wx, wy = _c2w(event.x, event.y)
    with _lock:
        _nodes[idx]["x"] = wx
        _nodes[idx]["y"] = wy

def _on_release(event):
    idx = _drag.pop("idx", None)
    if not idx:
        return
    wx, wy = _c2w(event.x, event.y)
    with _lock:
        _nodes[idx]["x"] = wx
        _nodes[idx]["y"] = wy
        _nodes[idx]["state"] = "fixed"

# ── MAC helpers ────────────────────────────────────────────────────────────────
_mac_vars    = {}
_rssi_ref_var = None
_path_loss_var = None
_status_var  = None

def _apply_mac(idx_str):
    mac = _mac_vars[idx_str].get().strip().upper()
    if MAC_RE.match(mac):
        with _lock:
            _nodes[idx_str]["mac"] = mac
        _serial_lines.append(f"Node {idx_str} MAC set: {mac}")
    else:
        _serial_lines.append(f"Invalid MAC: {mac}")

def _cycle_mac(idx_str):
    with _lock:
        opts = list(_discovered_macs)
        cur  = _nodes[idx_str].get("mac", "")
    if not opts:
        return
    try:
        nxt = opts[(opts.index(cur) + 1) % len(opts)]
    except ValueError:
        nxt = opts[0]
    _mac_vars[idx_str].set(nxt)
    with _lock:
        _nodes[idx_str]["mac"] = nxt

# ── Serial log updater ─────────────────────────────────────────────────────────
_autoscroll_var    = None   # tk.BooleanVar — set in _build_gui
_rendered_snapshot = []     # last deque snapshot rendered into the Text widget

def _update_log():
    """Sync the Text widget to _serial_lines.

    Uses a snapshot diff to:
    - Avoid rebuilding the widget when nothing changed
    - Detect deque wrap-around (cleared or rotated) and do a cheap full redraw
    - Prune widget lines that have been evicted from the bounded deque so the
      widget never grows beyond maxlen — keeps .see() and .insert() O(1)
    """
    global _rendered_snapshot
    current = list(_serial_lines)       # snapshot; at most maxlen=150 items

    if current == _rendered_snapshot:
        _root.after(300, _update_log)
        return

    n_old, n_new = len(_rendered_snapshot), len(current)
    # Simple append: current starts with the previous snapshot
    is_append = (n_new > n_old and current[:n_old] == _rendered_snapshot)

    _log_text.config(state=tk.NORMAL)

    if is_append:
        new_lines = current[n_old:]
        if _log_text.get("1.0", "1.1") != "":
            _log_text.insert(tk.END, "\n" + "\n".join(new_lines))
        else:
            _log_text.insert(tk.END, "\n".join(new_lines))
    else:
        # Deque was cleared or has rotated — full redraw (cheap: ≤150 lines)
        _log_text.delete("1.0", tk.END)
        if current:
            _log_text.insert(tk.END, "\n".join(current))

    # Prune widget to maxlen so it never grows beyond the deque's capacity.
    # Without this, the widget accumulates lines forever and .see() / .insert()
    # become O(N), stalling _draw_canvas on the same main thread.
    max_lines = _serial_lines.maxlen or 150
    widget_lines = int(_log_text.index("end-1c").split(".")[0])
    if widget_lines > max_lines:
        excess = widget_lines - max_lines
        _log_text.delete("1.0", f"{excess + 1}.0")

    if _autoscroll_var and _autoscroll_var.get():
        _log_text.see(tk.END)

    _log_text.config(state=tk.DISABLED)
    _rendered_snapshot = current
    _root.after(300, _update_log)


def _clear_plot():
    """Reset tracked devices and RSSI buffer — node positions/MACs are kept."""
    with _lock:
        _devices.clear()
        _rssi_buf.clear()
    _serial_lines.append("── plot cleared ──")

# ── GUI build ──────────────────────────────────────────────────────────────────
def _build_gui(port: str):
    global _root, _canvas, _log_text, _status_var
    global _mac_vars, _rssi_ref_var, _path_loss_var, _autoscroll_var

    _root = tk.Tk()
    _root.title("Indoor Positioning System — Visualizer")
    _root.resizable(True, True)
    _root.minsize(900, 560)
    _root.configure(bg=COL_BG)

    _status_var = tk.StringVar(value="Connecting…")

    # ── Left: canvas (fills all extra space when window is resized) ──────────────
    left = tk.Frame(_root, bg=COL_BG)
    left.pack(side=tk.LEFT, padx=8, pady=8, fill=tk.BOTH, expand=True)

    _canvas = tk.Canvas(left, width=CANVAS_W, height=CANVAS_H,
                        bg=COL_BG, highlightthickness=1,
                        highlightbackground="#333355", cursor="crosshair")
    _canvas.pack(fill=tk.BOTH, expand=True)
    _canvas.bind("<ButtonPress-1>",   _on_press)
    _canvas.bind("<B1-Motion>",       _on_motion)
    _canvas.bind("<ButtonRelease-1>", _on_release)

    # Legend
    leg = tk.Frame(left, bg=COL_BG)
    leg.pack(anchor=tk.W, pady=(4, 0))
    for col, label in [
        (COL_NODE_DEFAULT,  "Anchor — default"),
        (COL_NODE_DRAGGING, "Anchor — dragging"),
        (COL_NODE_FIXED,    "Anchor — set ✓"),
        (COL_KNOWN,         "Device — stable MAC"),
        (COL_RAND,          "Device — random MAC"),
    ]:
        f = tk.Frame(leg, bg=COL_BG)
        f.pack(side=tk.LEFT, padx=5)
        tk.Label(f, bg=col, width=2, height=1).pack(side=tk.LEFT)
        tk.Label(f, text=label, font=("Arial", 8), fg=COL_TEXT,
                 bg=COL_BG).pack(side=tk.LEFT)

    # ── Right: controls ──────────────────────────────────────────────────────────
    right = tk.Frame(_root, bg=COL_BG)
    right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)

    tk.Label(right, textvariable=_status_var, font=("Arial", 9, "italic"),
             fg="#aaaacc", bg=COL_BG).pack(anchor=tk.W)

    # ── Anchor nodes
    ttk.Separator(right, orient="horizontal").pack(fill=tk.X, pady=5)
    tk.Label(right, text="Anchor nodes", font=("Arial", 10, "bold"),
             fg=COL_TEXT, bg=COL_BG).pack(anchor=tk.W)

    _mac_vars = {}
    for idx in ("1", "2", "3"):
        row = tk.Frame(right, bg=COL_BG)
        row.pack(fill=tk.X, pady=2)
        tk.Label(row, text=f"Node {idx}:", width=7, anchor=tk.W,
                 fg=COL_TEXT, bg=COL_BG).pack(side=tk.LEFT)
        v = tk.StringVar(value="")
        _mac_vars[idx] = v
        e = tk.Entry(row, textvariable=v, width=20,
                     font=("Courier", 9), bg="#252540", fg=COL_TEXT,
                     insertbackground=COL_TEXT)
        e.pack(side=tk.LEFT)
        e.bind("<Return>", lambda _, i=idx: _apply_mac(i))
        tk.Button(row, text="✓", width=2, bg="#2a2a4a", fg=COL_TEXT,
                  command=lambda i=idx: _apply_mac(i)).pack(side=tk.LEFT, padx=2)
        tk.Button(row, text="⟳", width=2, bg="#2a2a4a", fg=COL_TEXT,
                  command=lambda i=idx: _cycle_mac(i)).pack(side=tk.LEFT)

    # ── Path-loss sliders
    ttk.Separator(right, orient="horizontal").pack(fill=tk.X, pady=5)
    tk.Label(right, text="Path-loss parameters", font=("Arial", 10, "bold"),
             fg=COL_TEXT, bg=COL_BG).pack(anchor=tk.W)

    _rssi_ref_var  = tk.DoubleVar(value=_rssi_ref)
    _path_loss_var = tk.DoubleVar(value=_path_loss_n)

    for label, var, lo, hi, fmt in [
        ("RSSI @ 1 m (dBm)", _rssi_ref_var,  -80.0, -20.0, "{:.0f} dBm"),
        ("Path-loss n",      _path_loss_var,   1.5,   5.0,  "{:.1f}"),
    ]:
        tk.Label(right, text=label, font=("Arial", 9),
                 fg=COL_TEXT, bg=COL_BG).pack(anchor=tk.W)
        row = tk.Frame(right, bg=COL_BG)
        row.pack(fill=tk.X, pady=(0, 4))
        s = ttk.Scale(row, from_=lo, to=hi, variable=var,
                      orient=tk.HORIZONTAL, length=190)
        s.pack(side=tk.LEFT)
        readout = tk.Label(row, text=fmt.format(var.get()), width=9,
                           font=("Courier", 9), fg=COL_KNOWN, bg=COL_BG)
        readout.pack(side=tk.LEFT, padx=4)

        def _trace(n, i, m, v=var, lbl=readout, f=fmt):
            lbl.config(text=f.format(v.get()))
        var.trace_add("write", _trace)

    # ── Device count + Refresh plot button
    ttk.Separator(right, orient="horizontal").pack(fill=tk.X, pady=5)
    dev_row = tk.Frame(right, bg=COL_BG)
    dev_row.pack(fill=tk.X)
    _dev_count_var = tk.StringVar(value="Tracked devices: 0")
    tk.Label(dev_row, textvariable=_dev_count_var, font=("Arial", 9),
             fg=COL_TEXT, bg=COL_BG).pack(side=tk.LEFT)
    tk.Button(dev_row, text="↺ Refresh plot", font=("Arial", 8),
              bg="#2a2a4a", fg=COL_NODE_FIXED, activebackground="#3a3a6a",
              relief=tk.FLAT, padx=6, command=_clear_plot).pack(side=tk.RIGHT)

    def _do_sync():
        if _ser and _ser.is_open:
            try:
                _ser.write(b"CMD:SYNC\n")
                _ser.flush()
                ts = time.strftime("%H:%M:%S")
                _serial_lines.append(f"[{ts}] → CMD:SYNC sent")
            except Exception as e:
                _serial_lines.append(f"[!] Sync send failed: {e}")
        else:
            _serial_lines.append("[!] Not connected — cannot send CMD:SYNC")

    tk.Button(dev_row, text="⚡ Sync clocks", font=("Arial", 8),
              bg="#2a2a4a", fg="#ffd740", activebackground="#3a3a6a",
              relief=tk.FLAT, padx=6, command=_do_sync).pack(side=tk.RIGHT, padx=(0, 4))

    def _update_dev_count():
        with _lock:
            n = len(_devices)
        _dev_count_var.set(f"Tracked devices: {n}")
        _root.after(1000, _update_dev_count)
    _root.after(1000, _update_dev_count)

    # ── Serial log
    ttk.Separator(right, orient="horizontal").pack(fill=tk.X, pady=5)
    log_hdr = tk.Frame(right, bg=COL_BG)
    log_hdr.pack(fill=tk.X, pady=(0, 2))
    tk.Label(log_hdr, text="Serial log", font=("Arial", 10, "bold"),
             fg=COL_TEXT, bg=COL_BG).pack(side=tk.LEFT)
    # Auto-scroll toggle — on by default
    _autoscroll_var = tk.BooleanVar(value=True)
    tk.Checkbutton(log_hdr, text="Auto-scroll", variable=_autoscroll_var,
                   font=("Arial", 8), fg=COL_TEXT, bg=COL_BG,
                   selectcolor="#252540", activebackground=COL_BG).pack(side=tk.LEFT, padx=8)
    def _do_clear_log():
        global _rendered_snapshot
        _serial_lines.clear()
        _rendered_snapshot = []
        _log_text.config(state=tk.NORMAL)
        _log_text.delete("1.0", tk.END)
        _log_text.config(state=tk.DISABLED)

    tk.Button(log_hdr, text="Clear log", font=("Arial", 8),
              bg="#2a2a4a", fg="#ff6b6b", activebackground="#3a3a6a",
              relief=tk.FLAT, padx=4,
              command=_do_clear_log).pack(side=tk.RIGHT)
    log_frame = tk.Frame(right, bg=COL_BG)
    log_frame.pack(fill=tk.BOTH, expand=True)
    sb = ttk.Scrollbar(log_frame)
    sb.pack(side=tk.RIGHT, fill=tk.Y)
    _log_text = tk.Text(log_frame, height=22, width=44,
                        font=("Courier", 8), yscrollcommand=sb.set,
                        state=tk.DISABLED, bg=COL_TERM_BG, fg=COL_TERM_FG,
                        insertbackground=COL_TERM_FG)
    _log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    sb.config(command=_log_text.yview)

    return _root

# ── Port-picker dialog ────────────────────────────────────────────────────────
def _pick_port_dialog() -> tuple[str, int]:
    """Show a small dialog to choose serial port + baud rate.
    Returns (port, baud) or exits if the user cancels."""
    dialog = tk.Tk()
    dialog.title("IPS Visualizer — Connect")
    dialog.resizable(False, False)
    dialog.configure(bg="#1a1a2e")

    style = ttk.Style(dialog)
    style.theme_use("clam")
    style.configure("TLabel",  background="#1a1a2e", foreground="#e0e0ff", font=("Helvetica", 11))
    style.configure("TButton", font=("Helvetica", 10, "bold"))
    style.configure("TCombobox", font=("Helvetica", 11))

    frm = tk.Frame(dialog, bg="#1a1a2e", padx=20, pady=16)
    frm.pack()

    tk.Label(frm, text="🛰  Indoor Positioning System",
             bg="#1a1a2e", fg="#00d4ff",
             font=("Helvetica", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 12))

    # Port row
    ttk.Label(frm, text="Serial port:").grid(row=1, column=0, sticky="w", pady=4)
    ports = [p.device for p in _list_ports.comports()] if _HAS_LIST_PORTS else []
    port_var = tk.StringVar(value=ports[0] if ports else DEFAULT_PORT)
    port_cb  = ttk.Combobox(frm, textvariable=port_var, values=ports, width=28)
    port_cb.grid(row=1, column=1, pady=4, padx=(8, 0))

    def _refresh_ports():
        if _HAS_LIST_PORTS:
            new = [p.device for p in _list_ports.comports()]
            port_cb["values"] = new
            if new and port_var.get() not in new:
                port_var.set(new[0])

    ttk.Button(frm, text="⟳", width=3, command=_refresh_ports).grid(row=1, column=2, padx=(4, 0))

    # Baud row
    ttk.Label(frm, text="Baud rate:").grid(row=2, column=0, sticky="w", pady=4)
    baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
    ttk.Combobox(frm, textvariable=baud_var,
                 values=["9600", "19200", "38400", "57600", "115200", "230400"],
                 width=28).grid(row=2, column=1, pady=4, padx=(8, 0))

    # Connect button
    result = {"port": None, "baud": DEFAULT_BAUD}

    def _on_connect():
        result["port"] = port_var.get().strip()
        result["baud"] = int(baud_var.get())
        dialog.destroy()

    def _on_cancel():
        dialog.destroy()

    btn_frm = tk.Frame(frm, bg="#1a1a2e")
    btn_frm.grid(row=3, column=0, columnspan=3, pady=(14, 0))
    ttk.Button(btn_frm, text="Connect", command=_on_connect).pack(side="left", padx=6)
    ttk.Button(btn_frm, text="Cancel",  command=_on_cancel).pack(side="left", padx=6)

    dialog.bind("<Return>", lambda _: _on_connect())
    dialog.protocol("WM_DELETE_WINDOW", _on_cancel)

    # Centre on screen
    dialog.update_idletasks()
    w, h = dialog.winfo_width(), dialog.winfo_height()
    sw, sh = dialog.winfo_screenwidth(), dialog.winfo_screenheight()
    dialog.geometry(f"+{(sw-w)//2}+{(sh-h)//2}")

    dialog.mainloop()

    if result["port"] is None:
        sys.exit(0)
    return result["port"], result["baud"]


# ── Entry point ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="IPS Visualizer")
    parser.add_argument("--port", default=None,
                        help="Serial port (omit to use the interactive picker)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    # If no --port given (e.g. launched as a .app), show the picker dialog
    if args.port is None:
        port, baud = _pick_port_dialog()
    else:
        port, baud = args.port, args.baud

    root = _build_gui(port)

    threading.Thread(target=_serial_reader, args=(port, baud),
                     daemon=True).start()

    root.after(UPDATE_MS, _draw_canvas)
    root.after(500, _update_log)
    root.mainloop()

if __name__ == "__main__":
    main()
