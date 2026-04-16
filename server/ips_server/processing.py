from __future__ import annotations

import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class Reading:
    node_id: str
    mac: str
    rssi: float
    timestamp: float


class RSSIWindowProcessor:
    def __init__(self, window_ms: int = 200):
        self.window_seconds = window_ms / 1000.0
        self._store: Dict[str, Dict[str, Reading]] = defaultdict(dict)

    def add_reading(self, reading: Reading) -> None:
        self._store[reading.mac][reading.node_id] = reading
        self._evict_expired()

    def ready_windows(self, minimum_nodes: int = 3) -> Dict[str, Dict[str, float]]:
        self._evict_expired()
        windows: Dict[str, Dict[str, float]] = {}
        for mac, node_readings in self._store.items():
            if len(node_readings) >= minimum_nodes:
                windows[mac] = {node_id: data.rssi for node_id, data in node_readings.items()}
        return windows

    def _evict_expired(self) -> None:
        cutoff = time.time() - self.window_seconds
        empty_macs: List[str] = []
        for mac, readings in list(self._store.items()):
            filtered = {
                node_id: reading
                for node_id, reading in readings.items()
                if reading.timestamp >= cutoff
            }
            self._store[mac] = filtered
            if not filtered:
                empty_macs.append(mac)

        for mac in empty_macs:
            del self._store[mac]
