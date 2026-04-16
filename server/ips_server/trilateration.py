from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class RSSIModel:
    tx_power_dbm: float = -40.0
    path_loss_exponent: float = 2.0

    def rssi_to_distance(self, rssi: float) -> float:
        return 10 ** ((self.tx_power_dbm - rssi) / (10 * self.path_loss_exponent))


class TrilaterationSolver:
    def __init__(self, node_positions: Dict[str, Tuple[float, float, float]], model: RSSIModel | None = None):
        self.node_positions = node_positions
        self.model = model or RSSIModel()

    def solve(self, rssi_by_node: Dict[str, float]) -> np.ndarray | None:
        active_nodes = [(node_id, rssi) for node_id, rssi in rssi_by_node.items() if node_id in self.node_positions]
        if len(active_nodes) < 3:
            return None

        anchors = np.array([self.node_positions[node_id] for node_id, _ in active_nodes], dtype=float)
        distances = np.array([self.model.rssi_to_distance(rssi) for _, rssi in active_nodes], dtype=float)

        initial_guess = np.mean(anchors, axis=0)

        def residuals(point: Iterable[float]) -> np.ndarray:
            point_array = np.array(point, dtype=float)
            estimated = np.linalg.norm(anchors - point_array, axis=1)
            return estimated - distances

        result = least_squares(residuals, x0=initial_guess)
        return result.x if result.success else None
