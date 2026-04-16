from pathlib import Path
from typing import Dict, Tuple

import yaml


def load_config(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def node_positions(config: dict) -> Dict[str, Tuple[float, float, float]]:
    positions: Dict[str, Tuple[float, float, float]] = {}
    for node_id, position in config["nodes"].items():
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError(f"Node '{node_id}' must have exactly 3 coordinates.")
        positions[node_id] = (float(position[0]), float(position[1]), float(position[2]))
    return positions
