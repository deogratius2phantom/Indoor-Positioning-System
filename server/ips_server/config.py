from pathlib import Path
from typing import Dict, Tuple

import yaml


def load_config(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def node_positions(config: dict) -> Dict[str, Tuple[float, float, float]]:
    return {
        node_id: tuple(position)  # type: ignore[arg-type]
        for node_id, position in config["nodes"].items()
    }
