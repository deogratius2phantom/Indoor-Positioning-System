from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple, Union

import yaml


@dataclass(frozen=True)
class SerialTransportConfig:
    ports: tuple[str, ...]
    baudrate: int


@dataclass(frozen=True)
class ProcessingConfig:
    window_ms: int
    minimum_nodes: int


def load_config(path: Union[str, Path]) -> dict:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def serial_transport(config: dict) -> SerialTransportConfig:
    serial_config = config.get("transport", {}).get("serial")
    if not isinstance(serial_config, dict):
        raise ValueError("Config must define transport.serial.")

    raw_ports = serial_config.get("ports")
    if not isinstance(raw_ports, list) or not raw_ports or not all(isinstance(port, str) and port for port in raw_ports):
        raise ValueError("transport.serial.ports must be a non-empty list of device paths.")

    baudrate = int(serial_config.get("baudrate", 921600))
    if baudrate <= 0:
        raise ValueError("transport.serial.baudrate must be a positive integer.")

    return SerialTransportConfig(ports=tuple(raw_ports), baudrate=baudrate)


def processing_config(config: dict) -> ProcessingConfig:
    raw_processing = config.get("processing", {})
    if not isinstance(raw_processing, dict):
        raise ValueError("processing must be a mapping.")

    window_ms = int(raw_processing.get("window_ms", 200))
    minimum_nodes = int(raw_processing.get("minimum_nodes", 3))

    if window_ms <= 0:
        raise ValueError("processing.window_ms must be a positive integer.")
    if minimum_nodes < 3:
        raise ValueError("processing.minimum_nodes must be at least 3 for trilateration.")

    return ProcessingConfig(window_ms=window_ms, minimum_nodes=minimum_nodes)


def node_positions(config: dict) -> Dict[str, Tuple[float, float, float]]:
    positions: Dict[str, Tuple[float, float, float]] = {}
    for node_id, position in config["nodes"].items():
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError(f"Node '{node_id}' must have exactly 3 coordinates, got {position!r}.")
        positions[node_id] = (float(position[0]), float(position[1]), float(position[2]))
    return positions
