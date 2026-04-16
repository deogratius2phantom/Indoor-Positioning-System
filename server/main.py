import asyncio
import json
import time
from pathlib import Path
from typing import Any, Tuple

import numpy as np

from ips_server.config import load_config, node_positions
from ips_server.kalman import PositionKalmanFilter
from ips_server.processing import RSSIWindowProcessor, Reading
from ips_server.trilateration import TrilaterationSolver

MIN_EPOCH_TIMESTAMP = 1_000_000_000


class UDPCollectorProtocol(asyncio.DatagramProtocol):
    def __init__(self, queue: asyncio.Queue[dict]):
        self.queue = queue

    def datagram_received(self, data: bytes, addr: Tuple[Any, int]) -> None:
        try:
            message = json.loads(data.decode("utf-8"))
            self.queue.put_nowait(message)
        except (UnicodeDecodeError, json.JSONDecodeError):
            return


async def process_loop(queue: asyncio.Queue[dict], config_path: Path) -> None:
    config = load_config(config_path)
    anchors = node_positions(config)

    processor = RSSIWindowProcessor(window_ms=200)
    solver = TrilaterationSolver(anchors)
    smoothers: dict[str, PositionKalmanFilter] = {}

    while True:
        message = await queue.get()
        timestamp = time.time()
        if "ts_ms" in message:
            try:
                candidate = float(message["ts_ms"]) / 1000.0
                # Use device timestamp when it looks like epoch time.
                if candidate > MIN_EPOCH_TIMESTAMP:
                    timestamp = candidate
            except (TypeError, ValueError):
                print(f"Invalid ts_ms value in message: {message!r}")
                pass

        try:
            reading = Reading(
                node_id=message["node_id"],
                mac=message["mac"],
                rssi=float(message["rssi"]),
                timestamp=timestamp,
            )
        except (KeyError, TypeError, ValueError):
            print(f"Invalid payload, skipping message: {message!r}")
            continue
        processor.add_reading(reading)

        for mac, rssi_window in processor.ready_windows().items():
            estimate = solver.solve(rssi_window)
            if estimate is None:
                continue

            if mac not in smoothers:
                smoothers[mac] = PositionKalmanFilter()

            smoothed = smoothers[mac].update(np.asarray(estimate, dtype=float))
            print(f"MAC={mac} raw={estimate.round(3)} filtered={smoothed.round(3)}")


async def main() -> None:
    config_path = Path(__file__).with_name("config.yaml")
    config = load_config(config_path)

    listen_ip = config["server"]["listen_ip"]
    listen_port = int(config["server"]["listen_port"])

    queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=5000)
    loop = asyncio.get_running_loop()

    transport, _ = await loop.create_datagram_endpoint(
        lambda: UDPCollectorProtocol(queue),
        local_addr=(listen_ip, listen_port),
    )
    print(f"Listening on udp://{listen_ip}:{listen_port}")

    try:
        await process_loop(queue, config_path)
    finally:
        transport.close()


if __name__ == "__main__":
    asyncio.run(main())
