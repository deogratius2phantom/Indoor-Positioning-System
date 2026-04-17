import asyncio
import json
import time
from pathlib import Path
from typing import Any

import numpy as np
import serial_asyncio

from ips_server.config import load_config, node_positions, processing_config, serial_transport
from ips_server.kalman import PositionKalmanFilter
from ips_server.processing import RSSIWindowProcessor, Reading
from ips_server.trilateration import TrilaterationSolver

QueuedMessage = tuple[str, dict[str, Any], float]


async def read_serial_stream(port: str, baudrate: int, queue: asyncio.Queue[QueuedMessage]) -> None:
    try:
        reader, _ = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
    except Exception as exc:  # serial_asyncio surfaces backend-specific exceptions here
        raise RuntimeError(f"Failed to open serial port {port}: {exc}") from exc

    print(f"Connected serial reader on {port} @ {baudrate} baud")
    while True:
        raw_line = await reader.readline()
        if not raw_line:
            raise ConnectionError(f"Serial port {port} closed unexpectedly.")

        received_at = time.time()
        try:
            message = json.loads(raw_line.decode("utf-8").strip())
        except UnicodeDecodeError:
            print(f"Discarding non-UTF-8 data from {port}: {raw_line!r}")
            continue
        except json.JSONDecodeError:
            print(f"Discarding malformed JSON from {port}: {raw_line!r}")
            continue

        await queue.put((port, message, received_at))


async def process_loop(
    queue: asyncio.Queue[QueuedMessage],
    anchors: dict[str, tuple[float, float, float]],
    window_ms: int,
    minimum_nodes: int,
) -> None:
    processor = RSSIWindowProcessor(window_ms=window_ms)
    solver = TrilaterationSolver(anchors)
    smoothers: dict[str, PositionKalmanFilter] = {}

    while True:
        port, message, received_at = await queue.get()
        try:
            reading = Reading(
                node_id=message["node_id"],
                mac=message["mac"],
                rssi=float(message["rssi"]),
                timestamp=received_at,
            )
        except (KeyError, TypeError, ValueError):
            print(f"Missing/invalid required fields (node_id, mac, rssi) in message from {port}: {message!r}")
            continue
        processor.add_reading(reading)

        for mac, rssi_window in processor.ready_windows(minimum_nodes=minimum_nodes).items():
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
    anchors = node_positions(config)
    serial = serial_transport(config)
    processing = processing_config(config)

    queue: asyncio.Queue[QueuedMessage] = asyncio.Queue(maxsize=5000)
    processor_task = asyncio.create_task(
        process_loop(
            queue,
            anchors=anchors,
            window_ms=processing.window_ms,
            minimum_nodes=processing.minimum_nodes,
        ),
        name="process-loop",
    )
    serial_tasks = [
        asyncio.create_task(read_serial_stream(port, serial.baudrate, queue), name=f"serial:{port}")
        for port in serial.ports
    ]
    print(f"Waiting for {len(serial.ports)} serial transports")

    try:
        done, pending = await asyncio.wait(
            {processor_task, *serial_tasks},
            return_when=asyncio.FIRST_EXCEPTION,
        )
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)

        for task in done:
            exception = task.exception()
            if exception is not None:
                raise exception
    finally:
        if not processor_task.done():
            processor_task.cancel()
            await asyncio.gather(processor_task, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(main())
