from __future__ import annotations

import ast
import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class G1NavUdpCmdBridgeConfig:
    host: str = "127.0.0.1"
    port: int = 18080
    default_command: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.8)
    stale_timeout_sec: float = 0.5
    socket_timeout_sec: float = 0.05


def parse_udp_command_payload(data: bytes) -> list[float]:
    packet = json.loads(data.decode("utf-8"))
    if isinstance(packet, dict) and isinstance(packet.get("payload"), str):
        command = ast.literal_eval(packet["payload"])
    elif isinstance(packet, dict) and isinstance(packet.get("command"), list):
        command = packet["command"]
    else:
        raise ValueError("UDP command packet must contain payload string or command list")

    return _coerce_command(command)


class G1NavUdpCmdBridge:
    def __init__(self, run_command_dds, config: G1NavUdpCmdBridgeConfig | None = None):
        self._run_command_dds = run_command_dds
        self._config = config or G1NavUdpCmdBridgeConfig()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._last_valid_time = time.monotonic()
        self._stale_command_written = False
        self.port = int(self._config.port)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self._config.host, int(self._config.port)))
        self._socket.settimeout(float(self._config.socket_timeout_sec))
        self.port = int(self._socket.getsockname()[1])
        self._last_valid_time = time.monotonic()
        self._stale_command_written = False
        self._running.set()
        self._thread = threading.Thread(
            target=self._run,
            name="g1_nav_udp_cmd_bridge",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._socket = None
        self._thread = None

    def _run(self) -> None:
        while self._running.is_set():
            try:
                data, _addr = self._socket.recvfrom(4096)
            except socket.timeout:
                self._write_default_if_stale()
                continue
            except OSError:
                break

            try:
                command = parse_udp_command_payload(data)
            except Exception as e:
                print(f"[nav_udp_cmd] ignored malformed packet: {e}")
                continue

            self._run_command_dds.write_run_command(command)
            self._last_valid_time = time.monotonic()
            self._stale_command_written = False

    def _write_default_if_stale(self) -> None:
        if self._stale_command_written:
            return
        elapsed = time.monotonic() - self._last_valid_time
        if elapsed < float(self._config.stale_timeout_sec):
            return
        self._run_command_dds.write_run_command(list(self._config.default_command))
        self._stale_command_written = True


def _coerce_command(command: Sequence[float]) -> list[float]:
    if not isinstance(command, (list, tuple)) or len(command) < 4:
        raise ValueError("command must be a list with at least four values")
    return [float(value) for value in command[:4]]
