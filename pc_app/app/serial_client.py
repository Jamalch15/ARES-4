from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from .config import SerialConfig


class SerialClientError(RuntimeError):
    pass


@dataclass
class SerialClient:
    config: SerialConfig
    connection: object | None = None

    @property
    def is_connected(self) -> bool:
        return self.connection is not None

    def connect(self, port: str | None = None, baud_rate: int | None = None) -> None:
        try:
            import serial
        except ImportError as exc:
            raise SerialClientError("pyserial is not installed") from exc

        if self.connection is not None:
            self.disconnect()

        selected_port = port or self.config.port
        selected_baud = baud_rate or self.config.baud_rate
        try:
            self.connection = serial.Serial(
                selected_port,
                selected_baud,
                timeout=self.config.timeout_s,
                write_timeout=self.config.timeout_s,
            )
        except Exception as exc:  # pyserial raises platform-specific exceptions.
            self.connection = None
            raise SerialClientError(f"could not open serial port {selected_port}: {exc}") from exc

    def disconnect(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def send_line(self, line: str) -> None:
        if self.connection is None:
            raise SerialClientError("serial port is not connected")
        self.connection.write((line.strip() + "\n").encode("ascii"))

    def read_line(self) -> str:
        if self.connection is None:
            raise SerialClientError("serial port is not connected")
        raw = self.connection.readline()
        return raw.decode("ascii", errors="replace").strip()

    def clear_input(self) -> None:
        if self.connection is None:
            raise SerialClientError("serial port is not connected")
        reset = getattr(self.connection, "reset_input_buffer", None)
        if callable(reset):
            reset()

    def read_until_prefix(self, prefix: str, timeout_s: float = 2.0) -> str:
        if self.connection is None:
            raise SerialClientError("serial port is not connected")

        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            line = self.read_line()
            if line.startswith(prefix):
                return line
        raise SerialClientError(f"timed out waiting for {prefix}")
