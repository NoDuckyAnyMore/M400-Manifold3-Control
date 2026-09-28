"""Reusable Pi4 client for the M400 Manifold 3 TCP text/event bridge.

The bridge displays text in Pilot and publishes widget events. It does not
interpret incoming text as flight-control commands.
"""

import socket
import threading
import time
from typing import Iterator, Optional


DEFAULT_PORT = 14560
MAX_MESSAGE_BYTES = 240


def _encode_text(message: str) -> bytes:
    payload = message.encode("utf-8")
    if not payload or len(payload) > MAX_MESSAGE_BYTES or b"\n" in payload or b"\r" in payload:
        raise ValueError("请输入一行不超过 240 字节的文字")
    return payload


def send_text_once(host: str, message: str, port: int = DEFAULT_PORT,
                   timeout: float = 3.0) -> str:
    """Send one Pilot-display message; return 'OK' or raise on rejection.

    This short-lived connection does not subscribe to Pilot widget events.
    """
    payload = _encode_text(message)
    with socket.create_connection((host, port), timeout=timeout) as connection:
        connection.sendall(payload + b"\n")
        with connection.makefile("rb") as incoming:
            reply = incoming.readline(128).decode("utf-8", errors="replace").strip()
    if reply != "OK":
        raise RuntimeError(f"妙算拒绝消息: {reply or '无应答'}")
    return reply


class M400Pi4Client:
    """Persistent full-duplex subscription; ACK and events arrive via lines()."""

    def __init__(self, host: str, port: int = DEFAULT_PORT) -> None:
        self.host = host
        self.port = port
        self._connection: Optional[socket.socket] = None
        self._incoming = None
        self._send_lock = threading.Lock()

    def connect(self, timeout: float = 3.0) -> None:
        if self._connection is not None:
            raise RuntimeError("已经连接")
        connection = socket.create_connection((self.host, self.port), timeout=timeout)
        try:
            connection.sendall(b"SUBSCRIBE_BUTTONS\n")
            incoming = connection.makefile("rb")
            if incoming.readline() != b"OK\n":
                incoming.close()
                raise ConnectionError("订阅被拒绝")
            connection.settimeout(None)
        except BaseException:
            connection.close()
            raise
        self._connection = connection
        self._incoming = incoming

    def send_text(self, message: str) -> None:
        """Send a text frame; read its asynchronous ACK from lines()."""
        payload = _encode_text(message)
        with self._send_lock:
            if self._connection is None:
                raise ConnectionError("尚未连接妙算 3")
            self._connection.sendall(b"TEXT\t" + payload + b"\n")

    def send_time_request(self) -> int:
        """Send a clock sample request and return the Pi4 origin timestamp (ns)."""
        with self._send_lock:
            if self._connection is None:
                raise ConnectionError("尚未连接妙算 3")
            origin = time.time_ns()
            self._connection.sendall(b"TIME_REQ\t" + str(origin).encode("ascii") + b"\n")
            return origin

    def lines(self) -> Iterator[str]:
        """Yield newline-delimited ACK, widget, keepalive and clock frames."""
        if self._incoming is None:
            raise ConnectionError("尚未连接妙算 3")
        for raw_line in self._incoming:
            yield raw_line.decode("utf-8", errors="replace").rstrip("\r\n")

    def close(self) -> None:
        with self._send_lock:
            connection = self._connection
            self._connection = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        if self._incoming is not None:
            self._incoming.close()
            self._incoming = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()
