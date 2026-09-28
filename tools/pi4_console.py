#!/usr/bin/env python3
"""Full-duplex Pi4 terminal for text and Pilot widget events on one TCP connection."""

import argparse
import curses
from collections import deque
from datetime import datetime, timezone
import locale
import queue
import threading
import time
from typing import Tuple
import unicodedata

from m400_pi4_client import M400Pi4Client

MIN_M3_TIME_NS = 1_577_836_800_000_000_000  # 2020-01-01 UTC
MAX_M3_TIME_NS = 4_102_444_800_000_000_000  # 2100-01-01 UTC


class Link:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connection = None

    def set(self, connection: M400Pi4Client) -> None:
        with self.lock:
            self.connection = connection

    def clear(self, connection: M400Pi4Client) -> None:
        with self.lock:
            if self.connection is connection:
                self.connection = None

    def send_text(self, message: str) -> None:
        with self.lock:
            if self.connection is None:
                raise ConnectionError("尚未连接妙算 3")
            self.connection.send_text(message)

    def send_time_request(self) -> int:
        with self.lock:
            if self.connection is None:
                raise ConnectionError("尚未连接妙算 3")
            return self.connection.send_time_request()

    def close(self) -> None:
        with self.lock:
            if self.connection is not None:
                self.connection.close()
                self.connection = None


def format_offset(offset_ns: int) -> str:
    if abs(offset_ns) >= 1_000_000_000:
        return f"{offset_ns / 1_000_000_000:+.3f}s"
    return f"{offset_ns / 1_000_000:+.2f}ms"


def estimate_offset(origin: int, received: int, transmitted: int,
                    destination: int) -> Tuple[int, int]:
    """Return M3-minus-Pi4 offset and round-trip delay in nanoseconds."""
    offset = ((received - origin) + (transmitted - destination)) // 2
    delay = (destination - origin) - (transmitted - received)
    return offset, delay


def receive_loop(host: str, port: int, link: Link, updates: queue.Queue,
                 stop: threading.Event, sync_enabled: threading.Event) -> None:
    while not stop.is_set():
        connection = None
        try:
            connection = M400Pi4Client(host, port)
            connection.connect()
            link.set(connection)
            updates.put(("status", "已连接"))
            samples = []
            pending_origin = None
            attempts = 0
            for line in connection.lines():
                destination = time.time_ns()
                if line == "PING":
                    continue
                if line == "TIME_POLL":
                    samples.clear()
                    pending_origin = link.send_time_request()
                    attempts = 1
                elif line.startswith("TIME_RESP\t"):
                    try:
                        _, origin_text, receive_text, transmit_text = line.split("\t")
                        origin, received, transmitted = map(
                            int, (origin_text, receive_text, transmit_text))
                    except ValueError:
                        continue
                    if origin != pending_origin:
                        continue
                    pending_origin = None
                    offset, delay = estimate_offset(origin, received,
                                                    transmitted, destination)
                    if delay >= 0:
                        samples.append((delay, offset, transmitted))
                    if len(samples) < 3 and attempts < 5:
                        pending_origin = link.send_time_request()
                        attempts += 1
                        continue
                    if not samples:
                        updates.put(("time", "时间取样失败，等待下一次取样"))
                        continue
                    best_delay, best_offset, m3_time = min(samples)
                    time_status = (f"时差 {format_offset(best_offset)}  "
                                   f"RTT {best_delay / 1_000_000:.2f}ms")
                    plausible = MIN_M3_TIME_NS <= m3_time <= MAX_M3_TIME_NS
                    if plausible:
                        m3_utc = datetime.fromtimestamp(m3_time // 1_000_000_000,
                                                        timezone.utc).strftime("%H:%M:%S")
                        time_status += f"  M3 {m3_utc}Z"
                    else:
                        time_status += "  M3 时间异常，未校时"
                    if sync_enabled.is_set():
                        if not plausible:
                            pass
                        elif best_delay > 200_000_000:
                            time_status += "  RTT 过高，未校时"
                        elif abs(best_offset) >= 1_000_000:
                            try:
                                time.clock_settime_ns(time.CLOCK_REALTIME,
                                                      time.time_ns() + best_offset)
                                time_status += "  已校时"
                            except OSError as error:
                                sync_enabled.clear()
                                updates.put(("message", f"校时失败（需 sudo/CAP_SYS_TIME）: {error}"))
                        else:
                            time_status += "  无需校时"
                    updates.put(("time", time_status))
                elif line.startswith("ACK\t"):
                    updates.put(("message", "M3 确认: " + line[4:]))
                else:
                    updates.put(("message", "M3 → Pi4: " + line.replace("\t", "  ")))
        except OSError as error:
            if not stop.is_set():
                updates.put(("message", "连接中断: " + str(error)))
        finally:
            if connection is not None:
                link.clear(connection)
                connection.close()
        if not stop.is_set():
            updates.put(("status", "等待重连"))
            stop.wait(2)


def draw(stdscr, messages: deque, typed: str, status: str,
         time_status: str, sync_enabled: bool) -> None:
    stdscr.erase()
    rows, columns = stdscr.getmaxyx()
    if rows < 4 or columns < 16:
        stdscr.addnstr(0, 0, "终端窗口太小", max(0, columns - 1))
        stdscr.refresh()
        return

    def write(row: int, value: str) -> None:
        try:
            stdscr.addnstr(row, 0, value, columns - 1)
        except curses.error:
            pass

    write(0, f"M3 ↔ Pi4 {status} | {time_status}")
    write(1, f"校时:{'开' if sync_enabled else '关'}  Ctrl+T切换  Ctrl+C退出")
    visible = list(messages)[-(rows - 4):]
    for row, message in enumerate(visible, start=2):
        write(row, message)
    write(rows - 2, "─" * (columns - 1))
    prompt = "send to m3: "
    available = max(0, columns - len(prompt) - 2)
    shown = ""
    width = 0
    for character in reversed(typed):
        cells = 0 if unicodedata.combining(character) else (
            2 if unicodedata.east_asian_width(character) in "WF" else 1)
        if width + cells > available:
            break
        shown = character + shown
        width += cells
    write(rows - 1, prompt + shown)
    try:
        stdscr.move(rows - 1, min(columns - 2, len(prompt) + width))
    except curses.error:
        pass
    stdscr.refresh()


def terminal(stdscr, host: str, port: int, initial_sync: bool) -> None:
    try:
        curses.curs_set(1)
    except curses.error:
        pass
    stdscr.timeout(100)
    link = Link()
    updates = queue.Queue()
    stop = threading.Event()
    sync_enabled = threading.Event()
    if initial_sync:
        sync_enabled.set()
    worker = threading.Thread(target=receive_loop,
                              args=(host, port, link, updates, stop, sync_enabled), daemon=True)
    worker.start()
    messages = deque(maxlen=200)
    status = "连接中"
    time_status = "等待 M3 时间取样"
    typed = ""
    try:
        while True:
            try:
                while True:
                    kind, value = updates.get_nowait()
                    if kind == "status":
                        status = value
                    elif kind == "time":
                        time_status = value
                    else:
                        messages.append(value)
            except queue.Empty:
                pass
            draw(stdscr, messages, typed, status, time_status, sync_enabled.is_set())
            try:
                key = stdscr.get_wch()
            except curses.error:
                continue
            if key == "\x03":
                break
            if key == "\x14":
                if sync_enabled.is_set():
                    sync_enabled.clear()
                    messages.append("Pi4 自动校时已关闭；继续测量时差")
                else:
                    sync_enabled.set()
                    messages.append("Pi4 自动校时已开启；下次取样时调整系统时间")
                continue
            if key in ("\n", "\r", curses.KEY_ENTER):
                if typed:
                    try:
                        link.send_text(typed)
                        messages.append("Pi4 → M3: " + typed)
                        typed = ""
                    except (OSError, ValueError) as error:
                        messages.append("发送失败: " + str(error))
            elif key in ("\b", "\x7f", curses.KEY_BACKSPACE):
                typed = typed[:-1]
            elif isinstance(key, str) and key.isprintable():
                typed += key
    finally:
        stop.set()
        link.close()
        worker.join(timeout=1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifold_ip", help="妙算 3 的 eth0 地址，例如 10.88.77.54")
    parser.add_argument("--port", type=int, default=14560)
    parser.add_argument("--sync-time", action="store_true", help="启动时允许校准 Pi4 系统时间（需要 sudo）")
    args = parser.parse_args()
    locale.setlocale(locale.LC_ALL, "")
    try:
        curses.wrapper(terminal, args.manifold_ip, args.port, args.sync_time)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
