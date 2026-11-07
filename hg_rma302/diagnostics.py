"""Fault records and optional nonblocking drive-session recording."""

import json
import math
import queue
import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path


FAULT_LOG = Path("artifacts/diagnostics/esc-faults.jsonl")


def write_fault_events(events, path=FAULT_LOG):
    if not events:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        for event in events:
            file.write(json.dumps({"logged_at": datetime.now(timezone.utc).isoformat(), **event},
                                  ensure_ascii=False) + "\n")


def _json_value(value):
    # Initial event times are -inf; keep files valid JSON even on bad input.
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class DriveLog:
    """Bounded queue: disk writes and JSON encoding never block motor control.

    Create/open before touching hardware. During operation a full queue drops
    records, and a write error disables logging; neither interrupts driving.
    close() is called only AFTER motor-stop/port cleanup.
    """

    def __init__(self, path="", *, queue_size=2048):
        if not path:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            path = Path("artifacts/diagnostics") / f"drive-{stamp}.jsonl"
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("x", encoding="utf-8")
        self._queue = queue.Queue(maxsize=queue_size)
        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._write, name="hg-cart-log", daemon=True)
        self.dropped_records = 0
        self.error = None
        self._warned_error = False
        self._warned_drops = False
        self._thread.start()

    def record(self, kind, *, now=None, **fields):
        if self.error or self._closing.is_set():
            return
        item = {"type": kind, "monotonic_s": time.monotonic() if now is None else now, **fields}
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.dropped_records += 1

    def snapshot(self, sample, output, pair, now, *, kind="sample", power=None):
        wheels = {}
        if pair:
            for side, wheel in zip(("left", "right"), pair.wheels):
                wheels[side] = {
                    "port": wheel.port.path,
                    "phase": wheel.phase,
                    "last_sent_mrpm": wheel.last_sent_rpm,
                    "pending_ack_mrpm": wheel.pending_rpm,
                    "status_age_s": now - wheel.last_status_at if wheel.last_status else None,
                    "status": dict(wheel.last_status) if wheel.last_status else None,
                }
        self.record(kind, now=now,
                    **({'power': power.snapshot(now)} if power is not None else {}),
                    input=asdict(sample) if sample is not None else None,
                    output=asdict(output) if output is not None else None,
                    wheels=wheels,
                    wheel_sync={"reason": pair.speed_sync.reason,
                                "left_rpm": pair.speed_sync.targets[0],
                                "right_rpm": pair.speed_sync.targets[1]}
                    if pair and hasattr(pair, 'speed_sync') else None,
                    recovery={"retry_count": pair.retry_count, "blocked": pair.retry_blocked,
                              "wait_remaining_s": max(0, pair.retry_after-now)} if pair else None)

    def warning(self):
        if self.error and not self._warned_error:
            self._warned_error = True
            return f"ログ保存に失敗しました: {self.path}: {self.error}（走行制御は継続）"
        if self.dropped_records and not self._warned_drops:
            self._warned_drops = True
            return f"ログ保存が追いつかず一部を省略しています: {self.path}（走行制御は継続）"
        return None

    def _write(self):
        written = 0
        flushed_at = time.monotonic()
        try:
            while not self._closing.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=.1)
                except queue.Empty:
                    item = None
                if item is not None:
                    self._file.write(json.dumps(_json_value(item), ensure_ascii=False, allow_nan=False) + "\n")
                    written += 1
                now = time.monotonic()
                if now - flushed_at >= .5:
                    self._file.flush()
                    flushed_at = now
            self._file.write(json.dumps({"type": "log_end", "records_written": written,
                                         "dropped_records": self.dropped_records}) + "\n")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                self._file.close()
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"

    def close(self):
        self._closing.set()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            return f"ログ書き込みの終了待ちがタイムアウトしました: {self.path}（未保存データの可能性あり）"
        if self.error:
            return f"ログ保存に失敗しました: {self.path}: {self.error}"
        if self.dropped_records:
            return f"ログの省略件数: {self.dropped_records}: {self.path}"
        return None
