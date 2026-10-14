"""Resolve ESC identity at connection time; never reconnect an active drive."""

import errno
import os
import time

from .config import Config

STMICRO_VID = 0x0483


class PortsUnavailable(RuntimeError):
    pass


def list_serial_ports():
    from serial.tools.list_ports import comports

    return list(comports())


def resolve_ports(config: Config, ports=None) -> tuple[str, str]:
    """One discovery snapshot. Exact serial + ST VID, never enumeration order."""
    if ports is None:
        ports = list_serial_ports() if config.left_serial or config.right_serial else []
    result, missing = [], []
    for side in ("left", "right"):
        serial_id = getattr(config, side + "_serial")
        path = getattr(config, side + "_port")
        if serial_id is not None:
            # A configured identity takes precedence over an old path. Never
            # fall back to that path when the intended ESC is unplugged.
            matches = {os.path.realpath(p.device) for p in ports
                       if p.vid == STMICRO_VID and p.serial_number == serial_id}
            if len(matches) > 1:
                raise ValueError(f"{side} ESC: USB serial={serial_id} が複数ポートに一致: {sorted(matches)}")
            if not matches:
                missing.append(f"{side} ESC serial={serial_id}")
                continue
            path = matches.pop()
        elif not path:
            raise ValueError(f"{side}_serial または {side}_port を設定してください")
        result.append(path)
    if missing:
        raise PortsUnavailable("未接続: " + ", ".join(missing))
    if os.path.realpath(result[0]) == os.path.realpath(result[1]):
        raise ValueError("Left and right ESC must use different devices")
    return tuple(result)


def _error_number(exc):
    # pySerial sometimes wraps termios errors without preserving errno, but
    # retains the original OSError as context.
    seen = set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        if getattr(exc, "errno", None) is not None:
            return exc.errno
        if exc.args and type(exc.args[0]) is int:
            return exc.args[0]
        exc = exc.__cause__ or exc.__context__
    return None


def open_ports(config: Config, port_factory):
    """Wait for re-enumeration, open both exclusively, recheck identity.

    Sends no UART commands. All retrying is confined to initial connection;
    a drive failure still stops both wheels and exits through EscPair.
    """
    deadline = time.monotonic() + config.connect_timeout_s
    while True:
        opened = []
        path = "ESC"
        try:
            paths = resolve_ports(config)
            for path in paths:
                opened.append(port_factory(path, config.baudrate))
            # Catch a disconnect / port-number reuse between discovery and
            # opening, before any stop, parameters, arm or run command.
            if config.left_serial or config.right_serial:
                if resolve_ports(config) != paths:
                    raise PortsUnavailable("接続中にESCのポートが変わりました")
            return opened
        except BaseException as exc:
            for port in opened:
                try:
                    port.close()
                except Exception:
                    pass  # Preserve the connection error; no drive was started.
            if isinstance(exc, PortsUnavailable):
                detail = str(exc)
            elif isinstance(exc, OSError):
                number = _error_number(exc)
                if number in (errno.EBUSY, errno.EAGAIN):
                    raise RuntimeError(f"{path}: ポート使用中。他の hg-cart・シリアルモニターを終了してください") from exc
                if number in (errno.EACCES, errno.EPERM):
                    detail = f"{path}: ポートのアクセス権限がありません（udev / dialout を確認）"
                elif number in (errno.ENOENT, errno.ENODEV, errno.ENXIO, errno.EIO):
                    detail = f"{path}: USBポートが切断されたか、まだ認識されていません"
                else:
                    raise
            else:
                raise
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PortsUnavailable(f"{detail}。接続待ち {config.connect_timeout_s:g} 秒でタイムアウト。"
                                       "接続状態: hg-cart devices --config <設定ファイル>") from exc
            time.sleep(min(.1, remaining))


def serial_device_report(config: Config | None = None) -> list[dict]:
    result = []
    for port in sorted(list_serial_ports(), key=lambda p: p.device):
        if port.vid is None:
            continue
        roles = []
        if config:
            for side in ("left", "right"):
                serial_id = getattr(config, side + "_serial")
                path = getattr(config, side + "_port")
                if ((serial_id is not None and port.vid == STMICRO_VID and port.serial_number == serial_id) or
                        (serial_id is None and path and os.path.realpath(path) == os.path.realpath(port.device))):
                    roles.append(side)
        result.append({"path": port.device, "serial": port.serial_number, "role": ",".join(roles) or None,
                       "usb_id": f"{port.vid:04x}:{port.pid:04x}" if port.pid is not None else None,
                       "description": port.description})
    return result
