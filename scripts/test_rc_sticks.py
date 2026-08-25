"""只读 SBUS 遥控器双操纵杆测试：CH3 前后、CH1 转向。"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import serial


_FRAME_HEADER = 0x0F
_FRAME_BYTES = 25
_CHANNEL_COUNT = 16
_CHANNEL_INPUT_MIN = 282
_CHANNEL_INPUT_MAX = 1772
_STICK_MIN = 282
_STICK_CENTER = 1002
_STICK_MAX = 1722


class _SbusFrameParser:
    """独立工具的有界 SBUS 字节流解析器，避免导入仿真依赖。"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, payload: bytes) -> tuple[tuple[int, ...], ...]:
        self._buffer.extend(payload)
        decoded: list[tuple[int, ...]] = []
        while True:
            try:
                start = self._buffer.index(_FRAME_HEADER)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < _FRAME_BYTES:
                break
            frame = bytes(self._buffer[:_FRAME_BYTES])
            bits = int.from_bytes(frame[1:23], "little")
            channels = tuple(
                (bits >> (11 * index)) & 0x07FF for index in range(_CHANNEL_COUNT)
            )
            if any(
                value < _CHANNEL_INPUT_MIN or value > _CHANNEL_INPUT_MAX
                for value in channels
            ):
                del self._buffer[0]
                continue
            del self._buffer[:_FRAME_BYTES]
            decoded.append(channels)
        return tuple(decoded)


def _stick_values(channels: tuple[int, ...]) -> tuple[int, int, float, float]:
    """返回 CH3 前后、CH1 转向的原始值与实测分段归一化值。"""
    def normalize(value: int) -> float:
        if value <= _STICK_CENTER:
            scaled = (value - _STICK_CENTER) / (_STICK_CENTER - _STICK_MIN)
        else:
            scaled = (value - _STICK_CENTER) / (_STICK_MAX - _STICK_CENTER)
        return max(-1.0, min(1.0, scaled))

    return channels[2], channels[0], normalize(channels[2]), normalize(channels[0])


def _port_argument(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("串口路径必须是绝对路径")
    return path


def _select_port(explicit_path: Path | None) -> Path:
    if explicit_path is not None:
        return explicit_path
    by_id_directory = Path("/dev/serial/by-id")
    candidates = tuple(sorted(path for path in by_id_directory.iterdir() if not path.is_dir())) if by_id_directory.is_dir() else ()
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("未发现 /dev/serial/by-id 设备；请使用 --port /dev/serial/by-id/<设备名>")
    names = ", ".join(str(path) for path in candidates)
    raise RuntimeError(f"发现多个稳定串口，请用 --port 明确选择：{names}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="只读取并显示 SBUS 遥控器 CH3 前后、CH1 转向")
    parser.add_argument("--port", type=_port_argument, help="稳定的绝对串口路径")
    parser.add_argument("--duration", type=float, default=None, help="测试秒数；省略则按 Ctrl-C 停止")
    args = parser.parse_args(argv)
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration 必须为正数")

    try:
        path = _select_port(args.port)
        reader = serial.Serial(str(path), baudrate=115200, timeout=0.02)
    except (serial.SerialException, ValueError, OSError) as error:
        print(f"无法打开遥控器串口：{error}", file=sys.stderr)
        return 2

    print(f"只读测试已连接：{path}")
    print("映射：CH3=左操纵杆前后；CH1=右操纵杆转向。不会发送任何控制命令。")
    print("输入允许 282..1772；校准 min/center/max=282/1002/1722。按 Ctrl-C 结束。")
    parser_state = _SbusFrameParser()
    deadline = None if args.duration is None else time.monotonic() + args.duration
    last_print_at = 0.0
    sample_count = 0
    ch1_min = ch3_min = _CHANNEL_INPUT_MAX
    ch1_max = ch3_max = _CHANNEL_INPUT_MIN
    exit_code = 0
    try:
        while deadline is None or time.monotonic() < deadline:
            for channels in parser_state.feed(reader.read(256)):
                sample_count += 1
                ch1_min, ch1_max = min(ch1_min, channels[0]), max(ch1_max, channels[0])
                ch3_min, ch3_max = min(ch3_min, channels[2]), max(ch3_max, channels[2])
                observed_at = time.monotonic()
                if observed_at - last_print_at < 0.1:
                    continue
                last_print_at = observed_at
                throttle_raw, steering_raw, throttle_normalized, steering_normalized = _stick_values(channels)
                print(
                    f"CH3 前后: {throttle_raw:4d} ({throttle_normalized:+.3f}) | "
                    f"CH1 转向: {steering_raw:4d} ({steering_normalized:+.3f})"
                )
    except (serial.SerialException, OSError) as error:
        print(f"\n遥控器串口读取失败：{error}", file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        print("\\n测试已停止。")
    finally:
        close = getattr(reader, "close", None)
        if callable(close):
            close()
    if sample_count:
        print(
            f"采样汇总：frames={sample_count} "
            f"CH1 min/max={ch1_min}/{ch1_max} CH3 min/max={ch3_min}/{ch3_max}"
        )
    else:
        print("采样汇总：未收到合法 SBUS 帧")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
