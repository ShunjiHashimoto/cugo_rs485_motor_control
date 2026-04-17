#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path
from typing import Dict

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from propo_control import RcChannelConfig, RcPwmReceiver


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="propo_signal_test.py",
        description="Simple RC PWM input tester (3ch: C/D/H)",
    )
    parser.add_argument("--interval", type=float, default=1.0, help="print interval [s]")
    parser.add_argument("--signal-timeout-ms", type=int, default=300, help="signal timeout [ms]")

    parser.add_argument("--ch-c-pin", type=int, default=24, help="BCM pin for CH-C")
    parser.add_argument("--ch-d-pin", type=int, default=4, help="BCM pin for CH-D")
    parser.add_argument("--ch-h-pin", type=int, default=14, help="BCM pin for CH-H")

    parser.add_argument("--ch-c-min", type=int, default=945)
    parser.add_argument("--ch-c-center", type=int, default=1484)
    parser.add_argument("--ch-c-max", type=int, default=2132)

    parser.add_argument("--ch-d-min", type=int, default=1114)
    parser.add_argument("--ch-d-center", type=int, default=1501)
    parser.add_argument("--ch-d-max", type=int, default=1870)

    parser.add_argument("--ch-h-min", type=int, default=997)
    parser.add_argument("--ch-h-center", type=int, default=1484)
    parser.add_argument("--ch-h-max", type=int, default=1994)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.interval <= 0.0:
        raise ValueError("--interval must be > 0")
    if args.signal_timeout_ms <= 0:
        raise ValueError("--signal-timeout-ms must be > 0")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        _validate_args(args)

        channels: Dict[str, RcChannelConfig] = {
            "C": RcChannelConfig(args.ch_c_pin, args.ch_c_min, args.ch_c_center, args.ch_c_max),
            "D": RcChannelConfig(args.ch_d_pin, args.ch_d_min, args.ch_d_center, args.ch_d_max),
            "H": RcChannelConfig(args.ch_h_pin, args.ch_h_min, args.ch_h_center, args.ch_h_max),
        }

        receiver = RcPwmReceiver(channels, signal_timeout_ms=args.signal_timeout_ms)

        running = True

        def _on_signal(_sig: int, _frame: object) -> None:
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)

        print("Start RC PWM signal test")
        print(f"Pins: CH-C={args.ch_c_pin} CH-D={args.ch_d_pin} CH-H={args.ch_h_pin}")
        print("Press Ctrl+C to stop")

        while running:
            frame = receiver.read()
            ts = time.strftime("%H:%M:%S", time.localtime())
            c_ok = frame.valid["C"]
            d_ok = frame.valid["D"]
            h_ok = frame.valid["H"]
            status = "SIGNAL_OK" if (c_ok and d_ok and h_ok) else "SIGNAL_NG"
            print(
                f"[{ts}] {status} "
                f"CH-C:{frame.pulse_us['C']}us({'OK' if c_ok else 'NG'}) "
                f"CH-D:{frame.pulse_us['D']}us({'OK' if d_ok else 'NG'}) "
                f"CH-H:{frame.pulse_us['H']}us({'OK' if h_ok else 'NG'})"
            )
            time.sleep(args.interval)

        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        receiver_obj = locals().get("receiver")
        if receiver_obj is not None:
            try:
                receiver_obj.close()
            except RuntimeError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
