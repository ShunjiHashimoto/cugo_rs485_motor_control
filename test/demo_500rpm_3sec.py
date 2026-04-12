#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from cugo_rs485_motor_control.blv_motor import BlvMotorController
from cugo_rs485_motor_control.modbus_rtu import ModbusError, ModbusRtuClient

DEFAULT_OP_NO = 2
DEMO_RPM = 500
DEMO_SECONDS = 3.0
FIXED_PARITY = "E"
FIXED_STOPBITS = 1
FIXED_TIMEOUT = 0.3


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="demo_500rpm_3sec.py",
        description="Run BLV motor at 500 rpm for 3 seconds and stop.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port path")
    parser.add_argument("--slave", type=int, default=1, help="Modbus slave ID")
    parser.add_argument("--baudrate", type=int, default=9600, help="serial baudrate")
    parser.add_argument("--dir", choices=["fwd", "rev"], default="fwd", help="demo direction")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        with ModbusRtuClient.from_params(
            port=args.port,
            baudrate=args.baudrate,
            parity=FIXED_PARITY,
            stopbits=FIXED_STOPBITS,
            timeout=FIXED_TIMEOUT,
        ) as client:
            motor = BlvMotorController(client, slave_id=args.slave)
            motor.set_rotation_speed(DEFAULT_OP_NO, DEMO_RPM)
            if args.dir == "rev":
                motor.run_reverse(DEFAULT_OP_NO)
            else:
                motor.run_forward(DEFAULT_OP_NO)

            print(
                f"Demo start: rpm={DEMO_RPM}, sec={DEMO_SECONDS}, "
                f"dir={args.dir}, op_no={DEFAULT_OP_NO}"
            )
            try:
                time.sleep(DEMO_SECONDS)
            finally:
                motor.stop(DEFAULT_OP_NO)
                print("Demo complete: motor stopped")
        return 0
    except (ModbusError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
