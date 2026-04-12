#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running this script directly.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cugo_rs485_motor_control.blv_motor import BlvMotorController
from cugo_rs485_motor_control.modbus_rtu import ModbusError, ModbusRtuClient

DEFAULT_OP_NO = 2
FIXED_PARITY = "E"
FIXED_STOPBITS = 1
FIXED_TIMEOUT = 0.3


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="Dual BLVD10KM control (left/right) over RS485 Modbus RTU",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port path")
    parser.add_argument("--baudrate", type=int, default=9600, help="serial baudrate")
    parser.add_argument("--left-slave", type=int, default=2, help="left motor slave ID")
    parser.add_argument("--right-slave", type=int, default=1, help="right motor slave ID")

    sub = parser.add_subparsers(dest="command", required=True)

    sp_set = sub.add_parser("set-speed", help="set speed for left/right motors")
    sp_set.add_argument("--left", type=int, required=True, help="left speed rpm (0 or 80..4000)")
    sp_set.add_argument("--right", type=int, required=True, help="right speed rpm (0 or 80..4000)")

    sp_run = sub.add_parser("run", help="run left/right motors")
    sp_run.add_argument("--left-dir", choices=["fwd", "rev", "stop"], required=True, help="left direction")
    sp_run.add_argument(
        "--right-dir", choices=["fwd", "rev", "stop"], required=True, help="right direction"
    )
    sp_run.add_argument(
        "--instant-stop-mode",
        action="store_true",
        help="set STOP-MODE=0 (instantaneous stop mode) for run command",
    )

    sp_stop = sub.add_parser("stop", help="stop left/right motors")
    sp_stop.add_argument(
        "--instant",
        action="store_true",
        help="instantaneous stop (default: deceleration stop)",
    )

    sub.add_parser("read-speed", help="read speed settings for left/right motors")

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
            left = BlvMotorController(client, slave_id=args.left_slave)
            right = BlvMotorController(client, slave_id=args.right_slave)
            run_command(left, right, args)
        return 0
    except (ModbusError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130


def run_command(left: BlvMotorController, right: BlvMotorController, args: argparse.Namespace) -> None:
    cmd = args.command

    if cmd == "set-speed":
        left.set_rotation_speed(DEFAULT_OP_NO, args.left)
        right.set_rotation_speed(DEFAULT_OP_NO, args.right)
        print(
            "OK: set speeds "
            f"(left slave={left.slave_id}: {args.left} rpm, "
            f"right slave={right.slave_id}: {args.right} rpm)"
        )
        return

    if cmd == "run":
        decel = not args.instant_stop_mode
        apply_direction(left, args.left_dir, decel)
        apply_direction(right, args.right_dir, decel)
        stop_mode = "deceleration" if decel else "instantaneous"
        print(
            "OK: run "
            f"(left={args.left_dir}, right={args.right_dir}, "
            f"op_no={DEFAULT_OP_NO}, stop_mode={stop_mode})"
        )
        return

    if cmd == "stop":
        decel = not args.instant
        left.stop(DEFAULT_OP_NO, deceleration_stop=decel)
        right.stop(DEFAULT_OP_NO, deceleration_stop=decel)
        how = "deceleration" if decel else "instantaneous"
        print(f"OK: stop both motors ({how})")
        return

    if cmd == "read-speed":
        left_speed = left.read_rotation_speed_setting(DEFAULT_OP_NO)
        right_speed = right.read_rotation_speed_setting(DEFAULT_OP_NO)
        print(f"left(speed No.{DEFAULT_OP_NO})={left_speed} rpm")
        print(f"right(speed No.{DEFAULT_OP_NO})={right_speed} rpm")
        return

    raise ValueError(f"unsupported command: {cmd}")


def apply_direction(motor: BlvMotorController, direction: str, deceleration_stop: bool) -> None:
    if direction == "fwd":
        motor.run_forward(DEFAULT_OP_NO, deceleration_stop=deceleration_stop)
    elif direction == "rev":
        motor.run_reverse(DEFAULT_OP_NO, deceleration_stop=deceleration_stop)
    elif direction == "stop":
        motor.stop(DEFAULT_OP_NO, deceleration_stop=deceleration_stop)
    else:
        raise ValueError(f"unknown direction: {direction}")


if __name__ == "__main__":
    raise SystemExit(main())
