#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow running this script directly.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from config import (
    DEFAULT_LEFT_MOTOR_SIGN,
    DEFAULT_LEFT_SLAVE_ID,
    DEFAULT_MODBUS_TIMEOUT_SEC,
    DEFAULT_OP_NO,
    DEFAULT_RIGHT_MOTOR_SIGN,
    DEFAULT_RIGHT_SLAVE_ID,
    DEFAULT_SERIAL_BAUDRATE,
    DEFAULT_SERIAL_PORT,
    FIXED_PARITY,
    FIXED_STOPBITS,
)
from cugo_rs485_motor_control.blv_motor import BlvMotorController
from cugo_rs485_motor_control.modbus_rtu import ModbusError, ModbusRtuClient


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="Dual BLVD10KM control (left/right) over RS485 Modbus RTU",
    )
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT, help="serial port path")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_SERIAL_BAUDRATE, help="serial baudrate")
    parser.add_argument("--left-slave", type=int, default=DEFAULT_LEFT_SLAVE_ID, help="left motor slave ID")
    parser.add_argument("--right-slave", type=int, default=DEFAULT_RIGHT_SLAVE_ID, help="right motor slave ID")
    parser.add_argument(
        "--left-dir-sign",
        type=int,
        choices=(-1, 1),
        default=DEFAULT_LEFT_MOTOR_SIGN,
        help="left motor direction sign for run command (-1: invert, 1: normal)",
    )
    parser.add_argument(
        "--right-dir-sign",
        type=int,
        choices=(-1, 1),
        default=DEFAULT_RIGHT_MOTOR_SIGN,
        help="right motor direction sign for run command (-1: invert, 1: normal)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="enable realtime speed monitor output",
    )
    parser.add_argument(
        "--debug-interval",
        type=float,
        default=0.5,
        help="realtime speed monitor interval seconds (used with --debug)",
    )

    sub = parser.add_subparsers(dest="command", required=False)
    parser.set_defaults(command="read-speed-setting")

    sp_set = sub.add_parser("set-speed", help="set speed for left/right motors")
    sp_set.add_argument("--left", type=int, required=True, help="left speed rpm (0 or 80..4000)")
    sp_set.add_argument("--right", type=int, required=True, help="right speed rpm (0 or 80..4000)")

    sp_run = sub.add_parser("run", help="run left/right motors")
    sp_run.add_argument("--left-dir", choices=["fwd", "rev", "stop"], required=True, help="left direction")
    sp_run.add_argument(
        "--right-dir", choices=["fwd", "rev", "stop"], required=True, help="right direction"
    )
    sp_run.add_argument(
        "--left-rpm",
        type=int,
        help="optionally set left speed rpm before run (0 or 80..4000)",
    )
    sp_run.add_argument(
        "--right-rpm",
        type=int,
        help="optionally set right speed rpm before run (0 or 80..4000)",
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

    sub.add_parser("read-speed", help="read speed info for left/right motors")
    sub.add_parser("read-speed-setting", help="read speed settings for left/right motors")

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        with ModbusRtuClient.from_params(
            port=args.port,
            baudrate=args.baudrate,
            parity=FIXED_PARITY,
            stopbits=FIXED_STOPBITS,
            timeout=DEFAULT_MODBUS_TIMEOUT_SEC,
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
        if args.left_rpm is not None:
            left.set_rotation_speed(DEFAULT_OP_NO, args.left_rpm)
        if args.right_rpm is not None:
            right.set_rotation_speed(DEFAULT_OP_NO, args.right_rpm)
        left_effective_dir = _map_direction_by_sign(args.left_dir, args.left_dir_sign)
        right_effective_dir = _map_direction_by_sign(args.right_dir, args.right_dir_sign)
        apply_direction(left, left_effective_dir, decel)
        apply_direction(right, right_effective_dir, decel)
        stop_mode = "deceleration" if decel else "instantaneous"
        speed_arg = ""
        if args.left_rpm is not None or args.right_rpm is not None:
            speed_arg = (
                f", left_rpm={args.left_rpm if args.left_rpm is not None else 'keep'}, "
                f"right_rpm={args.right_rpm if args.right_rpm is not None else 'keep'}"
            )
        print(
            "OK: run "
            f"(left={args.left_dir}, right={args.right_dir}, "
            f"effective_left={left_effective_dir}, effective_right={right_effective_dir}, "
            f"op_no={DEFAULT_OP_NO}, stop_mode={stop_mode}{speed_arg})"
        )
        if args.debug:
            monitor_speed_realtime(left, right, args.debug_interval)
        return

    if cmd == "stop":
        decel = not args.instant
        left.stop(DEFAULT_OP_NO, deceleration_stop=decel)
        right.stop(DEFAULT_OP_NO, deceleration_stop=decel)
        how = "deceleration" if decel else "instantaneous"
        print(f"OK: stop both motors ({how})")
        return

    if cmd == "read-speed":
        if args.debug:
            monitor_speed_realtime(left, right, args.debug_interval)
        else:
            print_speed_info(left, right)
        return

    if cmd == "read-speed-setting":
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


def _map_direction_by_sign(direction: str, sign: int) -> str:
    if direction == "stop" or sign == 1:
        return direction
    if direction == "fwd":
        return "rev"
    if direction == "rev":
        return "fwd"
    raise ValueError(f"unknown direction: {direction}")


def print_speed_info(left: BlvMotorController, right: BlvMotorController, prefix: str = "") -> None:
    left_setting = left.read_rotation_speed_setting(DEFAULT_OP_NO)
    right_setting = right.read_rotation_speed_setting(DEFAULT_OP_NO)
    left_command = left.read_command_speed_rpm()
    right_command = right.read_command_speed_rpm()
    left_feedback = left.read_feedback_speed_rpm()
    right_feedback = right.read_feedback_speed_rpm()
    print(
        f"{prefix}left(op_no={DEFAULT_OP_NO}): "
        f"setting={left_setting} rpm, command={left_command} rpm, feedback={left_feedback} rpm"
    )
    print(
        f"{prefix}right(op_no={DEFAULT_OP_NO}): "
        f"setting={right_setting} rpm, command={right_command} rpm, feedback={right_feedback} rpm"
    )


def monitor_speed_realtime(
    left: BlvMotorController,
    right: BlvMotorController,
    interval: float,
) -> None:
    if interval <= 0.0:
        raise ValueError("debug-interval must be > 0")
    print(f"DEBUG: realtime speed monitor started (interval={interval:.3f}s, Ctrl+C to stop)")
    while True:
        ts = time.strftime("%H:%M:%S", time.localtime())
        print_speed_info(left, right, prefix=f"[{ts}] ")
        time.sleep(interval)


if __name__ == "__main__":
    raise SystemExit(main())
