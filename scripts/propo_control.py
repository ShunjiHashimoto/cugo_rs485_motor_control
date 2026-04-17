#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

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

try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
    GPIO = None
    GPIO_IMPORT_ERROR = exc
else:
    GPIO_IMPORT_ERROR = None


DEFAULT_MAX_V_KMH = 1.89
DEFAULT_MAX_W_RADPS = 3.14/2

MIN_ACCEPTED_PULSE_US = 750
MAX_ACCEPTED_PULSE_US = 2250
DEFAULT_BUZZER_PIN = 2


@dataclass(frozen=True)
class RcChannelConfig:
    pin: int
    min_pulse_us: int
    center_pulse_us: int
    max_pulse_us: int


@dataclass
class RcFrame:
    pulse_us: Dict[str, int]
    valid: Dict[str, bool]


@dataclass
class VehicleCommand:
    axis_c: float = 0.0
    axis_d: float = 0.0
    output_scale: float = 1.0
    speed_level: int = 0
    signal_healthy: bool = False


@dataclass
class MotorBridgeConfig:
    op_no: int = DEFAULT_OP_NO
    wheel_radius_left: float = 0.03858
    wheel_radius_right: float = 0.03858
    tread: float = 0.376
    reduction_ratio: float = 20.0
    max_rpm: float = 2600.0
    min_rpm: float = 80.0
    anti_creep_start_rpm: float = 120.0
    left_motor_sign: int = DEFAULT_LEFT_MOTOR_SIGN
    right_motor_sign: int = DEFAULT_RIGHT_MOTOR_SIGN
    deceleration_stop: bool = True


class RcPwmReceiver:
    def __init__(self, channel_config: Dict[str, RcChannelConfig], signal_timeout_ms: int = 300):
        if GPIO is None:
            raise RuntimeError(
                "RPi.GPIO is required for propo_control.py. "
                "Install python3-rpi.gpio and run with sudo."
            ) from GPIO_IMPORT_ERROR

        self._channel_config = channel_config
        self._signal_timeout_ns = int(signal_timeout_ms * 1_000_000)
        self._lock = threading.Lock()
        self._pulse_us = {name: cfg.center_pulse_us for name, cfg in channel_config.items()}
        self._valid = {name: False for name in channel_config}
        self._last_update_ns = {name: 0 for name in channel_config}
        self._rise_timestamp_ns = {name: None for name in channel_config}
        self._pin_to_name = {cfg.pin: name for name, cfg in channel_config.items()}

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        for cfg in channel_config.values():
            GPIO.setup(cfg.pin, GPIO.IN)
            GPIO.add_event_detect(cfg.pin, GPIO.BOTH, callback=self._on_edge)

    def _on_edge(self, pin: int) -> None:
        name = self._pin_to_name.get(pin)
        if name is None:
            return

        timestamp_ns = time.monotonic_ns()
        level = GPIO.input(pin)

        with self._lock:
            if level:
                self._rise_timestamp_ns[name] = timestamp_ns
                return

            rise_ns = self._rise_timestamp_ns[name]
            self._rise_timestamp_ns[name] = None
            if rise_ns is None:
                return

            width_us = int((timestamp_ns - rise_ns) / 1000)
            if MIN_ACCEPTED_PULSE_US <= width_us <= MAX_ACCEPTED_PULSE_US:
                self._pulse_us[name] = width_us
                self._valid[name] = True
                self._last_update_ns[name] = timestamp_ns
            else:
                self._valid[name] = False

    def read(self) -> RcFrame:
        now_ns = time.monotonic_ns()
        pulse_us: Dict[str, int] = {}
        valid: Dict[str, bool] = {}

        with self._lock:
            for name in self._channel_config:
                pulse_us[name] = self._pulse_us[name]
                last_ns = self._last_update_ns[name]
                fresh = last_ns != 0 and (now_ns - last_ns) <= self._signal_timeout_ns
                valid[name] = self._valid[name] and fresh

        return RcFrame(pulse_us=pulse_us, valid=valid)

    def close(self) -> None:
        pins = [cfg.pin for cfg in self._channel_config.values()]
        for pin in pins:
            try:
                GPIO.remove_event_detect(pin)
            except RuntimeError:
                pass
        GPIO.cleanup(pins)


class StartupBuzzer:
    def __init__(self, pin: int):
        if GPIO is None:
            raise RuntimeError(
                "RPi.GPIO is required for buzzer output. "
                "Install python3-rpi.gpio and run with sudo."
            ) from GPIO_IMPORT_ERROR

        self._pin = pin
        GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)

    def beep(self, count: int = 3, on_s: float = 0.08, off_s: float = 0.08) -> None:
        if count <= 0:
            return
        for _ in range(count):
            GPIO.output(self._pin, GPIO.HIGH)
            time.sleep(on_s)
            GPIO.output(self._pin, GPIO.LOW)
            time.sleep(off_s)

    def close(self) -> None:
        GPIO.output(self._pin, GPIO.LOW)
        GPIO.cleanup(self._pin)


class VehicleControl:
    def __init__(
        self,
        neutral_deadband_us: int,
        output_scale_steps: int,
        min_output_scale: float,
        throttle_expo: float,
        steering_expo: float,
    ):
        self._neutral_deadband_us = neutral_deadband_us
        self._output_scale_steps = max(1, output_scale_steps)
        self._min_output_scale = min_output_scale
        self._throttle_expo = throttle_expo
        self._steering_expo = steering_expo
        self._speed_level = 0
        self._output_scale = self._min_output_scale

    def update(self, frame: RcFrame, channel_config: Dict[str, RcChannelConfig]) -> VehicleCommand:
        cmd = VehicleCommand()
        cmd.signal_healthy = frame.valid["C"] and frame.valid["D"] and frame.valid["H"]
        if not cmd.signal_healthy:
            cmd.speed_level = self._speed_level
            cmd.output_scale = self._output_scale
            return cmd

        cmd.axis_c = self._normalize_axis(
            frame.pulse_us["C"],
            channel_config["C"],
            self._neutral_deadband_us,
            self._throttle_expo,
        )
        cmd.axis_d = self._normalize_axis(
            frame.pulse_us["D"],
            channel_config["D"],
            self._neutral_deadband_us,
            self._steering_expo,
        )
        self._speed_level = self._scale_level(
            frame.pulse_us["H"],
            channel_config["H"],
            self._output_scale_steps,
        )
        self._output_scale = self._min_output_scale + (1.0 - self._min_output_scale) * (
            float(self._speed_level) / float(self._output_scale_steps)
        )
        cmd.speed_level = self._speed_level
        cmd.output_scale = self._output_scale
        return cmd

    @staticmethod
    def _normalize_axis(pulse_us: int, config: RcChannelConfig, deadband_us: int, expo: float) -> float:
        centered = pulse_us - config.center_pulse_us
        if abs(centered) <= deadband_us:
            return 0.0

        if centered > 0:
            positive_span = max(1, config.max_pulse_us - config.center_pulse_us - deadband_us)
            normalized = float(centered - deadband_us) / float(positive_span)
        else:
            negative_span = max(1, config.center_pulse_us - config.min_pulse_us - deadband_us)
            normalized = float(centered + deadband_us) / float(negative_span)

        normalized = max(-1.0, min(1.0, normalized))
        sign = -1.0 if normalized < 0.0 else 1.0
        return sign * (abs(normalized) ** expo)

    @staticmethod
    def _scale_level(pulse_us: int, config: RcChannelConfig, output_scale_steps: int) -> int:
        span = max(1, config.max_pulse_us - config.min_pulse_us)
        normalized = float(pulse_us - config.min_pulse_us) / float(span)
        normalized = max(0.0, min(1.0, normalized))
        level = int(round(normalized * output_scale_steps))
        return max(0, min(output_scale_steps, level))


class Rs485DualMotorBridge:
    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float,
        left_slave: int,
        right_slave: int,
        config: MotorBridgeConfig,
        dry_run: bool,
    ):
        if config.wheel_radius_left <= 0.0 or config.wheel_radius_right <= 0.0:
            raise ValueError("wheel radius must be > 0")
        if config.tread <= 0.0:
            raise ValueError("tread must be > 0")
        if config.reduction_ratio <= 0.0:
            raise ValueError("reduction_ratio must be > 0")
        if config.max_rpm <= 0.0:
            raise ValueError("max_rpm must be > 0")
        if config.min_rpm < 0.0:
            raise ValueError("min_rpm must be >= 0")
        if config.anti_creep_start_rpm < config.min_rpm:
            raise ValueError("anti_creep_start_rpm must be >= min_rpm")
        if config.anti_creep_start_rpm > config.max_rpm:
            raise ValueError("anti_creep_start_rpm must be <= max_rpm")
        if config.left_motor_sign not in (-1, 1) or config.right_motor_sign not in (-1, 1):
            raise ValueError("motor sign must be -1 or 1")

        self._config = config
        self._dry_run = dry_run
        self._left_state: Optional[Tuple[int, str]] = None
        self._right_state: Optional[Tuple[int, str]] = None

        self._client: Optional[ModbusRtuClient] = None
        self._left_motor: Optional[BlvMotorController] = None
        self._right_motor: Optional[BlvMotorController] = None

        if not dry_run:
            self._client = ModbusRtuClient.from_params(
                port=port,
                baudrate=baudrate,
                parity=FIXED_PARITY,
                stopbits=FIXED_STOPBITS,
                timeout=timeout,
            )
            self._client.connect()
            self._left_motor = BlvMotorController(self._client, slave_id=left_slave)
            self._right_motor = BlvMotorController(self._client, slave_id=right_slave)

        self.stop(force=True)

    def close(self) -> None:
        try:
            self.stop(force=True)
        finally:
            if self._client is not None:
                self._client.close()

    def apply_body_velocity(self, v_mps: float, w_radps: float) -> Tuple[float, float, float, float]:
        limited_v, limited_w, left_motor_rpm, right_motor_rpm = self._limit_body_velocity(v_mps, w_radps)
        cmd_left = self._apply_motor_state(left_motor_rpm, is_left=True)
        cmd_right = self._apply_motor_state(right_motor_rpm, is_left=False)
        return limited_v, limited_w, cmd_left, cmd_right

    def stop(self, force: bool = False) -> None:
        self._set_motor_command(is_left=True, rpm_cmd=0, direction="stop", force=force)
        self._set_motor_command(is_left=False, rpm_cmd=0, direction="stop", force=force)

    def _limit_body_velocity(self, v_mps: float, w_radps: float) -> Tuple[float, float, float, float]:
        left_motor_rpm, right_motor_rpm = self._velocity_to_motor_rpm(v_mps, w_radps)
        max_abs_rpm = max(abs(left_motor_rpm), abs(right_motor_rpm))
        if max_abs_rpm <= self._config.max_rpm or max_abs_rpm == 0.0:
            return v_mps, w_radps, left_motor_rpm, right_motor_rpm

        scale = self._config.max_rpm / max_abs_rpm
        left_linear = self._motor_rpm_to_linear_velocity(
            left_motor_rpm * scale,
            self._config.wheel_radius_left,
        )
        right_linear = self._motor_rpm_to_linear_velocity(
            right_motor_rpm * scale,
            self._config.wheel_radius_right,
        )
        limited_v = 0.5 * (left_linear + right_linear)
        limited_w = (right_linear - left_linear) / self._config.tread
        return limited_v, limited_w, left_motor_rpm * scale, right_motor_rpm * scale

    def _velocity_to_motor_rpm(self, v_mps: float, w_radps: float) -> Tuple[float, float]:
        left_linear = v_mps - (w_radps * self._config.tread * 0.5)
        right_linear = v_mps + (w_radps * self._config.tread * 0.5)
        left_motor_rpm = self._linear_to_motor_rpm(left_linear, self._config.wheel_radius_left)
        right_motor_rpm = self._linear_to_motor_rpm(right_linear, self._config.wheel_radius_right)
        return left_motor_rpm, right_motor_rpm

    def _linear_to_motor_rpm(self, linear_mps: float, wheel_radius_m: float) -> float:
        wheel_rpm = linear_mps / (2.0 * math.pi * wheel_radius_m) * 60.0
        return wheel_rpm * self._config.reduction_ratio

    def _motor_rpm_to_linear_velocity(self, motor_rpm: float, wheel_radius_m: float) -> float:
        wheel_rpm = motor_rpm / self._config.reduction_ratio
        return wheel_rpm * (2.0 * math.pi * wheel_radius_m) / 60.0

    def _apply_motor_state(self, wheel_motor_rpm: float, *, is_left: bool) -> float:
        sign = self._config.left_motor_sign if is_left else self._config.right_motor_sign
        signed_motor_rpm = wheel_motor_rpm * float(sign)
        abs_rpm = min(abs(signed_motor_rpm), self._config.max_rpm)
        last_state = self._left_state if is_left else self._right_state
        is_stopped = last_state is None or last_state[0] == 0 or last_state[1] == "stop"

        # Prevent tiny neutral-offset noise from restarting motors while stopped.
        if is_stopped and abs_rpm < self._config.anti_creep_start_rpm:
            self._set_motor_command(is_left=is_left, rpm_cmd=0, direction="stop")
            return 0.0

        if abs_rpm < self._config.min_rpm:
            self._set_motor_command(is_left=is_left, rpm_cmd=0, direction="stop")
            return 0.0

        rpm_cmd = int(round(abs_rpm))
        rpm_cmd = max(int(self._config.min_rpm), min(int(self._config.max_rpm), rpm_cmd))
        direction = "fwd" if signed_motor_rpm >= 0.0 else "rev"
        self._set_motor_command(is_left=is_left, rpm_cmd=rpm_cmd, direction=direction)
        return float(rpm_cmd if direction == "fwd" else -rpm_cmd)

    def _set_motor_command(self, is_left: bool, rpm_cmd: int, direction: str, force: bool = False) -> None:
        state = (rpm_cmd, direction)
        last_state = self._left_state if is_left else self._right_state
        if not force and last_state == state:
            return

        if not self._dry_run:
            motor = self._left_motor if is_left else self._right_motor
            if motor is None:
                raise RuntimeError("motor controller is not initialized")
            if rpm_cmd == 0 or direction == "stop":
                motor.stop(self._config.op_no, deceleration_stop=self._config.deceleration_stop)
            else:
                motor.set_rotation_speed(self._config.op_no, rpm_cmd)
                if direction == "fwd":
                    motor.run_forward(self._config.op_no, deceleration_stop=self._config.deceleration_stop)
                elif direction == "rev":
                    motor.run_reverse(self._config.op_no, deceleration_stop=self._config.deceleration_stop)
                else:
                    raise ValueError(f"unknown direction: {direction}")

        if is_left:
            self._left_state = state
        else:
            self._right_state = state


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="propo_control.py",
        description="RC propo PWM -> RS485 BLV dual motor control",
    )

    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT, help="serial port path")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_SERIAL_BAUDRATE, help="serial baudrate")
    parser.add_argument("--timeout", type=float, default=DEFAULT_MODBUS_TIMEOUT_SEC, help="modbus timeout [s]")
    parser.add_argument("--left-slave", type=int, default=DEFAULT_LEFT_SLAVE_ID, help="left motor slave ID")
    parser.add_argument("--right-slave", type=int, default=DEFAULT_RIGHT_SLAVE_ID, help="right motor slave ID")
    parser.add_argument(
        "--left-motor-sign",
        type=int,
        choices=(-1, 1),
        default=DEFAULT_LEFT_MOTOR_SIGN,
        help="left motor sign",
    )
    parser.add_argument(
        "--right-motor-sign",
        type=int,
        choices=(-1, 1),
        default=DEFAULT_RIGHT_MOTOR_SIGN,
        help="right motor sign",
    )

    parser.add_argument("--v-axis", choices=("C", "D"), default="C", help="axis for linear velocity")
    parser.add_argument("--w-axis", choices=("C", "D"), default="D", help="axis for angular velocity")
    parser.add_argument("--swap-axes", action="store_true", help="shortcut for --v-axis D --w-axis C")
    parser.add_argument("--invert-v", action="store_true", help="reverse linear velocity sign")
    parser.add_argument(
        "--invert-w",
        action="store_true",
        default=True,
        help="reverse angular velocity sign (default: enabled)",
    )
    parser.add_argument(
        "--no-invert-w",
        dest="invert_w",
        action="store_false",
        help="disable angular velocity sign inversion",
    )

    parser.add_argument(
        "--max-v-kmh",
        type=float,
        default=DEFAULT_MAX_V_KMH,
        help=f"maximum |v| in km/h (default: {DEFAULT_MAX_V_KMH})",
    )
    parser.add_argument(
        "--max-w-radps",
        type=float,
        default=DEFAULT_MAX_W_RADPS,
        help=f"maximum |w| in rad/s (default: {DEFAULT_MAX_W_RADPS})",
    )
    parser.add_argument(
        "--h-reduction",
        type=float,
        default=0.5,
        help="output scale at minimum CH-H (0.0 < value <= 0.5)",
    )
    parser.add_argument("--neutral-deadband-us", type=int, default=40, help="neutral deadband in usec")
    parser.add_argument("--throttle-expo", type=float, default=1.2, help="expo for CH-C")
    parser.add_argument("--steering-expo", type=float, default=1.0, help="expo for CH-D")
    parser.add_argument("--output-scale-steps", type=int, default=5, help="CH-H output scale steps")

    parser.add_argument("--signal-timeout-ms", type=int, default=100, help="signal timeout [ms]")
    parser.add_argument("--loop-interval", type=float, default=0.05, help="control loop interval [s]")

    parser.add_argument("--ch-c-pin", type=int, default=24, help="BCM pin for CH-C")
    parser.add_argument("--ch-d-pin", type=int, default=4, help="BCM pin for CH-D")
    parser.add_argument("--ch-h-pin", type=int, default=14, help="BCM pin for CH-H")
    parser.add_argument("--buzzer-pin", type=int, default=DEFAULT_BUZZER_PIN, help="BCM pin for buzzer output")

    parser.add_argument("--ch-c-min", type=int, default=945, help="CH-C min pulse [us]")
    parser.add_argument("--ch-c-center", type=int, default=1484, help="CH-C center pulse [us]")
    parser.add_argument("--ch-c-max", type=int, default=2132, help="CH-C max pulse [us]")

    parser.add_argument("--ch-d-min", type=int, default=1114, help="CH-D min pulse [us]")
    parser.add_argument("--ch-d-center", type=int, default=1501, help="CH-D center pulse [us]")
    parser.add_argument("--ch-d-max", type=int, default=1870, help="CH-D max pulse [us]")

    parser.add_argument("--ch-h-min", type=int, default=997, help="CH-H min pulse [us]")
    parser.add_argument("--ch-h-center", type=int, default=1484, help="CH-H center pulse [us]")
    parser.add_argument("--ch-h-max", type=int, default=1994, help="CH-H max pulse [us]")

    parser.add_argument("--wheel-radius-left", type=float, default=0.03858, help="left wheel radius [m]")
    parser.add_argument("--wheel-radius-right", type=float, default=0.03858, help="right wheel radius [m]")
    parser.add_argument("--tread", type=float, default=0.376, help="tread width [m]")
    parser.add_argument("--reduction-ratio", type=float, default=20.0, help="gear reduction ratio")
    parser.add_argument("--max-rpm", type=float, default=2600.0, help="max motor rpm")
    parser.add_argument("--min-rpm", type=float, default=80.0, help="min effective motor rpm")
    parser.add_argument(
        "--anti-creep-start-rpm",
        type=float,
        default=120.0,
        help="minimum rpm required to start from stop (anti-creep threshold)",
    )
    parser.add_argument(
        "--instant-stop-mode",
        action="store_true",
        help="set STOP-MODE=0 (instantaneous stop) instead of deceleration stop",
    )

    parser.add_argument("--dry-run", action="store_true", help="do not send RS485 commands")
    parser.add_argument("--verbose", action="store_true", help="print each control cycle")
    parser.add_argument("--quiet", action="store_true", help="suppress startup messages")

    return parser


def _axis_value(cmd: VehicleCommand, axis: str) -> float:
    if axis == "C":
        return cmd.axis_c
    if axis == "D":
        return cmd.axis_d
    raise ValueError(f"unknown axis: {axis}")


def _validate_args(args: argparse.Namespace) -> None:
    if args.max_v_kmh <= 0.0:
        raise ValueError("--max-v-kmh must be > 0")
    if args.max_w_radps <= 0.0:
        raise ValueError("--max-w-radps must be > 0")
    if not (0.0 < args.h_reduction <= 0.5):
        raise ValueError("--h-reduction must satisfy 0.0 < value <= 0.5")
    if args.loop_interval <= 0.0:
        raise ValueError("--loop-interval must be > 0")
    if args.signal_timeout_ms <= 0:
        raise ValueError("--signal-timeout-ms must be > 0")
    if args.output_scale_steps < 1:
        raise ValueError("--output-scale-steps must be >= 1")
    if args.anti_creep_start_rpm < args.min_rpm:
        raise ValueError("--anti-creep-start-rpm must be >= --min-rpm")
    if args.anti_creep_start_rpm > args.max_rpm:
        raise ValueError("--anti-creep-start-rpm must be <= --max-rpm")
    if args.quiet and args.verbose:
        raise ValueError("--quiet and --verbose cannot be used together")


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.swap_axes:
        args.v_axis = "D"
        args.w_axis = "C"

    try:
        _validate_args(args)

        max_v_mps = args.max_v_kmh / 3.6
        channel_config = {
            "C": RcChannelConfig(args.ch_c_pin, args.ch_c_min, args.ch_c_center, args.ch_c_max),
            "D": RcChannelConfig(args.ch_d_pin, args.ch_d_min, args.ch_d_center, args.ch_d_max),
            "H": RcChannelConfig(args.ch_h_pin, args.ch_h_min, args.ch_h_center, args.ch_h_max),
        }

        receiver = RcPwmReceiver(channel_config, signal_timeout_ms=args.signal_timeout_ms)
        buzzer = StartupBuzzer(pin=args.buzzer_pin)
        control = VehicleControl(
            neutral_deadband_us=args.neutral_deadband_us,
            output_scale_steps=args.output_scale_steps,
            min_output_scale=args.h_reduction,
            throttle_expo=args.throttle_expo,
            steering_expo=args.steering_expo,
        )

        bridge = Rs485DualMotorBridge(
            port=args.port,
            baudrate=args.baudrate,
            timeout=args.timeout,
            left_slave=args.left_slave,
            right_slave=args.right_slave,
            config=MotorBridgeConfig(
                op_no=DEFAULT_OP_NO,
                wheel_radius_left=args.wheel_radius_left,
                wheel_radius_right=args.wheel_radius_right,
                tread=args.tread,
                reduction_ratio=args.reduction_ratio,
                max_rpm=args.max_rpm,
                min_rpm=args.min_rpm,
                anti_creep_start_rpm=args.anti_creep_start_rpm,
                left_motor_sign=args.left_motor_sign,
                right_motor_sign=args.right_motor_sign,
                deceleration_stop=not args.instant_stop_mode,
            ),
            dry_run=args.dry_run,
        )

        running = True

        def _on_signal(_sig: int, _frame: object) -> None:
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)

        if not args.quiet:
            print(
                "Starting propo control: "
                f"v_axis={args.v_axis} invert_v={args.invert_v}, "
                f"w_axis={args.w_axis} invert_w={args.invert_w}, "
                f"max_v={max_v_mps:.3f} m/s ({args.max_v_kmh:.2f} km/h), "
                f"max_w={args.max_w_radps:.3f} rad/s, "
                f"h_reduction={args.h_reduction}, "
                f"dry_run={args.dry_run}"
            )
            print(
                "RC input BCM pins: "
                f"CH-C={args.ch_c_pin} CH-D={args.ch_d_pin} CH-H={args.ch_h_pin}. "
                "Press Ctrl+C to stop."
            )
        # Startup beep pattern follows tang2dne_handler/scripts/indicator.py defaults.
        buzzer.beep()

        last_signal_healthy = True
        last_invalid_log = 0.0

        while running:
            frame = receiver.read()
            cmd = control.update(frame, channel_config)

            if not cmd.signal_healthy:
                bridge.stop()
                now = time.monotonic()
                should_log = (
                    last_signal_healthy
                    or last_invalid_log == 0.0
                    or (now - last_invalid_log) >= 5.0
                )
                if should_log:
                    print(
                        "[WARN] RC signal invalid "
                        f"ch-C={frame.pulse_us['C']} "
                        f"ch-D={frame.pulse_us['D']} "
                        f"ch-H={frame.pulse_us['H']}"
                    )
                    last_invalid_log = now
                last_signal_healthy = False
            else:
                if not last_signal_healthy:
                    print("RC signal recovered")
                last_signal_healthy = True

                v_sign = -1.0 if args.invert_v else 1.0
                w_sign = -1.0 if args.invert_w else 1.0
                v_target = v_sign * _axis_value(cmd, args.v_axis) * max_v_mps * cmd.output_scale
                w_target = w_sign * _axis_value(cmd, args.w_axis) * args.max_w_radps * cmd.output_scale

                limited_v, limited_w, left_cmd_rpm, right_cmd_rpm = bridge.apply_body_velocity(v_target, w_target)

                if args.verbose:
                    print(
                        f"v={limited_v:.3f} m/s w={limited_w:.3f} rad/s "
                        f"axisC={cmd.axis_c:.3f} axisD={cmd.axis_d:.3f} "
                        f"scale={cmd.output_scale:.3f} level={cmd.speed_level} "
                        f"rpmL={left_cmd_rpm:.0f} rpmR={right_cmd_rpm:.0f} "
                        f"pulse=({frame.pulse_us['C']},{frame.pulse_us['D']},{frame.pulse_us['H']})"
                    )

            time.sleep(args.loop_interval)

        return 0
    except (ModbusError, OSError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        receiver_obj = locals().get("receiver")
        if receiver_obj is not None:
            try:
                receiver_obj.close()
            except RuntimeError:
                pass
        bridge_obj = locals().get("bridge")
        if bridge_obj is not None:
            try:
                bridge_obj.close()
            except (ModbusError, OSError):
                pass
        buzzer_obj = locals().get("buzzer")
        if buzzer_obj is not None:
            try:
                buzzer_obj.close()
            except RuntimeError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
