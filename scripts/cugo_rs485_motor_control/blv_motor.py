from __future__ import annotations

import time

from .modbus_rtu import ModbusRtuClient


class BlvMotorController:
    """High-level control helper for BLV/BLVD via Modbus RTU."""

    DRIVER_INPUT_COMMAND_LOWER = 0x007D
    COMMAND_SPEED_UPPER = 0x00C8
    FEEDBACK_SPEED_UPPER = 0x00CE
    ROTATION_SPEED_NO0_UPPER = 0x0480

    BIT_M0 = 1 << 0
    BIT_M1 = 1 << 1
    BIT_M2 = 1 << 2
    BIT_FWD = 1 << 3
    BIT_REV = 1 << 4
    BIT_STOP_MODE = 1 << 5  # 1: deceleration stop, 0: instantaneous stop

    def __init__(self, client: ModbusRtuClient, slave_id: int = 1):
        self.client = client
        self.slave_id = slave_id

    def set_rotation_speed(self, op_no: int, rpm: int) -> None:
        self._validate_op_no(op_no)
        if rpm != 0 and not (80 <= rpm <= 4000):
            raise ValueError("rpm must be 0 or 80..4000")

        upper_addr = self.ROTATION_SPEED_NO0_UPPER + op_no * 2
        self.client.write_multiple_registers(self.slave_id, upper_addr, [0x0000, rpm & 0xFFFF])

    def read_rotation_speed_setting(self, op_no: int) -> int:
        self._validate_op_no(op_no)
        upper_addr = self.ROTATION_SPEED_NO0_UPPER + op_no * 2
        return self._read_uint32(upper_addr)

    def read_command_speed_rpm(self) -> int:
        """Read monitor command speed (00C8h/00C9h) in r/min."""
        return self._read_int32(self.COMMAND_SPEED_UPPER)

    def read_feedback_speed_rpm(self) -> int:
        """Read monitor feedback speed (00CEh/00CFh) in r/min."""
        return self._read_int32(self.FEEDBACK_SPEED_UPPER)

    def run_forward(self, op_no: int = 2, deceleration_stop: bool = True) -> None:
        cmd = self._op_no_to_cmd_bits(op_no) | self.BIT_FWD
        if deceleration_stop:
            cmd |= self.BIT_STOP_MODE
        self.set_driver_input_command(cmd)

    def run_reverse(self, op_no: int = 2, deceleration_stop: bool = True) -> None:
        cmd = self._op_no_to_cmd_bits(op_no) | self.BIT_REV
        if deceleration_stop:
            cmd |= self.BIT_STOP_MODE
        self.set_driver_input_command(cmd)

    def stop(self, op_no: int = 2, deceleration_stop: bool = True) -> None:
        cmd = self._op_no_to_cmd_bits(op_no)
        if deceleration_stop:
            cmd |= self.BIT_STOP_MODE
        self.set_driver_input_command(cmd)

    def set_driver_input_command(self, lower_word: int) -> None:
        if not (0 <= lower_word <= 0xFFFF):
            raise ValueError("driver input command must be 0..65535")
        self.client.write_single_register(
            self.slave_id,
            self.DRIVER_INPUT_COMMAND_LOWER,
            lower_word,
        )

    def demo(self, rpm: int, seconds: float, op_no: int = 2, reverse: bool = False) -> None:
        self.set_rotation_speed(op_no, rpm)
        if reverse:
            self.run_reverse(op_no)
        else:
            self.run_forward(op_no)
        try:
            time.sleep(seconds)
        finally:
            self.stop(op_no)

    @staticmethod
    def _validate_op_no(op_no: int) -> None:
        if not (0 <= op_no <= 7):
            raise ValueError("op_no must be 0..7")

    @classmethod
    def _op_no_to_cmd_bits(cls, op_no: int) -> int:
        cls._validate_op_no(op_no)
        bits = 0
        if op_no & 0b001:
            bits |= cls.BIT_M0
        if op_no & 0b010:
            bits |= cls.BIT_M1
        if op_no & 0b100:
            bits |= cls.BIT_M2
        return bits

    def _read_uint32(self, upper_addr: int) -> int:
        values = self.client.read_holding_registers(self.slave_id, upper_addr, 2)
        return ((values[0] & 0xFFFF) << 16) | (values[1] & 0xFFFF)

    def _read_int32(self, upper_addr: int) -> int:
        value = self._read_uint32(upper_addr)
        if value & 0x80000000:
            value -= 0x100000000
        return value
