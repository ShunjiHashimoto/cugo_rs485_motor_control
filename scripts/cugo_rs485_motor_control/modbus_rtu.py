from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable

import serial


class ModbusError(RuntimeError):
    """Base error for Modbus RTU operations."""


class ModbusTimeoutError(ModbusError):
    """Raised when no complete response is received within timeout."""


class ModbusResponseError(ModbusError):
    """Raised when response frame is malformed or indicates an exception."""


@dataclass
class SerialConfig:
    port: str
    baudrate: int = 9600
    bytesize: int = 8
    parity: str = "E"
    stopbits: int = 1
    timeout: float = 0.2


def crc16_modbus(data: bytes) -> int:
    """Compute Modbus RTU CRC16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class ModbusRtuClient:
    """Minimal Modbus RTU client supporting 03h, 06h, 10h."""

    def __init__(self, config: SerialConfig):
        self._config = config
        self._ser: serial.Serial | None = None

    @classmethod
    def from_params(
        cls,
        port: str,
        baudrate: int = 9600,
        bytesize: int = 8,
        parity: str = "E",
        stopbits: int = 1,
        timeout: float = 0.2,
    ) -> "ModbusRtuClient":
        return cls(
            SerialConfig(
                port=port,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout,
            )
        )

    def connect(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self._config.port,
            baudrate=self._config.baudrate,
            bytesize=self._config.bytesize,
            parity=self._config.parity,
            stopbits=self._config.stopbits,
            timeout=self._config.timeout,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "ModbusRtuClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def read_holding_registers(self, slave_id: int, address: int, count: int) -> list[int]:
        if not (1 <= count <= 16):
            raise ValueError("count must be 1..16")
        req = self._build_request(
            slave_id,
            0x03,
            bytes([(address >> 8) & 0xFF, address & 0xFF, (count >> 8) & 0xFF, count & 0xFF]),
        )
        res = self._exchange(req)
        if len(res) < 5:
            raise ModbusResponseError("short response")
        byte_count = res[2]
        if byte_count != count * 2:
            raise ModbusResponseError(
                f"invalid byte count: expected {count*2}, got {byte_count}"
            )
        payload = res[3 : 3 + byte_count]
        return [int.from_bytes(payload[i : i + 2], "big") for i in range(0, byte_count, 2)]

    def write_single_register(self, slave_id: int, address: int, value: int) -> None:
        req = self._build_request(
            slave_id,
            0x06,
            bytes(
                [
                    (address >> 8) & 0xFF,
                    address & 0xFF,
                    (value >> 8) & 0xFF,
                    value & 0xFF,
                ]
            ),
        )
        res = self._exchange(req)
        if res[1] != 0x06:
            raise ModbusResponseError("unexpected function code in write_single response")

    def write_multiple_registers(self, slave_id: int, start_address: int, values: Iterable[int]) -> None:
        vals = list(values)
        count = len(vals)
        if not (1 <= count <= 16):
            raise ValueError("number of registers must be 1..16")

        payload = bytearray(
            [
                (start_address >> 8) & 0xFF,
                start_address & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF,
                count * 2,
            ]
        )
        for v in vals:
            if not (0 <= v <= 0xFFFF):
                raise ValueError(f"register value out of range: {v}")
            payload.extend([(v >> 8) & 0xFF, v & 0xFF])

        req = self._build_request(slave_id, 0x10, bytes(payload))
        res = self._exchange(req)
        if res[1] != 0x10:
            raise ModbusResponseError("unexpected function code in write_multiple response")

    def _build_request(self, slave_id: int, func_code: int, payload: bytes) -> bytes:
        if not (0 <= slave_id <= 247):
            raise ValueError("slave_id must be 0..247")
        body = bytes([slave_id & 0xFF, func_code & 0xFF]) + payload
        crc = crc16_modbus(body)
        return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def _exchange(self, request: bytes) -> bytes:
        if not self._ser or not self._ser.is_open:
            raise ModbusError("serial port is not connected")

        self._ser.reset_input_buffer()
        self._ser.write(request)
        self._ser.flush()

        header = self._read_exact(2)
        slave_id, function = header[0], header[1]

        if function & 0x80:
            exception_and_crc = self._read_exact(3)
            frame = header + exception_and_crc
            self._validate_crc(frame)
            exception_code = exception_and_crc[0]
            raise ModbusResponseError(
                f"exception response: function=0x{function:02X}, code=0x{exception_code:02X}"
            )

        if function == 0x03:
            byte_count = self._read_exact(1)
            data_and_crc = self._read_exact(byte_count[0] + 2)
            frame = header + byte_count + data_and_crc
        elif function in (0x06, 0x08, 0x10):
            remainder = self._read_exact(6)
            frame = header + remainder
        else:
            raise ModbusResponseError(f"unsupported response function code: 0x{function:02X}")

        self._validate_crc(frame)
        return frame[:-2]

    def _read_exact(self, size: int) -> bytes:
        if not self._ser:
            raise ModbusError("serial port is not connected")

        deadline = time.monotonic() + float(self._config.timeout)
        data = bytearray()
        while len(data) < size:
            chunk = self._ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
                continue
            if time.monotonic() > deadline:
                raise ModbusTimeoutError(f"timeout while reading {size} bytes")
        return bytes(data)

    @staticmethod
    def _validate_crc(frame: bytes) -> None:
        if len(frame) < 4:
            raise ModbusResponseError("frame too short for CRC")
        expected = (frame[-1] << 8) | frame[-2]
        actual = crc16_modbus(frame[:-2])
        if actual != expected:
            raise ModbusResponseError(
                f"CRC mismatch: expected=0x{expected:04X}, actual=0x{actual:04X}"
            )
