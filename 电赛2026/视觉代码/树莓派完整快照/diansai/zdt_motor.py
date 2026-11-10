#!/usr/bin/env python3
"""ZDT X42S motor access for the current competition hardware.

The installed motor-side board accepts RS485 commands but does not return
bytes.  Control methods therefore use a fast, send-only path by default.
Read/query methods remain available for future replacement communication
boards and fail quickly instead of blocking the vision loop.

Hardware: /dev/ttyUSB0, 115200 8N1, address 1, fixed checksum 0x6B.
The tested PL2303 and FT232R adapters switch RS485 direction automatically;
RTS must not be manually toggled.
"""
from __future__ import annotations

import argparse
import threading
import time
from typing import Iterable, Optional

import serial

STEP_ANGLE = 1.8
MICROSTEPS = 16
PULSES_PER_REV = int(360.0 / STEP_ANGLE * MICROSTEPS)


def angle_to_pulses(angle: float) -> int:
    return int(round(angle / 360.0 * PULSES_PER_REV))


def pulses_to_angle(pulses: int) -> float:
    return pulses / PULSES_PER_REV * 360.0


class ZDTMotor:
    CKSUM = 0x6B
    DIR_CW = 0x01
    DIR_CCW = 0x00

    CMD_ENABLE = 0xF3
    CMD_JOG = 0xF6
    CMD_MOVE = 0xFD
    CMD_ESTOP = 0xFE
    CMD_SYNC_MOVE = 0xFF
    CMD_GET_VER = 0x1F
    CMD_GET_VBUS = 0x24
    CMD_GET_CURRENT = 0x27
    CMD_GET_ENC = 0x31
    CMD_GET_SPEED = 0x35
    CMD_GET_POS = 0x36
    CMD_GET_STATUS = 0x3A
    CMD_ZERO_ALL = 0x0A

    SUB_ENABLE = 0xAB
    SUB_ESTOP = 0x98
    SUB_SYNC_MOVE = 0x66
    SUB_ZERO_ALL = 0x6D

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        addr: int = 1,
        baud: int = 115200,
        query_timeout: float = 0.08,
        step_angle: float = 1.8,
        microsteps: int = 16,
        expect_command_response: bool = False,
    ) -> None:
        self.addr = addr
        self.query_timeout = query_timeout
        self.expect_command_response = expect_command_response
        self._sync = 0x00
        self._lock = threading.Lock()
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=query_timeout,
            write_timeout=0.2,
            rtscts=False,
            dsrdtr=False,
        )
        self.set_step_params(step_angle, microsteps)

    def set_step_params(self, step_angle: float, microsteps: int) -> None:
        global STEP_ANGLE, MICROSTEPS, PULSES_PER_REV
        STEP_ANGLE = step_angle
        MICROSTEPS = microsteps
        PULSES_PER_REV = int(round(360.0 / step_angle * microsteps))

    def _frame(self, body: Iterable[int]) -> bytes:
        return bytes(body) + bytes([self.CKSUM])

    def send_only(self, body: Iterable[int]) -> None:
        """Write one command frame without waiting for a response."""
        frame = self._frame(body)
        with self._lock:
            self.ser.write(frame)
            self.ser.flush()

    def _control(self, body: Iterable[int], ack_len: int = 4) -> bytes:
        if not self.expect_command_response:
            self.send_only(body)
            return b""
        return self._transact(body, ack_len)

    def _transact(self, body: Iterable[int], expected_len: int) -> bytes:
        frame = self._frame(body)
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()
            deadline = time.monotonic() + self.query_timeout
            data = bytearray()
            while time.monotonic() < deadline and len(data) < expected_len:
                chunk = self.ser.read(expected_len - len(data))
                if chunk:
                    data.extend(chunk)
                else:
                    break
        response = bytes(data)
        if len(response) != expected_len:
            return b""
        if response[0] != self.addr or response[-1] != self.CKSUM:
            return b""
        return response

    def enable(self) -> None:
        self._control([self.addr, self.CMD_ENABLE, self.SUB_ENABLE, 0x01, self._sync])

    def disable(self) -> None:
        self._control([self.addr, self.CMD_ENABLE, self.SUB_ENABLE, 0x00, self._sync])

    def stop(self) -> None:
        self._control([self.addr, self.CMD_ESTOP, self.SUB_ESTOP, self._sync])

    estop = stop

    def jog(self, direction: str | int, speed_rpm: int, accel: int = 50) -> None:
        direction_byte = self._direction(direction)
        speed = max(0, min(0xFFFF, int(speed_rpm)))
        accel = max(0, min(255, int(accel)))
        self._control([
            self.addr, self.CMD_JOG, direction_byte,
            (speed >> 8) & 0xFF, speed & 0xFF, accel, self._sync,
        ])

    def jog_cw(self, speed: int = 100, accel: int = 50) -> None:
        self.jog("cw", speed, accel)

    def jog_ccw(self, speed: int = 100, accel: int = 50) -> None:
        self.jog("ccw", speed, accel)

    def move(
        self,
        direction: str | int,
        pulses: int,
        speed_rpm: int = 100,
        accel: int = 50,
        absolute: bool = False,
    ) -> None:
        direction_byte = self._direction(direction)
        speed = max(0, min(0xFFFF, int(speed_rpm)))
        pulse_count = max(0, min(0xFFFFFFFF, abs(int(pulses))))
        self._control([
            self.addr, self.CMD_MOVE, direction_byte,
            (speed >> 8) & 0xFF, speed & 0xFF,
            max(0, min(255, int(accel))),
            (pulse_count >> 24) & 0xFF, (pulse_count >> 16) & 0xFF,
            (pulse_count >> 8) & 0xFF, pulse_count & 0xFF,
            0x01 if absolute else 0x00, self._sync,
        ])

    def move_by_angle(self, direction: str | int, angle: float, speed: int = 100, accel: int = 50) -> None:
        self.move(direction, angle_to_pulses(angle), speed, accel)

    def move_cw(self, pulses: int, speed: int = 100, accel: int = 50) -> None:
        self.move("cw", pulses, speed, accel)

    def move_ccw(self, pulses: int, speed: int = 100, accel: int = 50) -> None:
        self.move("ccw", pulses, speed, accel)

    def enable_sync(self) -> None:
        self._sync = 0x01

    def disable_sync(self) -> None:
        self._sync = 0x00

    def sync_move(self) -> None:
        self._control([self.addr, self.CMD_SYNC_MOVE, self.SUB_SYNC_MOVE])

    def set_zero(self) -> None:
        """Set the driver's current encoder/pulse/target positions to zero."""
        self._control([self.addr, self.CMD_ZERO_ALL, self.SUB_ZERO_ALL])

    def move_absolute_signed(
        self, target_pulses: int, speed: int = 60, accel: int = 80
    ) -> None:
        """Move to a signed absolute target in the driver's zeroed frame."""
        direction = "cw" if target_pulses >= 0 else "ccw"
        self.move(direction, abs(int(target_pulses)), speed, accel, absolute=True)

    def get_version(self) -> Optional[tuple[int, int]]:
        response = self._transact([self.addr, self.CMD_GET_VER], 5)
        return (response[2], response[3]) if response else None

    def get_encoder(self) -> Optional[int]:
        response = self._transact([self.addr, self.CMD_GET_ENC], 5)
        return int.from_bytes(response[2:4], "big") if response else None

    def get_status(self) -> dict[str, bool]:
        response = self._transact([self.addr, self.CMD_GET_STATUS], 4)
        if not response:
            return {}
        flags = response[2]
        return {
            "enabled": bool(flags & 0x01),
            "in_position": bool(flags & 0x02),
            "stalled": bool(flags & 0x04),
            "protect": bool(flags & 0x08),
        }

    def get_speed(self) -> Optional[int]:
        response = self._transact([self.addr, self.CMD_GET_SPEED], 6)
        if not response:
            return None
        return (-1 if response[2] else 1) * int.from_bytes(response[3:5], "big")

    def get_position(self) -> Optional[float]:
        response = self._transact([self.addr, self.CMD_GET_POS], 8)
        if not response:
            return None
        sign = -1 if response[2] else 1
        return sign * int.from_bytes(response[3:7], "big") * 360.0 / 65536.0

    def get_voltage(self) -> Optional[float]:
        response = self._transact([self.addr, self.CMD_GET_VBUS], 5)
        return int.from_bytes(response[2:4], "big") / 1000.0 if response else None

    def get_current(self) -> Optional[float]:
        response = self._transact([self.addr, self.CMD_GET_CURRENT], 5)
        return int.from_bytes(response[2:4], "big") / 1000.0 if response else None

    def is_enabled(self) -> bool:
        return self.get_status().get("enabled", False)

    def is_in_position(self) -> bool:
        return self.get_status().get("in_position", False)

    @staticmethod
    def _direction(direction: str | int) -> int:
        if isinstance(direction, str):
            return ZDTMotor.DIR_CW if direction.lower() == "cw" else ZDTMotor.DIR_CCW
        return ZDTMotor.DIR_CW if direction else ZDTMotor.DIR_CCW

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "ZDTMotor":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe ZDT serial diagnostic")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--query", action="store_true", help="query version/status/encoder")
    args = parser.parse_args()
    print(f"port={args.port}, pulses/rev={PULSES_PER_REV}, 90deg={angle_to_pulses(90)}")
    with ZDTMotor(port=args.port) as motor:
        if args.query:
            print("version:", motor.get_version())
            print("status:", motor.get_status())
            print("encoder:", motor.get_encoder())
        else:
            print("Serial port opened. No motor command was sent.")


if __name__ == "__main__":
    main()
