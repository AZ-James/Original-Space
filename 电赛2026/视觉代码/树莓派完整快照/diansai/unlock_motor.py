#!/usr/bin/env python3
"""Disable/unlock the motor without sending a motion command."""
import time

from zdt_motor import ZDTMotor


motor = ZDTMotor(port="/dev/ttyUSB0")
try:
    motor.disable()
    time.sleep(0.5)
    print("MOTOR_DISABLED_UNLOCKED", flush=True)
finally:
    motor.close()
