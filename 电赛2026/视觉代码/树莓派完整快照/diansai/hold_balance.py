#!/usr/bin/env python3
"""Enable the motor and keep it locked at the relative zero/level position."""
import time

from zdt_motor import ZDTMotor


motor = ZDTMotor(port="/dev/ttyUSB0")
try:
    motor.enable()
    # The send-only driver may ignore a target sent immediately after ENABLE.
    time.sleep(0.6)
    motor.move_absolute_signed(0, 20, 80)
    time.sleep(0.5)
    print("MOTOR_ENABLED_AND_HOLDING_ZERO", flush=True)
finally:
    # Closing only releases the serial port. It does not send DISABLE, so the
    # driver remains enabled and keeps holding the zero target.
    motor.close()
