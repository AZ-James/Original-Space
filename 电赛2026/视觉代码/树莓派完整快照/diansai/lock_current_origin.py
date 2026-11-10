#!/usr/bin/env python3
"""Visually verify the centre, define the current mechanism position as zero, and lock."""
import os
import time

import cv2

from ball_controller import CAMERA_ID, CAM_H, CAM_W, FixedBallDetector, PositionFilter
from pulse_controller import wait_for_settle
from vision_calibration import QUAD, TARGET_POINTS
from zdt_motor import ZDTMotor


CENTRE_X = float(TARGET_POINTS[0][0])
# QUAD已经包含可视化配置中的左右检测裁边。再留10像素余量，既保持
# 原安全意图，也能随装配后重新标定的轨道位置一起移动。
SAFE_MARGIN_X = 10.0
SAFE_LEFT_X = float(QUAD[:, 0].min()) + SAFE_MARGIN_X
SAFE_RIGHT_X = float(QUAD[:, 0].max()) - SAFE_MARGIN_X

os.system(
    f"v4l2-ctl -d /dev/video{CAMERA_ID} "
    f"--set-fmt-video=width={CAM_W},height={CAM_H},pixelformat=MJPG "
    f"--set-parm=60 2>/dev/null"
)
camera = cv2.VideoCapture(CAMERA_ID)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
camera.set(cv2.CAP_PROP_FPS, 60)
if not camera.isOpened():
    raise RuntimeError("camera could not be opened")

detector = FixedBallDetector()
filt = PositionFilter()
motor = None
try:
    x, settled, max_speed = wait_for_settle(camera, detector, filt, 15.0)
    if x is None or not settled:
        raise RuntimeError("ball was not detected as stable; motor remains unlocked")
    print(
        f"VISION_OK x={x:.1f} centre={CENTRE_X:.1f} "
        f"error={CENTRE_X-x:+.1f}px max_speed={max_speed:.1f}",
        flush=True,
    )
    # The operator has manually declared the current physical position to be
    # the new origin. The old calibrated centre may have shifted after the
    # assembly adjustment, so only enforce the independent track safety zone.
    if not SAFE_LEFT_X <= x <= SAFE_RIGHT_X:
        raise RuntimeError("ball is outside the safe track zone; motor remains unlocked")
    if abs(CENTRE_X - x) > 25.0:
        print("WARNING_OLD_CENTRE_CALIBRATION_NO_LONGER_MATCHES", flush=True)

    motor = ZDTMotor(port="/dev/ttyUSB0")
    # The motor is currently disabled, so define the manually adjusted physical
    # position as the driver's new relative zero before enabling it.
    motor.set_zero()
    time.sleep(0.3)
    motor.enable()
    time.sleep(0.6)
    motor.move_absolute_signed(0, 20, 80)
    time.sleep(0.4)
    print("MOTOR_LOCKED_AT_NEW_ZERO", flush=True)

    # Re-check after locking. The motor remains enabled even after serial close.
    x2, settled2, max_speed2 = wait_for_settle(camera, detector, filt, 10.0)
    print(
        f"LOCK_CHECK x={x2} settled={settled2} max_speed={max_speed2:.1f}",
        flush=True,
    )
    if x2 is None or not settled2:
        raise RuntimeError(
            "motor is locked, but the post-lock visual stability check failed"
        )
finally:
    if motor is not None:
        motor.close()
    camera.release()
    print("SERIAL_CLOSED_MOTOR_REMAINS_ENABLED", flush=True)
