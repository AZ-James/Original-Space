#!/usr/bin/env python3
"""Adaptive event-driven control; no fixed angle-to-distance model."""
from __future__ import annotations

import argparse
import os
import time
from collections import deque
from pathlib import Path
from statistics import median
from typing import Optional

import cv2

from ball_controller import CAMERA_ID, CAM_H, CAM_W, FixedBallDetector, MAX_ABS_PULSES, PositionFilter
from vision_calibration import QUAD, TARGET_POINTS
from zdt_motor import ZDTMotor, angle_to_pulses


# These are conservative search/safety bounds, not precise plant parameters.
INITIAL_TILT_DEG = 5.5
MAX_TILT_DEG = 15.0
TILT_STEP_DEG = 0.5
TILT_APPROACH_SPEED_RPM = 1
TILT_APPROACH_ACCEL = 10
LEVEL_SPEED_RPM = 80
LEVEL_ACCEL = 200
MOTION_HOLD_AFTER_SETTLE_SECONDS = 1.0
PLATFORM_SETTLE_MARGIN_SECONDS = 0.20
MIN_RAW_TRIGGER_PX = 0.6
MAX_RAW_TRIGGER_PX = 2.0
RAW_NOISE_MULTIPLIER = 6.0
MIN_VELOCITY_TRIGGER_PX_S = 15.0
VELOCITY_NOISE_MULTIPLIER = 5.0
TARGET_TOLERANCE_PX = 15.0
MIN_PERMANENT_PROGRESS_PX = 2.0
SETTLE_SECONDS = 3.0
SETTLE_POSITION_SPAN_PX = 2.5
SETTLE_RECENT_SECONDS = 1.0
SETTLE_RECENT_SPEED_PX_S = 12.0
SETTLE_TIMEOUT_SECONDS = 12.0
OVERSPEED_PX_S = 100.0
RAMP_MOTION_SPEED_PX_S = 60.0
REBOUND_SPEED_PX_S = 60.0
EDGE_GUARD_PX = 25.0

# 控制进程独占摄像头；把低频缩略图原子写入共享文件，供桌面总面板显示，
# 避免总面板再次打开 /dev/video0 造成冲突。默认约8帧/秒。
LIVE_FRAME_PATH = Path(os.environ.get("DIANSAI_LIVE_FRAME", "/tmp/diansai_live.jpg"))
LIVE_FRAME_INTERVAL = 0.12
_last_live_frame_time = 0.0


def publish_live_frame(frame, measurement=None):
    global _last_live_frame_time
    now = time.monotonic()
    if now - _last_live_frame_time < LIVE_FRAME_INTERVAL:
        return
    _last_live_frame_time = now
    display = frame.copy()
    cv2.polylines(display, [QUAD], True, (0, 255, 0), 2)
    for index, point in enumerate(TARGET_POINTS):
        color = ((0, 255, 255), (255, 150, 0), (0, 140, 255))[index]
        cv2.circle(display, tuple(map(int, point)), 6, color, -1)
    if measurement is not None:
        cv2.drawMarker(
            display,
            (int(measurement.x), int(measurement.y)),
            (255, 0, 255),
            cv2.MARKER_CROSS,
            18,
            2,
        )
    display = cv2.resize(display, (480, 360), interpolation=cv2.INTER_AREA)
    temporary = LIVE_FRAME_PATH.with_name(
        LIVE_FRAME_PATH.stem + ".tmp" + LIVE_FRAME_PATH.suffix
    )
    try:
        cv2.imwrite(str(temporary), display, [cv2.IMWRITE_JPEG_QUALITY, 72])
        os.replace(temporary, LIVE_FRAME_PATH)
    except OSError:
        pass


def robust_mad(values: list[float]) -> float:
    if not values:
        return 0.0
    centre = median(values)
    return median([abs(value - centre) for value in values])


def read_measurement(camera, detector, filt):
    ok, frame = camera.read()
    now = time.perf_counter()
    if not ok or frame is None:
        return None
    measurement = detector.detect(frame)
    publish_live_frame(frame, measurement)
    filt.update(measurement, now)
    if measurement is None or filt.x is None or filt.lost_frames:
        return None
    return measurement, float(filt.x), float(filt.velocity)


def wait_for_settle(camera, detector, filt, timeout):
    deadline = time.monotonic() + timeout
    history: deque[tuple[float, float, float]] = deque()
    max_speed = 0.0
    all_min: Optional[float] = None
    all_max: Optional[float] = None
    while time.monotonic() < deadline:
        value = read_measurement(camera, detector, filt)
        if value is None:
            continue
        _, x, velocity = value
        now = time.monotonic()
        max_speed = max(max_speed, abs(velocity))
        all_min = x if all_min is None else min(all_min, x)
        all_max = x if all_max is None else max(all_max, x)
        history.append((now, x, velocity))
        while history and now - history[0][0] > SETTLE_SECONDS:
            history.popleft()
        if not history or now - history[0][0] < SETTLE_SECONDS * 0.95:
            continue
        span = max(point[1] for point in history) - min(point[1] for point in history)
        recent_speeds = [abs(p[2]) for p in history if now - p[0] <= SETTLE_RECENT_SECONDS]
        if (
            span <= SETTLE_POSITION_SPAN_PX
            and recent_speeds
            and median(recent_speeds) <= SETTLE_RECENT_SPEED_PX_S
        ):
            print(
                f"SETTLE_TRACE range=({all_min:.1f},{all_max:.1f}) max_speed={max_speed:.1f}px/s",
                flush=True,
            )
            return median([point[1] for point in history]), True, max_speed
    if all_min is not None:
        print(
            f"SETTLE_TIMEOUT range=({all_min:.1f},{all_max:.1f}) max_speed={max_speed:.1f}px/s",
            flush=True,
        )
    return filt.x, False, max_speed


def learn_current_noise(camera, detector, filt, seconds=0.5):
    deadline = time.monotonic() + seconds
    raw_positions: list[float] = []
    velocities: list[float] = []
    while time.monotonic() < deadline:
        value = read_measurement(camera, detector, filt)
        if value is None:
            continue
        measurement, _, velocity = value
        raw_positions.append(float(measurement.x))
        velocities.append(float(velocity))
    if len(raw_positions) < 10:
        return None
    baseline = median(raw_positions)
    raw_trigger = max(MIN_RAW_TRIGGER_PX, RAW_NOISE_MULTIPLIER * robust_mad(raw_positions))
    raw_trigger = min(MAX_RAW_TRIGGER_PX, raw_trigger)
    velocity_trigger = max(
        MIN_VELOCITY_TRIGGER_PX_S,
        VELOCITY_NOISE_MULTIPLIER * robust_mad(velocities),
    )
    return baseline, raw_trigger, velocity_trigger


def guarded_tilt(camera, detector, filt, motor, direction, angle_deg, active_left, active_right):
    noise = learn_current_noise(camera, detector, filt)
    if noise is None:
        return "vision_lost", 0.0, None
    baseline, raw_trigger, velocity_trigger = noise
    target_pulses = direction * abs(angle_to_pulses(angle_deg))
    if abs(target_pulses) > MAX_ABS_PULSES:
        raise RuntimeError("tilt exceeds hard motor limit")

    motor.move_absolute_signed(target_pulses, TILT_APPROACH_SPEED_RPM, TILT_APPROACH_ACCEL)
    started = time.monotonic()
    # At N rpm the shaft moves 6*N degrees/s. Add a modest margin for the
    # configured acceleration, then establish a new baseline at the tilted
    # position instead of using an exact camera-projection compensation.
    arm_after = started + angle_deg / (6.0 * TILT_APPROACH_SPEED_RPM) + PLATFORM_SETTLE_MARGIN_SECONDS
    deadline = arm_after + MOTION_HOLD_AFTER_SETTLE_SECONDS
    reason = "no_motion"
    max_speed = 0.0
    trigger_x: Optional[float] = None
    baseline_directed = direction * baseline
    minimum_directed = baseline_directed
    tilted_baseline: Optional[float] = None
    try:
        while time.monotonic() < deadline:
            value = read_measurement(camera, detector, filt)
            if value is None:
                continue
            measurement, x, velocity = value
            trigger_x = x
            max_speed = max(max_speed, abs(velocity))
            if not active_left <= x <= active_right:
                reason = "edge_guard"
                break
            if abs(velocity) >= OVERSPEED_PX_S:
                reason = "overspeed"
                break
            directed_raw = direction * float(measurement.x)
            minimum_directed = min(minimum_directed, directed_raw)
            if time.monotonic() < arm_after:
                if abs(velocity) >= RAMP_MOTION_SPEED_PX_S:
                    reason = "ramp_motion"
                    break
                continue
            if tilted_baseline is None:
                tilted_baseline = directed_raw
            tilted_progress = directed_raw - tilted_baseline
            directed_speed = direction * velocity
            # Once platform travel should be complete, filtered velocity is no
            # longer contaminated by the motor ramp and can corroborate a
            # small real displacement.
            if tilted_progress >= raw_trigger or (
                tilted_progress >= 0.2 and directed_speed >= velocity_trigger
            ):
                reason = "motion"
                break
    finally:
        # Independent fail-safe: every attempt levels immediately.
        motor.move_absolute_signed(0, LEVEL_SPEED_RPM, LEVEL_ACCEL)
    print(
        f"TRIGGER reason={reason} baseline={baseline:.2f} x={trigger_x} "
        f"raw_threshold={raw_trigger:.2f}px platform_shift="
        f"{baseline_directed-minimum_directed:.2f}px elapsed={time.monotonic()-started:.2f}s",
        flush=True,
    )
    return reason, max_speed, trigger_x


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=("centre", "left", "right"), default="centre")
    parser.add_argument("--max-pulses", type=int, default=8)
    parser.add_argument("--start-tilt", type=float, default=INITIAL_TILT_DEG)
    parser.add_argument("--tilt-step", type=float, default=TILT_STEP_DEG)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    if not 1 <= args.max_pulses <= 12:
        raise SystemExit("--max-pulses must be between 1 and 12")
    if not INITIAL_TILT_DEG <= args.start_tilt <= MAX_TILT_DEG:
        raise SystemExit(
            f"--start-tilt must be between {INITIAL_TILT_DEG} and {MAX_TILT_DEG} degrees"
        )
    if not 0.25 <= args.tilt_step <= 2.0:
        raise SystemExit("--tilt-step must be between 0.25 and 2.0 degrees")

    target_index = {"centre": 0, "left": 1, "right": 2}[args.target]
    target_x = float(TARGET_POINTS[target_index][0])
    active_left = float(QUAD[:, 0].min()) + EDGE_GUARD_PX
    active_right = float(QUAD[:, 0].max()) - EDGE_GUARD_PX
    if abs(angle_to_pulses(MAX_TILT_DEG)) > MAX_ABS_PULSES:
        raise RuntimeError("configured tilt exceeds the hard motor limit")

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
        raise RuntimeError("cannot open camera")

    detector = FixedBallDetector()
    filt = PositionFilter()
    motor: Optional[ZDTMotor] = None
    try:
        x, settled, _ = wait_for_settle(camera, detector, filt, SETTLE_TIMEOUT_SECONDS)
        if x is None or not settled:
            print("ABORT: ball not genuinely settled; motor was not opened")
            raise SystemExit(2)
        print(f"START target={args.target} target_x={target_x:.1f} x={x:.1f}", flush=True)
        if abs(target_x - x) <= TARGET_TOLERANCE_PX:
            print(f"SUCCESS without motion x={x:.1f}", flush=True)
            return

        motor = ZDTMotor(port=args.port)
        motor.enable()
        time.sleep(0.6)  # Send-only driver needs time after ENABLE.
        motor.move_absolute_signed(0, LEVEL_SPEED_RPM, LEVEL_ACCEL)
        time.sleep(0.3)

        tilt_angle = args.start_tilt
        previous_direction = 0
        for index in range(args.max_pulses):
            error = target_x - x
            if abs(error) <= TARGET_TOLERANCE_PX:
                print(f"SUCCESS x={x:.1f} error={error:+.1f}px attempts={index}", flush=True)
                return
            if not active_left <= x <= active_right:
                print(f"ABORT: x={x:.1f} is inside the excluded edge zone", flush=True)
                raise SystemExit(2)

            direction = 1 if error > 0 else -1
            if previous_direction and direction != previous_direction:
                tilt_angle = args.start_tilt
            previous_direction = direction
            before_x = x
            print(
                f"ATTEMPT {index + 1}/{args.max_pulses} x={x:.1f} "
                f"error={error:+.1f}px max_tilt={direction * tilt_angle:+.1f}deg",
                flush=True,
            )
            reason, trigger_speed, _ = guarded_tilt(
                camera, detector, filt, motor, direction, tilt_angle, active_left, active_right
            )
            if reason in ("vision_lost", "edge_guard", "overspeed"):
                print(f"ABORT: guarded tilt ended with {reason}", flush=True)
                raise SystemExit(2)

            x, settled, coast_speed = wait_for_settle(
                camera, detector, filt, SETTLE_TIMEOUT_SECONDS
            )
            if x is None or not settled:
                print("ABORT: ball did not become genuinely settled; motor is level", flush=True)
                raise SystemExit(2)
            max_speed = max(trigger_speed, coast_speed)
            progress = direction * (x - before_x)
            print(
                f"RESULT x={x:.1f} error={target_x-x:+.1f}px "
                f"progress={progress:+.1f}px max_speed={max_speed:.1f}px/s",
                flush=True,
            )
            if max_speed >= OVERSPEED_PX_S:
                print("ABORT: overspeed threshold reached", flush=True)
                raise SystemExit(2)
            if progress < MIN_PERMANENT_PROGRESS_PX:
                if max_speed > REBOUND_SPEED_PX_S:
                    print("ABORT: rebound/no progress; not increasing tilt", flush=True)
                    raise SystemExit(2)
                if tilt_angle >= MAX_TILT_DEG:
                    print("ABORT: friction remains at conservative tilt limit", flush=True)
                    raise SystemExit(2)
                tilt_angle = min(MAX_TILT_DEG, tilt_angle + args.tilt_step)
                print(f"ADAPT: next maximum tilt={tilt_angle:.1f}deg", flush=True)

        print(f"STOP: attempt budget exhausted at x={x:.1f}", flush=True)
        raise SystemExit(3)
    finally:
        if motor is not None:
            try:
                motor.move_absolute_signed(0, LEVEL_SPEED_RPM, LEVEL_ACCEL)
                time.sleep(0.4)
            finally:
                motor.close()
        camera.release()


if __name__ == "__main__":
    main()
