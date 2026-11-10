#!/usr/bin/env python3
"""Fixed-calibration visual ball controller with a hard +/-30 degree command limit.

This file is separate from detect_rect.py; the user's original visual test
program is not imported or modified.

Normal startup is deliberately interlocked:
1. The controller sends ENABLE, but does not move the motor.
2. The operator confirms that the mechanism is locked and holds the ball at
   the centre target point (355, 261).
3. After the ball is stable for two seconds, press Space to arm.
4. The driver position is zeroed at that physical centre/level position.
5. This installation uses the experimentally confirmed motor sign +1 by
   default. ``--motor-sign -1`` is retained for reversed wiring/mechanics.

Every motion command is an ABSOLUTE target within +/-30 degrees after zeroing.
This is safer than accumulating relative commands when serial responses are
unavailable. Only one process may control /dev/ttyUSB0 at a time.
"""
from __future__ import annotations

import argparse
import math
import os
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from zdt_motor import ZDTMotor, angle_to_pulses, pulses_to_angle
from vision_calibration import QUAD, TARGET_POINTS, TRACK_ENDPOINTS

# TARGET_POINTS order in vision_calibration.json: centre, left, right.
TARGET_NAMES = ("centre", "left", "right")

CAMERA_ID = 0
CAM_W = 640
CAM_H = 480

# Mechanical safety limits. Do not raise without a physical safety review.
MAX_ANGLE_DEG = 30.0
PULSES_PER_REV = abs(angle_to_pulses(360.0))
# floor, never round: 267 pulses would be 30.0375 degrees and violate the limit.
MAX_ABS_PULSES = max(1, math.floor(PULSES_PER_REV * MAX_ANGLE_DEG / 360.0))
MAX_CHANGE_DEG = 4.0          # allows timely reverse-tilt braking
MAX_CHANGE_PULSES = max(1, math.floor(PULSES_PER_REV * MAX_CHANGE_DEG / 360.0))
CONTROL_INTERVAL = 0.04
MOTOR_SPEED_RPM = 40
MOTOR_ACCEL = 80

# Normalized PD gains: output is tube tilt in degrees.
POSITION_NORMALIZER_PX = 256.0
KP_DEG = 12.0
KD_DEG = 28.0
MAX_CONTROL_ANGLE_DEG = 18.0
MAX_MOVING_TILT_DEG = 10.0
BREAKAWAY_ANGLE_DEG = 8.0
BREAKAWAY_MAX_ANGLE_DEG = 18.0
BREAKAWAY_STEP_SECONDS = 1.0
BREAKAWAY_MIN_ERROR_PX = 30.0
POSITION_DEADBAND_PX = 10.0
VELOCITY_DEADBAND_PX_S = 5.0
FAST_BRAKE_SPEED_PX_S = 55.0
MOVING_SPEED_THRESHOLD_PX_S = 8.0
MAX_CRUISE_SPEED_PX_S = 40.0
POSITION_TO_SPEED_GAIN = 0.5
VELOCITY_GAIN_DEG = 0.18
EDGE_LEFT_X = 105.0
EDGE_RIGHT_X = 535.0
EDGE_LOOKAHEAD_SECONDS = 0.25

# Startup and detection interlocks.
CENTER_TOLERANCE_PX = 10.0
LOCK_STABLE_SECONDS = 2.0
LOCK_MAX_SPEED_PX_S = 6.0
LOST_FRAME_LIMIT = 10
MAX_MEASUREMENT_JUMP_PX = 45.0
MAX_FILTER_VELOCITY_PX_S = 500.0
OVERSPEED_STOP_PX_S = 120.0
OVERSPEED_RESUME_PX_S = 10.0
SEARCH_STEP_SECONDS = 1.2
SEARCH_ANGLES_DEG = (0.0, 8.0, 12.0, 18.0, 24.0, -8.0, -12.0, -18.0, -24.0)
PROBE_ANGLE_DEG = 2.0
PROBE_SECONDS = 0.8
PROBE_MIN_MOVEMENT_PX = 3.0


@dataclass
class BallMeasurement:
    x: float
    y: float
    left: int
    right: int
    top: int
    bottom: int


class FixedBallDetector:
    """Reflection-sensitive 1-D detector derived from the original program."""

    def __init__(self, min_stain_fraction: float = 0.05) -> None:
        self.min_stain_fraction = min_stain_fraction
        self.last_x_warped: Optional[float] = None

    def detect(self, frame: np.ndarray) -> Optional[BallMeasurement]:
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = np.zeros((height, width), np.uint8)
        cv2.fillPoly(mask, [QUAD], 255)

        rect = cv2.minAreaRect(QUAD.astype(np.float32))
        (cx, cy), (rect_w, rect_h), angle = rect
        if rect_w < rect_h:
            rect_w, rect_h = rect_h, rect_w
            angle += 90

        matrix = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
        warped_gray = cv2.warpAffine(gray, matrix, (width, height))
        warped_mask = cv2.warpAffine(mask, matrix, (width, height))
        warped = cv2.bitwise_and(warped_gray, warped_gray, mask=warped_mask)

        corners = cv2.boxPoints(rect)
        warped_corners = cv2.transform(np.array([corners], np.float32), matrix)[0]
        x_min = max(0, int(np.min(warped_corners[:, 0])))
        x_max = min(width - 1, int(np.max(warped_corners[:, 0])))
        y_min = max(0, int(np.min(warped_corners[:, 1])))
        y_max = min(height - 1, int(np.max(warped_corners[:, 1])))
        if x_max - x_min < 30 or y_max - y_min < 5:
            return None

        roi = warped[y_min:y_max, x_min:x_max]
        roi_h, roi_w = roi.shape
        y1, y2 = roi_h // 3, 2 * roi_h // 3
        if y2 - y1 < 2:
            return None

        profile = roi[y1:y2, :].mean(axis=0)
        window = max(3, roi_w // 20)
        if roi_w <= window * 2 + 1:
            return None
        # Same 1-D local standard deviation, calculated as running moments.
        # Avoiding one Python/NumPy call per X coordinate reduces brake delay.
        kernel_size = 2 * window + 1
        kernel = np.full(kernel_size, 1.0 / kernel_size, dtype=np.float32)
        profile32 = profile.astype(np.float32, copy=False)
        local_mean = np.convolve(profile32, kernel, mode="same")
        local_mean_sq = np.convolve(profile32 * profile32, kernel, mode="same")
        local_std = np.sqrt(np.maximum(local_mean_sq - local_mean * local_mean, 0.0))
        # Match the original loop exactly: incomplete edge windows were zero.
        local_std[:window] = 0.0
        local_std[roi_w - window:] = 0.0

        valid_std = local_std[window:roi_w - window]
        threshold = float(np.median(valid_std) + max(2.0, valid_std.std() * 2.0))
        candidates = (local_std > threshold).astype(np.uint8)
        candidates = cv2.morphologyEx(
            candidates,
            cv2.MORPH_CLOSE,
            np.ones(max(3, roi_w // 40), np.uint8),
        )

        indices = np.where(candidates > 0)[0]
        if len(indices) == 0:
            return None
        gaps = np.diff(indices)
        split_points = np.where(gaps > max(2, roi_w // 30))[0]
        starts = np.insert(split_points + 1, 0, 0)
        ends = np.append(split_points, len(indices) - 1)
        minimum_width = max(3, int(roi_w * self.min_stain_fraction))
        best: Optional[tuple[int, int, float]] = None
        best_score = -1e9
        for start, end in zip(starts, ends):
            candidate_width = int(indices[end] - indices[start])
            if candidate_width < minimum_width:
                continue
            segment_left = int(indices[start])
            segment_right = int(indices[end])
            positions = np.arange(segment_left, segment_right + 1, dtype=np.float32)
            weights = np.maximum(local_std[segment_left:segment_right + 1] - threshold, 0.01)
            weighted_center = float(np.average(positions, weights=weights))
            peak = float(local_std[segment_left:segment_right + 1].max())
            score = float(candidate_width) + 0.8 * peak
            if self.last_x_warped is not None:
                score -= 0.18 * abs((x_min + weighted_center) - self.last_x_warped)
            if score > best_score:
                best = (segment_left, segment_right, weighted_center)
                best_score = score
        if best is None:
            return None

        left_warped = x_min + best[0]
        right_warped = x_min + best[1]
        centre_warped = x_min + best[2]
        self.last_x_warped = centre_warped
        point_warped = np.array([[[centre_warped, (y_min + y_max) / 2.0]]], np.float32)
        inverse = cv2.invertAffineTransform(matrix)
        original = cv2.transform(point_warped, inverse)[0, 0]
        x = float(np.clip(original[0], 0, width - 1))
        y = float(np.clip(original[1], 0, height - 1))

        # Rectangle coordinates are only used for the rectified debug overlay.
        return BallMeasurement(x, y, left_warped, right_warped, y_min + y1, y_min + y2)

class PositionFilter:
    def __init__(self) -> None:
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.velocity = 0.0
        self.last_time: Optional[float] = None
        self.lost_frames = 0
        self.outlier_frames = 0
        self.history: deque[tuple[float, float]] = deque(maxlen=3)
        self.last_seen_x: Optional[float] = None

    def update(self, measurement: Optional[BallMeasurement], now: float) -> None:
        if measurement is None:
            self.lost_frames += 1
            self.velocity *= 0.90
            self.last_time = now
            return

        raw_x, raw_y = measurement.x, measurement.y
        if self.x is not None and self.last_time is not None and self.lost_frames <= LOST_FRAME_LIMIT:
            dt = max(0.01, min(0.25, now - self.last_time))
            allowed_jump = MAX_MEASUREMENT_JUMP_PX + 120.0 * dt
            if abs(raw_x - self.x) > allowed_jump:
                self.outlier_frames += 1
                self.lost_frames += 1
                self.velocity *= 0.90
                self.last_time = now
                return

        self.history.append((raw_x, raw_y))
        median_x = float(np.median([point[0] for point in self.history]))
        median_y = float(np.median([point[1] for point in self.history]))
        if self.x is None or self.last_time is None or self.lost_frames > LOST_FRAME_LIMIT:
            self.x, self.y = median_x, median_y
            self.velocity = 0.0
        else:
            dt = max(0.01, min(0.25, now - self.last_time))
            previous_x = self.x
            self.x = 0.70 * median_x + 0.30 * self.x
            self.y = 0.70 * median_y + 0.30 * self.y
            measured_velocity = float(np.clip(
                (self.x - previous_x) / dt,
                -MAX_FILTER_VELOCITY_PX_S,
                MAX_FILTER_VELOCITY_PX_S,
            ))
            self.velocity = 0.55 * measured_velocity + 0.45 * self.velocity
        self.last_time = now
        self.lost_frames = 0
        self.outlier_frames = 0
        self.last_seen_x = self.x


class SafeController:
    def __init__(self, motor: Optional[ZDTMotor], motor_sign: int = 0) -> None:
        self.motor = motor
        self.motor_sign = motor_sign
        self.commanded_pulses = 0
        self.last_command_time = 0.0
        self.target_index = 0
        self.state = "MONITOR" if motor is None else "WAIT_LOCK"
        self.status = "Dry-run: motor disabled" if motor is None else "Confirm lock and centre ball; Space arms"
        self.stable_since: Optional[float] = None
        self.probe_start_x: Optional[float] = None
        self.state_started = time.monotonic()
        self.reacquired_since: Optional[float] = None
        self.stuck_since: Optional[float] = None

    @property
    def target(self) -> tuple[int, int]:
        return TARGET_POINTS[self.target_index]

    def set_absolute_pulses(self, requested: int, now: float, rate_limit: bool = True) -> None:
        if self.motor is None:
            return
        bounded = int(np.clip(requested, -MAX_ABS_PULSES, MAX_ABS_PULSES))
        if rate_limit:
            change = int(np.clip(
                bounded - self.commanded_pulses,
                -MAX_CHANGE_PULSES,
                MAX_CHANGE_PULSES,
            ))
            bounded = self.commanded_pulses + change
        # A second independent clamp makes accidental future edits fail safe.
        bounded = int(np.clip(bounded, -MAX_ABS_PULSES, MAX_ABS_PULSES))
        if bounded == self.commanded_pulses:
            return
        if now - self.last_command_time < CONTROL_INTERVAL:
            return
        self.motor.move_absolute_signed(bounded, MOTOR_SPEED_RPM, MOTOR_ACCEL)
        self.commanded_pulses = bounded
        self.last_command_time = now

    def return_level(self, now: float, immediate: bool = False) -> None:
        self.set_absolute_pulses(0, now, rate_limit=not immediate)

    def update_wait_lock(self, filt: PositionFilter, now: float) -> None:
        if filt.x is None or filt.lost_frames:
            self.stable_since = None
            return
        centred = abs(filt.x - TARGET_POINTS[0][0]) <= CENTER_TOLERANCE_PX
        slow = abs(filt.velocity) <= LOCK_MAX_SPEED_PX_S
        if centred and slow:
            if self.stable_since is None:
                self.stable_since = now
            elapsed = now - self.stable_since
            self.status = f"Centre stable {elapsed:.1f}/{LOCK_STABLE_SECONDS:.1f}s; confirm lock, then Space"
        else:
            self.stable_since = None
            self.status = "Place ball at centre and wait until stationary"

    def arm(self, filt: PositionFilter, now: float) -> None:
        if self.state == "WAIT_LOCK":
            ready = self.stable_since is not None and now - self.stable_since >= LOCK_STABLE_SECONDS
            if not ready or filt.x is None:
                self.status = "ARM REFUSED: ball must be stable at centre"
                return
            assert self.motor is not None
            self.motor.set_zero()
            self.commanded_pulses = 0
            self.last_command_time = now
            if self.motor_sign in (-1, 1):
                self.state = "RUN"
                self.status = f"RUN; motor sign={self.motor_sign:+d}"
            else:
                self.probe_start_x = filt.x
                self.state = "PROBE_OUT"
                self.state_started = now
                self.status = "Learning motor direction with +2deg probe"
        elif self.state == "RUN":
            self.return_level(now, immediate=True)
            self.state = "PAUSED"
            self.status = "PAUSED at level; Space resumes"
        elif self.state == "PAUSED":
            self.state = "RUN"
            self.status = "RUN"
        elif self.state == "OVERSPEED":
            if filt.x is None or abs(filt.velocity) > OVERSPEED_RESUME_PX_S:
                self.status = "RESUME REFUSED: wait for the ball to slow"
                return
            self.state = "RUN"
            self.status = "RUN resumed after overspeed stop"

    def update_probe(self, filt: PositionFilter, now: float) -> None:
        probe_pulses = max(1, math.floor(PULSES_PER_REV * PROBE_ANGLE_DEG / 360.0))
        if self.state == "PROBE_OUT":
            self.set_absolute_pulses(probe_pulses, now, rate_limit=False)
            if now - self.state_started >= PROBE_SECONDS:
                movement = 0.0 if filt.x is None or self.probe_start_x is None else filt.x - self.probe_start_x
                self.return_level(now, immediate=True)
                self.state = "PROBE_RETURN"
                self.state_started = now
                if abs(movement) >= PROBE_MIN_MOVEMENT_PX:
                    self.motor_sign = 1 if movement > 0 else -1
                    self.status = f"Direction learned: sign={self.motor_sign:+d}, dx={movement:+.1f}px"
                else:
                    self.status = f"Probe inconclusive: dx={movement:+.1f}px; use --motor-sign"
        elif self.state == "PROBE_RETURN" and now - self.state_started >= PROBE_SECONDS:
            if self.motor_sign in (-1, 1):
                self.state = "RUN"
                self.status = f"RUN; motor sign={self.motor_sign:+d}"
            else:
                self.state = "PAUSED"

    def update_control(self, filt: PositionFilter, now: float) -> None:
        if self.state == "WAIT_LOCK":
            self.update_wait_lock(filt, now)
            return
        if self.state in ("PROBE_OUT", "PROBE_RETURN"):
            self.update_probe(filt, now)
            return
        if self.state == "OVERSPEED":
            self.return_level(now, immediate=True)
            if filt.x is not None and abs(filt.velocity) <= OVERSPEED_RESUME_PX_S:
                self.status = "OVERSPEED stopped; Space resumes"
            return
        if self.state not in ("RUN", "LOST"):
            return

        if filt.x is not None and abs(filt.velocity) >= OVERSPEED_STOP_PX_S:
            self.return_level(now, immediate=True)
            self.state = "OVERSPEED"
            self.status = f"OVERSPEED {filt.velocity:+.1f}px/s; level and pause"
            return

        if filt.x is None or filt.lost_frames > LOST_FRAME_LIMIT:
            self.stuck_since = None
            if self.state != "LOST":
                self.state = "LOST"
                self.state_started = now
            self._search_for_ball(filt, now)
            self.reacquired_since = None
            return

        # During a brief visual dropout never keep accelerating from a stale
        # position.  If the last reliable speed was high, counter-tilt to
        # remove energy; otherwise level the trough.
        if filt.lost_frames:
            self.stuck_since = None
            if abs(filt.velocity) >= FAST_BRAKE_SPEED_PX_S:
                brake_deg = -math.copysign(MAX_MOVING_TILT_DEG, filt.velocity)
                brake_pulses = angle_to_pulses(brake_deg * self.motor_sign)
                self.set_absolute_pulses(brake_pulses, now, rate_limit=True)
                self.status = f"Brief detection gap; brake {brake_deg:+.1f}deg"
            else:
                self.return_level(now)
                self.status = f"Brief detection gap ({filt.lost_frames}); level"
            return

        if self.state == "LOST":
            if self.reacquired_since is None:
                self.reacquired_since = now
            if now - self.reacquired_since < 0.5:
                self.return_level(now)
                return
            self.state = "RUN"
            self.status = "Ball reacquired; RUN"

        error_px = self.target[0] - filt.x
        predicted_x = filt.x + filt.velocity * EDGE_LOOKAHEAD_SECONDS
        outward_edge_risk = (
            (predicted_x <= EDGE_LEFT_X and filt.velocity < 0.0)
            or (predicted_x >= EDGE_RIGHT_X and filt.velocity > 0.0)
        )

        if outward_edge_risk:
            # Do not drive toward a rebounding end stop.  A moderate opposite
            # tilt removes energy without launching the ball back across the
            # entire concave track.
            physical_tilt = -math.copysign(MAX_MOVING_TILT_DEG, filt.velocity)
            self.stuck_since = None
        elif abs(error_px) < POSITION_DEADBAND_PX and abs(filt.velocity) < VELOCITY_DEADBAND_PX_S:
            physical_tilt = 0.0
            self.stuck_since = None
        elif abs(filt.velocity) >= MOVING_SPEED_THRESHOLD_PX_S:
            # Position-to-speed outer loop plus a bounded speed inner loop.
            # Far from the target request at most 65 px/s; close to the target
            # the requested speed tends smoothly to zero.
            desired_velocity = float(np.clip(
                POSITION_TO_SPEED_GAIN * error_px,
                -MAX_CRUISE_SPEED_PX_S,
                MAX_CRUISE_SPEED_PX_S,
            ))
            speed_error = desired_velocity - filt.velocity
            physical_tilt = float(np.clip(
                VELOCITY_GAIN_DEG * speed_error,
                -MAX_MOVING_TILT_DEG,
                MAX_MOVING_TILT_DEG,
            ))
            self.stuck_since = None
        else:
            # Stationary phase: ramp only enough to overcome local static
            # friction.  The moment motion starts, the speed loop above takes
            # over and its limit drops to 10 degrees.
            error_norm = float(np.clip(error_px / POSITION_NORMALIZER_PX, -1.0, 1.0))
            physical_tilt = KP_DEG * error_norm
            if abs(error_px) > BREAKAWAY_MIN_ERROR_PX:
                if self.stuck_since is None:
                    self.stuck_since = now
                elapsed_stuck = now - self.stuck_since
                breakaway = min(
                    BREAKAWAY_MAX_ANGLE_DEG,
                    BREAKAWAY_ANGLE_DEG
                    + 2.0 * int(elapsed_stuck / BREAKAWAY_STEP_SECONDS),
                )
                if abs(physical_tilt) < breakaway:
                    physical_tilt = math.copysign(breakaway, error_px)
            else:
                self.stuck_since = None
        physical_tilt = float(np.clip(
            physical_tilt,
            -MAX_CONTROL_ANGLE_DEG,
            MAX_CONTROL_ANGLE_DEG,
        ))
        motor_target_deg = physical_tilt * self.motor_sign
        target_pulses = int(np.clip(
            angle_to_pulses(motor_target_deg),
            -MAX_ABS_PULSES,
            MAX_ABS_PULSES,
        ))
        self.set_absolute_pulses(target_pulses, now, rate_limit=True)
        self.status = (f"RUN target={TARGET_NAMES[self.target_index]} error={error_px:+.1f}px "
                       f"tilt={physical_tilt:+.2f}deg")

    def _search_for_ball(self, filt: PositionFilter, now: float) -> None:
        """Level and wait: a blind sweep can launch the ball into an end stop."""
        self.stuck_since = None
        self.return_level(now)
        self.status = "BALL LOST: level and wait for visual reacquisition"


def draw_overlay(
    frame: np.ndarray,
    measurement: Optional[BallMeasurement],
    filt: PositionFilter,
    controller: SafeController,
    fps: float,
) -> None:
    cv2.polylines(frame, [QUAD], True, (0, 255, 0), 2)
    for index, point in enumerate(TARGET_POINTS):
        color = (0, 255, 255) if index == controller.target_index else (0, 128, 255)
        cv2.circle(frame, point, 6, color, -1)
        cv2.putText(frame, str(index + 1), (point[0] + 8, point[1] - 8), 0, 0.5, color, 2)
    if measurement is not None:
        cv2.drawMarker(frame, (int(measurement.x), int(measurement.y)), (255, 0, 255), cv2.MARKER_CROSS, 16, 2)
    if filt.x is not None:
        cv2.circle(frame, (int(filt.x), int(filt.y or 0)), 5, (255, 255, 0), -1)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (frame.shape[1], 82), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.65, frame, 0.35, 0)
    distances = ""
    if filt.x is not None and filt.y is not None:
        values = [math.hypot(filt.x - x, filt.y - y) for x, y in TARGET_POINTS]
        distances = f"d1={values[0]:.0f} d2={values[1]:.0f} d3={values[2]:.0f} v={filt.velocity:+.1f}px/s"
    angle = pulses_to_angle(controller.commanded_pulses)
    cv2.putText(frame, controller.status[:100], (8, 21), 0, 0.44, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, f"state={controller.state} cmd={angle:+.2f}deg limit=+/-{MAX_ANGLE_DEG:.0f} fps={fps:.1f}",
                (8, 43), 0, 0.42, (0, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(frame, distances, (8, 62), 0, 0.40, (255, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "Space=arm/pause  1/2/3=target point  E=emergency  Q=level+quit",
                (8, 79), 0, 0.34, (230, 230, 230), 1, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--camera", type=int, default=CAMERA_ID)
    parser.add_argument("--dry-run", action="store_true", help="vision only; never open the motor port")
    parser.add_argument("--motor-sign", type=int, choices=(-1, 1), default=1,
                        help="motor mapping; default +1 was confirmed on this mechanism")
    args = parser.parse_args()

    os.system(
        f"v4l2-ctl -d /dev/video{args.camera} "
        f"--set-fmt-video=width={CAM_W},height={CAM_H},pixelformat=MJPG "
        f"--set-parm=60 2>/dev/null"
    )
    camera = cv2.VideoCapture(args.camera)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    camera.set(cv2.CAP_PROP_FPS, 60)
    if not camera.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera}")

    motor: Optional[ZDTMotor] = None
    controller: Optional[SafeController] = None
    detector = FixedBallDetector()
    filt = PositionFilter()
    try:
        if not args.dry_run:
            motor = ZDTMotor(port=args.port)
            motor.enable()  # lock/hold only; no movement command
            time.sleep(0.5)
        controller = SafeController(motor, args.motor_sign)
        previous = time.perf_counter()
        fps = 0.0
        cv2.namedWindow("Ball Controller", cv2.WINDOW_AUTOSIZE)

        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            now = time.perf_counter()
            dt = max(now - previous, 1e-6)
            previous = now
            fps = 0.9 * fps + 0.1 / dt
            measurement = detector.detect(frame)
            filt.update(measurement, now)
            controller.update_control(filt, now)
            draw_overlay(frame, measurement, filt, controller, fps)
            cv2.imshow("Ball Controller", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                if motor is not None:
                    controller.return_level(now, immediate=True)
                break
            if key == ord(" "):
                controller.arm(filt, now)
            elif key in (ord("1"), ord("2"), ord("3")):
                controller.target_index = key - ord("1")
                controller.status = f"Target {controller.target_index + 1} selected"
            elif key in (ord("e"), ord("E")) and motor is not None:
                motor.stop()
                controller.state = "EMERGENCY"
                controller.status = "EMERGENCY STOP; restart program to continue"
    finally:
        camera.release()
        cv2.destroyAllWindows()
        if motor is not None:
            try:
                # Absolute zero is always inside the hard limit. Leave enabled
                # so the mechanism remains locked at its level reference.
                # After an emergency stop, do not initiate any new movement.
                if controller is None or controller.state != "EMERGENCY":
                    motor.move_absolute_signed(0, MOTOR_SPEED_RPM, MOTOR_ACCEL)
            finally:
                motor.close()


if __name__ == "__main__":
    main()
