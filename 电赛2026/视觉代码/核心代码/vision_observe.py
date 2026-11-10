#!/usr/bin/env python3
"""Fixed-coordinate visual observation tool (no motor access).

This program is only for checking the updated quadrilateral, target points,
ball location, distances and detector stability. It does not import zdt_motor
and cannot send any motor command.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from vision_calibration import QUAD, TARGET_POINTS, TRACK_ENDPOINTS

CAMERA_ID = 0
CAM_W = 640
CAM_H = 480

CAPTURE_DIR = Path(__file__).with_name("camera_captures")


@dataclass
class Result:
    x: float
    y: float
    left_warped: int
    right_warped: int
    y1_warped: int
    y2_warped: int


class ObserverDetector:
    def __init__(self) -> None:
        self.std_multiplier = 2.0
        self.min_width_fraction = 0.05

    def detect(self, frame: np.ndarray) -> tuple[Optional[Result], np.ndarray]:
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
        corners_w = cv2.transform(np.array([corners], np.float32), matrix)[0]
        x_min = max(0, int(np.min(corners_w[:, 0])))
        x_max = min(width - 1, int(np.max(corners_w[:, 0])))
        y_min = max(0, int(np.min(corners_w[:, 1])))
        y_max = min(height - 1, int(np.max(corners_w[:, 1])))
        if x_max - x_min < 30 or y_max - y_min < 5:
            return None, np.zeros((220, 500, 3), np.uint8)

        roi = warped[y_min:y_max, x_min:x_max]
        roi_h, roi_w = roi.shape
        strip_y1, strip_y2 = roi_h // 3, 2 * roi_h // 3
        strip = roi[strip_y1:strip_y2, :]
        profile = strip.mean(axis=0)
        window = max(3, roi_w // 20)
        kernel_size = 2 * window + 1
        kernel = np.full(kernel_size, 1.0 / kernel_size, dtype=np.float32)
        profile32 = profile.astype(np.float32, copy=False)
        local_mean = np.convolve(profile32, kernel, mode="same")
        local_mean_sq = np.convolve(profile32 * profile32, kernel, mode="same")
        local_std = np.sqrt(np.maximum(local_mean_sq - local_mean * local_mean, 0.0))
        local_std[:window] = 0.0
        local_std[roi_w - window:] = 0.0

        valid = local_std[window:roi_w - window]
        base = float(np.median(valid)) if len(valid) else 0.0
        spread = float(valid.std()) if len(valid) else 0.0
        threshold = base + max(2.0, spread * self.std_multiplier)
        candidates = (local_std > threshold).astype(np.uint8)
        candidates = cv2.morphologyEx(
            candidates,
            cv2.MORPH_CLOSE,
            np.ones(max(3, roi_w // 40), np.uint8),
        )

        result: Optional[Result] = None
        indices = np.where(candidates > 0)[0]
        if len(indices):
            gaps = np.diff(indices)
            splits = np.where(gaps > max(2, roi_w // 30))[0]
            starts = np.insert(splits + 1, 0, 0)
            ends = np.append(splits, len(indices) - 1)
            minimum_width = max(3, int(roi_w * self.min_width_fraction))
            best: Optional[tuple[int, int]] = None
            best_width = 0
            for start, end in zip(starts, ends):
                candidate_width = int(indices[end] - indices[start])
                if candidate_width >= minimum_width and candidate_width > best_width:
                    best = (int(indices[start]), int(indices[end]))
                    best_width = candidate_width
            if best is not None:
                left = x_min + best[0]
                right = x_min + best[1]
                point_w = np.array([[[(left + right) / 2.0, (y_min + y_max) / 2.0]]], np.float32)
                inverse = cv2.invertAffineTransform(matrix)
                original = cv2.transform(point_w, inverse)[0, 0]
                result = Result(
                    float(np.clip(original[0], 0, width - 1)),
                    float(np.clip(original[1], 0, height - 1)),
                    left,
                    right,
                    y_min + strip_y1,
                    y_min + strip_y2,
                )

        debug = self._make_debug(roi, profile, local_std, threshold, candidates, result, x_min, y_min)
        return result, debug

    @staticmethod
    def _make_debug(
        roi: np.ndarray,
        profile: np.ndarray,
        local_std: np.ndarray,
        threshold: float,
        candidates: np.ndarray,
        result: Optional[Result],
        x_min: int,
        y_min: int,
    ) -> np.ndarray:
        width = len(profile)
        canvas = np.zeros((260, width, 3), np.uint8)
        roi_display = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        roi_display = cv2.resize(roi_display, (width, 100))
        canvas[:100] = roi_display

        profile_min, profile_max = float(profile.min()), float(profile.max())
        profile_scale = max(profile_max - profile_min, 1.0)
        std_max = max(float(local_std.max()), threshold, 1.0)
        for index in range(1, width):
            p0 = 175 - int((profile[index - 1] - profile_min) / profile_scale * 65)
            p1 = 175 - int((profile[index] - profile_min) / profile_scale * 65)
            cv2.line(canvas, (index - 1, p0), (index, p1), (120, 120, 120), 1)
            s0 = 250 - int(local_std[index - 1] / std_max * 65)
            s1 = 250 - int(local_std[index] / std_max * 65)
            cv2.line(canvas, (index - 1, s0), (index, s1), (0, 255, 255), 1)
        threshold_y = 250 - int(threshold / std_max * 65)
        cv2.line(canvas, (0, threshold_y), (width - 1, threshold_y), (0, 0, 255), 1)
        candidate_y = 180
        for index in np.where(candidates > 0)[0]:
            cv2.line(canvas, (int(index), candidate_y), (int(index), candidate_y + 4), (255, 0, 255), 1)
        if result is not None:
            left = result.left_warped - x_min
            right = result.right_warped - x_min
            cv2.rectangle(canvas, (left, 0), (right, 99), (255, 0, 255), 2)
        cv2.putText(canvas, "ROI", (5, 16), 0, 0.45, (0, 255, 0), 1)
        cv2.putText(canvas, "gray profile", (5, 118), 0, 0.4, (150, 150, 150), 1)
        cv2.putText(canvas, "local std / red threshold", (5, 198), 0, 0.4, (0, 255, 255), 1)
        return canvas


class Filter:
    def __init__(self) -> None:
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.velocity = 0.0
        self.last_time: Optional[float] = None
        self.lost = 0

    def reset(self) -> None:
        self.__init__()

    def update(self, result: Optional[Result], now: float) -> None:
        if result is None:
            self.lost += 1
            self.velocity *= 0.8
            self.last_time = now
            return
        if self.x is None or self.last_time is None or self.lost > 10:
            self.x, self.y, self.velocity = result.x, result.y, 0.0
        else:
            dt = max(0.01, min(0.25, now - self.last_time))
            old_x = self.x
            self.x = 0.55 * result.x + 0.45 * self.x
            self.y = 0.55 * result.y + 0.45 * self.y
            measured_velocity = (self.x - old_x) / dt
            self.velocity = 0.3 * measured_velocity + 0.7 * self.velocity
        self.last_time = now
        self.lost = 0


def main() -> None:
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
        raise RuntimeError("Cannot open /dev/video0")

    detector = ObserverDetector()
    filt = Filter()
    paused = False
    frozen: Optional[np.ndarray] = None
    fps = 0.0
    previous = time.perf_counter()
    CAPTURE_DIR.mkdir(exist_ok=True)
    cv2.namedWindow("Vision Observe", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Detection Debug", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            if not paused or frozen is None:
                ok, frame = camera.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue
                frozen = frame.copy()
            else:
                frame = frozen.copy()

            now = time.perf_counter()
            dt = max(now - previous, 1e-6)
            previous = now
            fps = 0.9 * fps + 0.1 / dt
            result, debug = detector.detect(frame)
            if not paused:
                filt.update(result, now)

            display = frame.copy()
            cv2.polylines(display, [QUAD], True, (0, 255, 0), 2)
            for index, point in enumerate(TARGET_POINTS):
                color = ((0, 255, 255), (255, 150, 0), (0, 150, 255))[index]
                cv2.circle(display, point, 6, color, -1)
                cv2.putText(display, ("C", "L", "R")[index], (point[0] + 8, point[1] - 8), 0, 0.5, color, 2)
            if result is not None:
                cv2.drawMarker(display, (int(result.x), int(result.y)), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
            if filt.x is not None and filt.y is not None:
                cv2.circle(display, (int(filt.x), int(filt.y)), 5, (255, 255, 0), -1)

            state = "BALL LOST"
            distances = ""
            if filt.x is not None and filt.y is not None and filt.lost <= 10:
                values = [math.hypot(filt.x - x, filt.y - y) for x, y in TARGET_POINTS]
                state = f"ball=({filt.x:.1f},{filt.y:.1f}) v={filt.velocity:+.1f}px/s"
                distances = f"centre={values[0]:.1f}px  left={values[1]:.1f}px  right={values[2]:.1f}px"

            overlay = display.copy()
            cv2.rectangle(overlay, (0, 0), (display.shape[1], 78), (0, 0, 0), -1)
            display = cv2.addWeighted(overlay, 0.65, display, 0.35, 0)
            cv2.putText(display, state, (8, 21), 0, 0.46, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(display, distances, (8, 43), 0, 0.43, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(display, f"std_mult={detector.std_multiplier:.1f} fps={fps:.1f} lost={filt.lost}",
                        (8, 61), 0, 0.39, (255, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(display, "+/- threshold  P pause  R reset  S save  Q quit (NO MOTOR)",
                        (8, 76), 0, 0.33, (220, 220, 220), 1, cv2.LINE_AA)
            cv2.imshow("Vision Observe", display)
            cv2.imshow("Detection Debug", debug)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("+"), ord("=")):
                detector.std_multiplier = min(5.0, detector.std_multiplier + 0.1)
            elif key in (ord("-"), ord("_")):
                detector.std_multiplier = max(0.5, detector.std_multiplier - 0.1)
            elif key in (ord("p"), ord("P")):
                paused = not paused
            elif key in (ord("r"), ord("R")):
                filt.reset()
            elif key in (ord("s"), ord("S")):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                image_path = CAPTURE_DIR / f"observe_{stamp}.jpg"
                debug_path = CAPTURE_DIR / f"observe_debug_{stamp}.jpg"
                cv2.imwrite(str(image_path), display)
                cv2.imwrite(str(debug_path), debug)
                print(f"Saved {image_path.name} and {debug_path.name}")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
