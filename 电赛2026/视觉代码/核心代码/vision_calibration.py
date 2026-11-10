#!/usr/bin/env python3
"""Load and validate the shared camera calibration parameters."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np


DEFAULT_CONFIG_PATH = Path(__file__).with_name("vision_calibration.json")
CONFIG_PATH = Path(os.environ.get("DIANSAI_CALIBRATION", DEFAULT_CONFIG_PATH))

# 仅当主配置、上一次有效配置和备份都不存在/损坏时才使用这一组初始值。
BUILTIN_FALLBACK = {
    "frame_size": [640, 480],
    "track_quad": [[69, 243], [567, 238], [567, 277], [69, 290]],
    "target_points": [[355, 261], [211, 264], [492, 257]],
    "edge_inset_px": {"left": 26, "right": 25},
}


def _points(value: object, name: str, count: int) -> np.ndarray:
    result = np.asarray(value, dtype=np.int32)
    if result.shape != (count, 2):
        raise ValueError(f"{name} must contain exactly {count} [x, y] points")
    return result


def _polygon_area(points: np.ndarray) -> float:
    x = points[:, 0].astype(np.float64)
    y = points[:, 1].astype(np.float64)
    return abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))) / 2.0


def _parse_calibration(data: dict) -> tuple[np.ndarray, np.ndarray, list[tuple[int, int]]]:
    frame_size = data.get("frame_size", [640, 480])
    if not isinstance(frame_size, list) or len(frame_size) != 2:
        raise ValueError("frame_size must be [width, height]")
    frame_w, frame_h = (int(frame_size[0]), int(frame_size[1]))
    if frame_w <= 0 or frame_h <= 0:
        raise ValueError("frame_size values must be positive")

    track = _points(data.get("track_quad"), "track_quad", 4)
    targets_array = _points(data.get("target_points"), "target_points", 3)
    for name, points in (("track_quad", track), ("target_points", targets_array)):
        if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= frame_w):
            raise ValueError(f"{name} contains an X coordinate outside the frame")
        if np.any(points[:, 1] < 0) or np.any(points[:, 1] >= frame_h):
            raise ValueError(f"{name} contains a Y coordinate outside the frame")
    if _polygon_area(track) < 100.0:
        raise ValueError("track_quad is degenerate or its points are not in boundary order")

    inset = data.get("edge_inset_px", {})
    if not isinstance(inset, dict):
        raise ValueError("edge_inset_px must contain left and right values")
    left_inset = int(inset.get("left", 0))
    right_inset = int(inset.get("right", 0))
    if not 0 <= left_inset <= 100 or not 0 <= right_inset <= 100:
        raise ValueError("edge insets must be between 0 and 100 pixels")

    active = track.copy()
    x_order = np.argsort(active[:, 0])
    active[x_order[:2], 0] += left_inset
    active[x_order[-2:], 0] -= right_inset
    if _polygon_area(active) < 100.0 or int(active[:, 0].max() - active[:, 0].min()) < 100:
        raise ValueError("edge insets leave an invalid detection polygon")

    targets = [(int(x), int(y)) for x, y in targets_array]
    return track, active, targets


def _fallback_paths(path: Path) -> list[Path]:
    """主配置不存在时，依次尝试最近有效配置和保存前备份。"""
    last_good = path.with_name(f"{path.stem}.last_good{path.suffix}")
    backup = path.with_suffix(path.suffix + ".bak")
    return [path, last_good, backup]


def load_calibration(path: Path = CONFIG_PATH) -> tuple[np.ndarray, np.ndarray, list[tuple[int, int]]]:
    """加载共享标定；没有新配置时自动沿用上一次有效配置。"""
    path = Path(path)
    errors: list[str] = []
    for index, candidate in enumerate(_fallback_paths(path)):
        if not candidate.exists():
            errors.append(f"{candidate.name}: missing")
            continue
        try:
            data = json.loads(candidate.read_text(encoding="utf-8"))
            result = _parse_calibration(data)
            if index:
                print(
                    f"[vision_calibration] primary unavailable; using {candidate.name}",
                    file=sys.stderr,
                )
            return result
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            errors.append(f"{candidate.name}: {exc}")

    print(
        "[vision_calibration] no valid saved configuration; using built-in defaults; "
        + " | ".join(errors),
        file=sys.stderr,
    )
    return _parse_calibration(BUILTIN_FALLBACK)


TRACK_ENDPOINTS, QUAD, TARGET_POINTS = load_calibration()
