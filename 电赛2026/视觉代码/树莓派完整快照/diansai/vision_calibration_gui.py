#!/usr/bin/env python3
"""轨道与目标点可视化标定工具（不连接、不控制电机）。

界面操作：
* 拖动 Q1~Q4：修改轨道四个角，顺序固定为左上、右上、右下、左下。
* 拖动 C/L/R 彩色竖线：只修改中/左/右三个目标点的 X 坐标。
* 目标点 Y 坐标会自动放在当前四边形轨道的中心线上。
* 点击 SAVE 或按 S：原子写入 vision_calibration.json，并先生成 .bak 备份。
* 点击 RELOAD 或按 R：放弃未保存修改，重新读取配置文件。
* 点击 QUIT、按 Q 或 Esc：退出。

该文件参考 detect_rect.py 的摄像头和鼠标窗口实现，但不会打开电机串口。
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
PANEL_WIDTH = 300
WINDOW_NAME = "Track Calibration (NO MOTOR)"
HANDLE_RADIUS = 16

CORNER_NAMES = ("Q1 TL", "Q2 TR", "Q3 BR", "Q4 BL")
TARGET_NAMES = ("C CENTER", "L LEFT", "R RIGHT")
TARGET_COLORS = ((0, 255, 255), (255, 150, 0), (0, 140, 255))


def inside(rect: tuple[int, int, int, int], x: int, y: int) -> bool:
    rx, ry, rw, rh = rect
    return rx <= x <= rx + rw and ry <= y <= ry + rh


def draw_button(
    image: np.ndarray,
    rect: tuple[int, int, int, int],
    text: str,
    color: tuple[int, int, int],
) -> None:
    x, y, w, h = rect
    cv2.rectangle(image, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(image, (x, y), (x + w, y + h), (220, 220, 220), 1)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.putText(
        image,
        text,
        (x + (w - tw) // 2, y + (h + th) // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


class CalibrationGUI:
    def __init__(self, config_path: Path, camera_id: int) -> None:
        self.config_path = config_path
        self.camera_id = camera_id
        self.frame_w = CAMERA_WIDTH
        self.frame_h = CAMERA_HEIGHT
        self.config: dict = {}
        self.quad = np.zeros((4, 2), dtype=np.int32)
        # 配置格式顺序始终是 centre、left、right；界面只编辑横坐标。
        self.target_x = np.zeros(3, dtype=np.int32)
        self.selected: Optional[tuple[str, int]] = None
        self.dragging = False
        self.paused = False
        self.frozen_frame: Optional[np.ndarray] = None
        self.status = "Ready"
        self.status_color = (180, 220, 180)

        panel_x = self.frame_w + 20
        self.save_button = (panel_x, 346, PANEL_WIDTH - 40, 36)
        self.reload_button = (panel_x, 392, PANEL_WIDTH - 40, 36)
        self.quit_button = (panel_x, 438, PANEL_WIDTH - 40, 30)
        self.load_config(initial=True)

    def load_config(self, initial: bool = False) -> None:
        last_good_path = self.config_path.with_name(
            f"{self.config_path.stem}.last_good{self.config_path.suffix}"
        )
        backup_path = self.config_path.with_suffix(self.config_path.suffix + ".bak")
        data = None
        loaded_path = None
        errors = []
        for candidate in (self.config_path, last_good_path, backup_path):
            if not candidate.exists():
                errors.append(f"{candidate.name}: missing")
                continue
            try:
                candidate_data = json.loads(candidate.read_text(encoding="utf-8"))
                candidate_frame = list(map(int, candidate_data.get("frame_size", [])))
                candidate_quad = np.asarray(candidate_data.get("track_quad"), dtype=np.int32)
                candidate_targets = np.asarray(candidate_data.get("target_points"), dtype=np.int32)
                if candidate_frame != [CAMERA_WIDTH, CAMERA_HEIGHT]:
                    raise ValueError("frame_size must be [640, 480]")
                if candidate_quad.shape != (4, 2):
                    raise ValueError("track_quad must contain four [x, y] points")
                if candidate_targets.shape != (3, 2):
                    raise ValueError("target_points must contain three [x, y] points")
                data = candidate_data
                loaded_path = candidate
                break
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
                errors.append(f"{candidate.name}: {exc}")
        if data is None or loaded_path is None:
            raise ValueError("no valid saved configuration; " + " | ".join(errors))
        frame_size = data.get("frame_size", [CAMERA_WIDTH, CAMERA_HEIGHT])
        if list(map(int, frame_size)) != [CAMERA_WIDTH, CAMERA_HEIGHT]:
            raise ValueError("GUI currently requires frame_size [640, 480]")
        quad = np.asarray(data.get("track_quad"), dtype=np.int32)
        targets = np.asarray(data.get("target_points"), dtype=np.int32)
        if quad.shape != (4, 2):
            raise ValueError("track_quad must contain four [x, y] points")
        if targets.shape != (3, 2):
            raise ValueError("target_points must contain three [x, y] points")
        self.config = data
        self.quad = quad.copy()
        self.target_x = targets[:, 0].copy()
        self.selected = None
        self.dragging = False
        if loaded_path == self.config_path:
            self.status = "Loaded configuration" if initial else "Reloaded; edits discarded"
        else:
            self.status = f"Primary missing; using {loaded_path.name}"
        self.status_color = (180, 220, 180)

    @staticmethod
    def interpolate_y(a: np.ndarray, b: np.ndarray, x: float) -> float:
        dx = float(b[0] - a[0])
        if abs(dx) < 1e-6:
            return float(a[1] + b[1]) * 0.5
        t = (float(x) - float(a[0])) / dx
        return float(a[1]) + t * float(b[1] - a[1])

    def target_point(self, index: int) -> tuple[int, int]:
        """根据目标X与四边形上下边界，自动求轨道中心线上的Y。"""
        x = int(self.target_x[index])
        top_y = self.interpolate_y(self.quad[0], self.quad[1], x)
        bottom_y = self.interpolate_y(self.quad[3], self.quad[2], x)
        y = int(round((top_y + bottom_y) * 0.5))
        return x, int(np.clip(y, 0, self.frame_h - 1))

    def active_quad(self) -> np.ndarray:
        """预览 vision_calibration.py 应用左右裁边后的实际检测区域。"""
        active = self.quad.copy()
        inset = self.config.get("edge_inset_px", {})
        left = int(inset.get("left", 0))
        right = int(inset.get("right", 0))
        order = np.argsort(active[:, 0])
        active[order[:2], 0] += left
        active[order[-2:], 0] -= right
        return active

    def validation_error(self) -> Optional[str]:
        if np.any(self.quad[:, 0] < 0) or np.any(self.quad[:, 0] >= self.frame_w):
            return "A corner X is outside the image"
        if np.any(self.quad[:, 1] < 0) or np.any(self.quad[:, 1] >= self.frame_h):
            return "A corner Y is outside the image"
        contour = self.quad.reshape((-1, 1, 2)).astype(np.int32)
        if abs(cv2.contourArea(contour)) < 500.0:
            return "Track quadrilateral is too small"
        if not cv2.isContourConvex(contour):
            return "Corner order must be TL, TR, BR, BL"
        left_mean = float(self.quad[[0, 3], 0].mean())
        right_mean = float(self.quad[[1, 2], 0].mean())
        top_mean = float(self.quad[[0, 1], 1].mean())
        bottom_mean = float(self.quad[[2, 3], 1].mean())
        if left_mean >= right_mean or top_mean >= bottom_mean:
            return "Corner order must be TL, TR, BR, BL"

        centre, left, right = map(int, self.target_x)
        if not left < centre < right:
            return "Target X order must satisfy L < C < R"
        min_x = int(self.quad[:, 0].min())
        max_x = int(self.quad[:, 0].max())
        if any(x < min_x or x > max_x for x in (centre, left, right)):
            return "Every target X must stay inside the track span"
        return None

    def save_config(self) -> None:
        error = self.validation_error()
        if error:
            self.status = "NOT SAVED: " + error
            self.status_color = (80, 80, 255)
            print("[WARN]", self.status)
            return

        data = dict(self.config)
        data["frame_size"] = [self.frame_w, self.frame_h]
        data["track_quad"] = self.quad.astype(int).tolist()
        data["target_points"] = [list(self.target_point(i)) for i in range(3)]

        backup_path = self.config_path.with_suffix(self.config_path.suffix + ".bak")
        last_good_path = self.config_path.with_name(
            f"{self.config_path.stem}.last_good{self.config_path.suffix}"
        )
        temporary_path = self.config_path.with_suffix(self.config_path.suffix + ".tmp")
        if self.config_path.exists():
            shutil.copy2(self.config_path, backup_path)
        temporary_path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary_path, self.config_path)
        # last_good 始终保存最近一次成功写入的完整配置。主文件缺失或损坏时，
        # vision_calibration.py 和本界面都会自动使用它。
        shutil.copy2(self.config_path, last_good_path)
        self.config = data
        self.status = f"Saved {self.config_path.name}; last-good updated"
        self.status_color = (80, 255, 80)
        print("[INFO]", self.status)
        print("[INFO] track_quad =", data["track_quad"])
        print("[INFO] target_points =", data["target_points"])

    def nearest_handle(self, x: int, y: int) -> Optional[tuple[str, int]]:
        point = np.array([x, y], dtype=np.float32)
        distances = np.linalg.norm(self.quad.astype(np.float32) - point, axis=1)
        corner_index = int(np.argmin(distances))
        if distances[corner_index] <= HANDLE_RADIUS:
            return "corner", corner_index

        # 目标点只编辑X，因此整条彩色竖线都可以抓取。
        line_distances = np.abs(self.target_x.astype(np.int32) - int(x))
        target_index = int(np.argmin(line_distances))
        if line_distances[target_index] <= 10:
            return "target", target_index
        return None

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            if inside(self.save_button, x, y):
                self.save_config()
                return
            if inside(self.reload_button, x, y):
                try:
                    self.load_config()
                except Exception as exc:  # 界面中显示配置读取错误，而不是直接消失。
                    self.status = f"Reload failed: {exc}"
                    self.status_color = (80, 80, 255)
                return
            if inside(self.quit_button, x, y):
                self.status = "QUIT_REQUESTED"
                return
            if x < self.frame_w and y < self.frame_h:
                handle = self.nearest_handle(x, y)
                if handle is not None:
                    self.selected = handle
                    self.dragging = True
                    self.apply_drag(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.apply_drag(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False

    def apply_drag(self, x: int, y: int) -> None:
        if self.selected is None:
            return
        kind, index = self.selected
        x = int(np.clip(x, 0, self.frame_w - 1))
        y = int(np.clip(y, 0, self.frame_h - 1))
        if kind == "corner":
            self.quad[index] = (x, y)
            self.status = f"Edited {CORNER_NAMES[index]} = ({x}, {y})"
        else:
            self.target_x[index] = x
            self.status = f"Edited {TARGET_NAMES[index]} X = {x}"
        self.status_color = (220, 220, 120)

    def draw_camera_overlay(self, frame: np.ndarray) -> np.ndarray:
        display = frame.copy()
        # 灰色线表示应用 edge_inset 后的检测区域，绿色线表示用户四角。
        cv2.polylines(display, [self.active_quad()], True, (120, 120, 120), 1)
        cv2.polylines(display, [self.quad], True, (0, 255, 0), 2)

        # 中心线帮助确认三个目标点的自动Y是否位于轨道中央。
        centreline = np.array(
            [self.target_point_from_x(x) for x in np.linspace(
                int(self.quad[[0, 3], 0].mean()),
                int(self.quad[[1, 2], 0].mean()),
                60,
            )],
            dtype=np.int32,
        )
        cv2.polylines(display, [centreline], False, (100, 180, 100), 1)

        for index, point in enumerate(self.quad):
            p = tuple(map(int, point))
            selected = self.selected == ("corner", index)
            cv2.circle(display, p, 8 if selected else 6, (0, 255, 255) if selected else (0, 255, 0), -1)
            cv2.putText(
                display,
                f"Q{index + 1} ({p[0]},{p[1]})",
                (max(2, min(p[0] + 9, self.frame_w - 120)), max(15, p[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        for index, color in enumerate(TARGET_COLORS):
            x, y = self.target_point(index)
            thickness = 3 if self.selected == ("target", index) else 1
            cv2.line(display, (x, 0), (x, self.frame_h - 1), color, thickness)
            cv2.circle(display, (x, y), 7, color, -1)
            cv2.putText(
                display,
                f"{TARGET_NAMES[index].split()[0]} x={x}",
                (max(2, min(x + 7, self.frame_w - 75)), max(16, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )

        overlay = display.copy()
        cv2.rectangle(overlay, (0, self.frame_h - 28), (self.frame_w, self.frame_h), (0, 0, 0), -1)
        display = cv2.addWeighted(overlay, 0.65, display, 0.35, 0)
        cv2.putText(
            display,
            "Drag Q1-Q4 corners or C/L/R vertical lines | P pause",
            (8, self.frame_h - 9),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        return display

    def target_point_from_x(self, x: float) -> tuple[int, int]:
        top_y = self.interpolate_y(self.quad[0], self.quad[1], x)
        bottom_y = self.interpolate_y(self.quad[3], self.quad[2], x)
        return int(round(x)), int(round((top_y + bottom_y) * 0.5))

    def draw_panel(self, display: np.ndarray) -> None:
        x0 = self.frame_w
        display[:, x0:] = (38, 40, 48)
        cv2.putText(display, "TRACK CALIBRATION", (x0 + 18, 28), 0, 0.57, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(display, "NO MOTOR ACCESS", (x0 + 18, 50), 0, 0.42, (80, 220, 255), 1, cv2.LINE_AA)

        y = 79
        for index, name in enumerate(CORNER_NAMES):
            px, py = map(int, self.quad[index])
            color = (0, 255, 255) if self.selected == ("corner", index) else (160, 230, 160)
            cv2.putText(display, f"{name}: ({px:3d}, {py:3d})", (x0 + 18, y), 0, 0.44, color, 1, cv2.LINE_AA)
            y += 24

        y += 9
        for index, name in enumerate(TARGET_NAMES):
            px, py = self.target_point(index)
            cv2.putText(display, f"{name}: x={px:3d}  auto-y={py:3d}", (x0 + 18, y), 0, 0.42, TARGET_COLORS[index], 1, cv2.LINE_AA)
            y += 24

        inset = self.config.get("edge_inset_px", {})
        cv2.putText(
            display,
            f"Detection inset L={inset.get('left', 0)} R={inset.get('right', 0)}",
            (x0 + 18, y + 8),
            0,
            0.38,
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(display, "S save | R reload | Q/Esc quit", (x0 + 18, y + 33), 0, 0.38, (200, 200, 200), 1, cv2.LINE_AA)

        # 状态分成两行，避免长路径溢出侧栏。
        status = self.status
        cv2.putText(display, status[:38], (x0 + 18, 304), 0, 0.37, self.status_color, 1, cv2.LINE_AA)
        if len(status) > 38:
            cv2.putText(display, status[38:76], (x0 + 18, 324), 0, 0.37, self.status_color, 1, cv2.LINE_AA)

        draw_button(display, self.save_button, "SAVE CONFIG", (20, 125, 35))
        draw_button(display, self.reload_button, "RELOAD", (115, 80, 20))
        draw_button(display, self.quit_button, "QUIT", (75, 55, 120))

    def run(self) -> None:
        os.system(
            f"v4l2-ctl -d /dev/video{self.camera_id} "
            f"--set-fmt-video=width={self.frame_w},height={self.frame_h},pixelformat=MJPG "
            f"--set-parm=60 2>/dev/null"
        )
        camera = cv2.VideoCapture(self.camera_id)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        camera.set(cv2.CAP_PROP_FPS, 60)
        if not camera.isOpened():
            raise RuntimeError(f"Cannot open /dev/video{self.camera_id}")

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        print("[INFO] Drag Q1-Q4 or C/L/R lines; S=save R=reload P=pause Q=quit")
        try:
            while True:
                # 标题栏叉号不会自动结束OpenCV的Python循环；显式检测，
                # 防止窗口被下一次imshow重新创建而表现为“关不掉”。
                try:
                    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
                if not self.paused or self.frozen_frame is None:
                    ok, frame = camera.read()
                    if not ok or frame is None:
                        time.sleep(0.01)
                        continue
                    self.frozen_frame = frame.copy()
                frame = self.frozen_frame.copy()
                camera_view = self.draw_camera_overlay(frame)
                display = np.zeros((self.frame_h, self.frame_w + PANEL_WIDTH, 3), np.uint8)
                display[:, :self.frame_w] = camera_view
                self.draw_panel(display)
                cv2.imshow(WINDOW_NAME, display)

                if self.status == "QUIT_REQUESTED":
                    break
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("s"), ord("S")):
                    self.save_config()
                elif key in (ord("r"), ord("R")):
                    try:
                        self.load_config()
                    except Exception as exc:
                        self.status = f"Reload failed: {exc}"
                        self.status_color = (80, 80, 255)
                elif key in (ord("p"), ord("P")):
                    self.paused = not self.paused
                    self.status = "Camera paused" if self.paused else "Camera live"
                    self.status_color = (220, 220, 120)
        finally:
            camera.release()
            cv2.destroyAllWindows()


def main() -> None:
    default_config = Path(__file__).with_name("vision_calibration.json")
    parser = argparse.ArgumentParser(description="Visual editor for track corners and three target X coordinates")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()
    CalibrationGUI(args.config.resolve(), args.camera).run()


if __name__ == "__main__":
    main()
