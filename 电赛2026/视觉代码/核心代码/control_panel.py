#!/usr/bin/env python3
"""电赛小球控制总面板（四个主功能按钮）。

按钮：
1. SEQUENCE：依次到中目标、左目标、右目标。
2. HOLD CENTRE：持续把钢球稳定在中目标，直到按 STOP。
3. INITIAL LOCK 3S：钢球稳定三秒后，把当前机械位置设零并锁定。
4. COORD CONFIG：打开四角和三个目标横坐标配置窗口。

总面板正常运行时不打开摄像头或电机串口；它只保证同一时刻启动一个
子程序，从而避免多个程序争用设备。仅在子程序无法正常响应停止信号的
极端情况下，面板才会在强制结束后补发一次绝对零位命令。
"""
from __future__ import annotations

import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import fcntl
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


# 控制面板文件放在桌面，但被控制的项目代码仍统一保存在 diansai 目录。
ROOT = Path(os.environ.get("DIANSAI_ROOT", "/home/bg/diansai")).resolve()
WINDOW_NAME = "Ball Control Panel"
WIN_W = 1000
WIN_H = 680
LIVE_FRAME_PATH = Path("/tmp/diansai_live.jpg")
PANEL_LOCK_PATH = Path("/tmp/diansai_control_panel.lock")
CHINESE_FONT_PATH = "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
LATIN_FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
_FONT_CACHE: dict[tuple[str, int], ImageFont.FreeTypeFont] = {}

# 树莓派比赛屏幕实测为 1024x768。摄像头画面嵌入主窗口，避免旧版将
# 独立预览窗口放到 x=850 后大半部分跑到屏幕外。
PREVIEW_X = 540
PREVIEW_Y = 330
PREVIEW_W = 440
PREVIEW_H = 330
UI_REFRESH_INTERVAL = 0.10

# x, y, width, height
BUTTONS = {
    "sequence": (20, 70, 460, 70),
    "centre": (520, 70, 460, 70),
    "initial": (20, 152, 460, 70),
    "config": (520, 152, 460, 70),
    "stop": (20, 234, 960, 58),
}

KNOWN_EXCLUSIVE_SCRIPTS = (
    "disturbance_stabilizer.py",
    "pulse_controller.py",
    "lock_current_origin.py",
    "ball_controller.py",
    "relative_left_controller.py",
    "vision_calibration_gui.py",
    "vision_observe.py",
    "motor_init.py",
    "hold_balance.py",
    "unlock_motor.py",
)


def inside(rect: tuple[int, int, int, int], x: int, y: int) -> bool:
    rx, ry, rw, rh = rect
    return rx <= x <= rx + rw and ry <= y <= ry + rh


def draw_button(
    frame: np.ndarray,
    rect: tuple[int, int, int, int],
    color: tuple[int, int, int],
    enabled: bool = True,
) -> None:
    x, y, w, h = rect
    fill = color if enabled else tuple(int(v * 0.35) for v in color)
    cv2.rectangle(frame, (x, y), (x + w, y + h), fill, -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (220, 220, 220), 2)


def text_font(path: str, size: int) -> ImageFont.FreeTypeFont:
    key = (path, size)
    font = _FONT_CACHE.get(key)
    if font is None:
        font = ImageFont.truetype(path, size)
        _FONT_CACHE[key] = font
    return font


def font_for_character(character: str, size: int) -> ImageFont.FreeTypeFont:
    """Droid中文字体不保证包含数字；ASCII单独使用DejaVu，避免方框。"""
    path = LATIN_FONT_PATH if ord(character) < 128 else CHINESE_FONT_PATH
    return text_font(path, size)


def fit_text(text: str, size: int, max_width: float) -> str:
    """按实际字体宽度截断，防止中文状态文字覆盖右侧摄像头画面。"""
    width = 0.0
    result: list[str] = []
    # 使用三个ASCII句点；当前Droid中文字体不包含单字符省略号，后者会
    # 显示成用户之前遇到的小方框。
    ellipsis = "..."
    ellipsis_font = font_for_character(".", size)
    ellipsis_width = ellipsis_font.getlength(ellipsis)
    for character in text:
        font = font_for_character(character, size)
        character_width = font.getlength(character)
        if width + character_width + ellipsis_width > max_width:
            return "".join(result) + ellipsis
        result.append(character)
        width += character_width
    return "".join(result)


def render_texts(
    frame: np.ndarray,
    items: list[tuple[str, tuple[int, int], int, tuple[int, int, int]]],
) -> np.ndarray:
    """一次完成所有中英文文字绘制，避免OpenCV不支持中文的问题。"""
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    for text, position, size, bgr in items:
        cursor_x = float(position[0])
        fill = (int(bgr[2]), int(bgr[1]), int(bgr[0]))
        # 将连续ASCII/中文字符合并成一段绘制。旧版逐字符调用Pillow，
        # 在树莓派上让空闲面板占用约88% CPU，会与视觉闭环争抢算力。
        start = 0
        while start < len(text):
            ascii_run = ord(text[start]) < 128
            end = start + 1
            while end < len(text) and (ord(text[end]) < 128) == ascii_run:
                end += 1
            run = text[start:end]
            font = font_for_character(run[0], size)
            draw.text((cursor_x, position[1]), run, font=font, fill=fill)
            cursor_x += draw.textlength(run, font=font)
            start = end
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


class ControlPanel:
    def __init__(self) -> None:
        self.process: Optional[subprocess.Popen[str]] = None
        self.worker: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.messages: queue.Queue[str] = queue.Queue()
        self.output_lines: list[str] = []
        self.state = "IDLE"
        self.detail = "请选择一个功能"
        self.exit_requested = False
        self.exit_after_stop = False
        self.stop_requested = threading.Event()
        self.coordinates = self.read_coordinates()
        self.active_mode: Optional[str] = None
        self.preview_mtime_ns = 0
        self.preview_frame: Optional[np.ndarray] = None
        self.base_frame_cache: Optional[np.ndarray] = None
        self.base_frame_signature: Optional[tuple[object, ...]] = None

    def read_coordinates(self) -> str:
        primary = ROOT / "vision_calibration.json"
        last_good = ROOT / "vision_calibration.last_good.json"
        for path in (primary, last_good):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                targets = data["target_points"]
                quad = data["track_quad"]
                return (
                    f"目标横坐标 中/左/右：{targets[0][0]} / {targets[1][0]} / {targets[2][0]}"
                    f"    轨道四角：{quad}"
                )
            except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
        return "没有有效的坐标配置"

    def emit(self, text: str) -> None:
        self.messages.put(text.rstrip())

    def update_messages(self) -> None:
        while True:
            try:
                text = self.messages.get_nowait()
            except queue.Empty:
                break
            if text:
                self.output_lines.append(text)
                self.output_lines = self.output_lines[-7:]

    def is_busy(self) -> bool:
        with self.lock:
            return self.worker is not None and self.worker.is_alive()

    def external_conflict(self) -> Optional[str]:
        """拒绝启动第二个摄像头/电机程序。"""
        result = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        own_pid = os.getpid()
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split(maxsplit=1)
            try:
                pid = int(parts[0])
            except (ValueError, IndexError):
                continue
            if pid == own_pid:
                continue
            command = parts[1] if len(parts) > 1 else ""
            for script in KNOWN_EXCLUSIVE_SCRIPTS:
                if script in command:
                    return f"检测到冲突程序：PID {pid}，{script}"
        return None

    def preflight_error(self, mode: str) -> Optional[str]:
        """点击后先检查比赛必需文件和设备，给出可操作的中文错误。"""
        script_for_mode = {
            "sequence": "pulse_controller.py",
            "centre": "disturbance_stabilizer.py",
            "initial": "lock_current_origin.py",
            "config": "vision_calibration_gui.py",
        }
        script = script_for_mode.get(mode)
        if script is None:
            return "未知功能"
        if not (ROOT / script).is_file():
            return f"缺少程序文件：{ROOT / script}"
        if not Path("/dev/video0").exists():
            return "未检测到摄像头 /dev/video0，请检查USB连接"
        if mode != "config" and not Path("/dev/ttyUSB0").exists():
            return "未检测到电机串口 /dev/ttyUSB0，请检查USB转485"
        return None

    def launch(self, mode: str) -> None:
        if self.is_busy():
            # 不覆盖真实的 RUNNING/STOPPING 状态；误点另一个按钮只提示。
            self.detail = "请先点击停止，再选择其他功能"
            self.emit("当前功能仍在运行，已忽略本次点击")
            return
        conflict = self.external_conflict()
        if conflict:
            self.state = "CONFLICT"
            self.detail = conflict
            return
        error = self.preflight_error(mode)
        if error:
            self.state = "ERROR"
            self.detail = error
            self.emit(error)
            return

        if mode == "sequence":
            commands = [
                ([sys.executable, "pulse_controller.py", "--target", "centre", "--max-pulses", "12"], None),
                ([sys.executable, "pulse_controller.py", "--target", "left", "--max-pulses", "12"], None),
                ([sys.executable, "pulse_controller.py", "--target", "right", "--max-pulses", "12"], None),
            ]
            label = "顺序运行：中目标 到 左目标 到 右目标"
        elif mode == "centre":
            commands = [([sys.executable, "disturbance_stabilizer.py"], None)]
            label = "持续保持中目标，直到点击停止"
        elif mode == "initial":
            commands = [([sys.executable, "lock_current_origin.py"], None)]
            label = "稳定3秒后设置初始零位并锁定"
        elif mode == "config":
            env = os.environ.copy()
            env.setdefault("DISPLAY", ":0")
            env.setdefault("XAUTHORITY", "/home/bg/.Xauthority")
            commands = [([sys.executable, "vision_calibration_gui.py"], env)]
            label = "轨道与目标坐标配置"
        else:
            return

        self.state = "STARTING"
        self.detail = label
        self.output_lines.clear()
        self.active_mode = mode
        self.stop_requested.clear()
        self.preview_mtime_ns = 0
        self.preview_frame = None
        try:
            LIVE_FRAME_PATH.unlink(missing_ok=True)
        except OSError:
            pass
        self.worker = threading.Thread(
            target=self.run_commands,
            args=(label, commands, mode),
            daemon=True,
        )
        self.worker.start()

    def run_commands(
        self,
        label: str,
        commands: list[tuple[list[str], Optional[dict[str, str]]]],
        mode: str,
    ) -> None:
        self.state = "RUNNING"
        self.detail = label
        self.emit(f"START {label}")
        success = True
        cancelled = False
        try:
            for index, (command, environment) in enumerate(commands, start=1):
                # STOP可能恰好按在顺序任务的两步之间；旧版此时找不到
                # self.process，会错误地继续启动下一步。事件标志堵住这个竞态。
                if self.stop_requested.is_set():
                    cancelled = True
                    break
                if len(commands) > 1:
                    self.emit(f"STEP {index}/{len(commands)}: {' '.join(command[1:])}")
                process = subprocess.Popen(
                    command,
                    cwd=str(ROOT),
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
                with self.lock:
                    self.process = process
                # 覆盖“Popen刚创建但STOP线程尚未看到process”的极短窗口。
                if self.stop_requested.is_set():
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                assert process.stdout is not None
                for line in process.stdout:
                    self.emit(line)
                return_code = process.wait()
                with self.lock:
                    self.process = None
                if self.stop_requested.is_set():
                    cancelled = True
                    break
                if return_code != 0:
                    success = False
                    self.emit(f"程序异常结束，返回码 {return_code}；后续步骤已取消")
                    break
            if cancelled:
                self.state = "STOPPED"
                self.detail = "已安全停止，未继续执行后续步骤"
                self.emit("STOPPED")
            elif success:
                self.state = "COMPLETE" if mode != "centre" else "STOPPED"
                self.detail = f"已完成：{label}"
                self.emit("COMPLETE")
        except Exception as exc:
            success = False
            self.state = "ERROR"
            self.detail = str(exc)
            self.emit(f"ERROR: {exc}")
        finally:
            with self.lock:
                self.process = None
            if mode == "config":
                self.coordinates = self.read_coordinates()
            self.active_mode = None
            if not success and self.state != "ERROR":
                self.state = "FAILED"

    def stop(self) -> None:
        if self.state == "STOPPING":
            return
        busy = self.is_busy()
        with self.lock:
            process = self.process
        if not busy and (process is None or process.poll() is not None):
            self.state = "IDLE"
            self.detail = "当前没有正在运行的子程序"
            return
        # 即使当前恰好处于顺序任务两步之间，也必须取消后续步骤。
        self.stop_requested.set()
        self.state = "STOPPING"
        self.detail = "正在安全停止，并请求电机回到零位"
        self.emit("USER STOP")
        if process is not None and process.poll() is None:
            threading.Thread(
                target=self._stop_process,
                args=(process, self.active_mode),
                daemon=True,
            ).start()

    def _stop_process(
        self,
        process: subprocess.Popen[str],
        mode: Optional[str],
    ) -> None:
        forced = False
        try:
            os.killpg(process.pid, signal.SIGINT)
            try:
                # 所有控制程序都用finally回零；正常情况下会在1秒内退出。
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # 极端卡死时终止进程后，由面板补发绝对零位，避免电机
                # 保持在最后一个倾角。
                forced = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=1.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        if forced and mode != "config":
            self.emit("正常停止超时，正在补发电机零位命令")
            try:
                if str(ROOT) not in sys.path:
                    sys.path.insert(0, str(ROOT))
                from zdt_motor import ZDTMotor

                with ZDTMotor(port="/dev/ttyUSB0") as motor:
                    motor.move_absolute_signed(0, 120, 220)
                self.emit("已补发电机零位命令")
            except Exception as exc:
                self.emit(f"警告：补发零位失败：{exc}")

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if inside(BUTTONS["stop"], x, y):
            self.stop()
            return
        for mode in ("sequence", "centre", "initial", "config"):
            if inside(BUTTONS[mode], x, y):
                self.launch(mode)
                return

    def update_camera_preview(self) -> None:
        """读取子控制器发布的缩略图，不重复打开物理摄像头。"""
        should_show = self.active_mode in ("sequence", "centre", "initial")
        if not should_show:
            self.preview_frame = None
            return

        try:
            mtime_ns = LIVE_FRAME_PATH.stat().st_mtime_ns
            if mtime_ns != self.preview_mtime_ns:
                image = cv2.imread(str(LIVE_FRAME_PATH), cv2.IMREAD_COLOR)
                if image is not None:
                    self.preview_frame = image
                    self.preview_mtime_ns = mtime_ns
        except OSError:
            pass

    def draw(self) -> np.ndarray:
        self.update_messages()
        busy = self.is_busy()
        signature: tuple[object, ...] = (
            self.state,
            self.detail,
            self.coordinates,
            tuple(self.output_lines[-11:]),
            busy,
            self.active_mode,
            self.preview_frame is None,
        )
        if self.base_frame_cache is not None and signature == self.base_frame_signature:
            frame = self.base_frame_cache.copy()
            if self.preview_frame is not None:
                frame[PREVIEW_Y:PREVIEW_Y + PREVIEW_H, PREVIEW_X:PREVIEW_X + PREVIEW_W] = cv2.resize(
                    self.preview_frame,
                    (PREVIEW_W, PREVIEW_H),
                    interpolation=cv2.INTER_AREA,
                )
            cv2.rectangle(
                frame,
                (PREVIEW_X, PREVIEW_Y),
                (PREVIEW_X + PREVIEW_W, PREVIEW_Y + PREVIEW_H),
                (115, 120, 135),
                2,
            )
            return frame

        frame = np.full((WIN_H, WIN_W, 3), (34, 37, 45), np.uint8)
        cv2.rectangle(frame, (0, 0), (WIN_W, 56), (56, 62, 75), -1)
        draw_button(frame, BUTTONS["sequence"], (35, 125, 40), not busy)
        draw_button(frame, BUTTONS["centre"], (130, 85, 25), not busy)
        draw_button(frame, BUTTONS["initial"], (120, 90, 25), not busy)
        draw_button(frame, BUTTONS["config"], (105, 65, 125), not busy)
        draw_button(frame, BUTTONS["stop"], (30, 35, 170), True)

        # 实时画面直接嵌入主窗口，保证在1024x768比赛屏幕上完整可见。
        frame[PREVIEW_Y:PREVIEW_Y + PREVIEW_H, PREVIEW_X:PREVIEW_X + PREVIEW_W] = (
            25, 27, 33
        )

        state_color = {
            "RUNNING": (80, 255, 80),
            "COMPLETE": (80, 255, 80),
            "ERROR": (80, 80, 255),
            "FAILED": (80, 80, 255),
            "CONFLICT": (80, 80, 255),
            "STOPPING": (80, 220, 255),
        }.get(self.state, (220, 220, 180))
        state_names = {
            "IDLE": "空闲",
            "STARTING": "正在启动",
            "RUNNING": "运行中",
            "COMPLETE": "已完成",
            "STOPPING": "正在停止",
            "STOPPED": "已停止",
            "BUSY": "忙碌",
            "CONFLICT": "程序冲突",
            "FAILED": "运行失败",
            "ERROR": "发生错误",
        }
        normal_text = (255, 255, 255) if not busy else (145, 145, 145)
        texts: list[tuple[str, tuple[int, int], int, tuple[int, int, int]]] = [
            ("电赛小球控制面板", (24, 13), 27, (245, 245, 245)),
            ("1  一键顺序", (40, 79), 24, normal_text),
            ("中目标 到 左目标 到 右目标", (40, 111), 16, normal_text),
            ("2  保持中点", (540, 79), 24, normal_text),
            ("持续运行，直到点击停止", (540, 111), 16, normal_text),
            ("3  初始锁定", (40, 161), 24, normal_text),
            ("稳定3秒后设零并锁定", (40, 193), 16, normal_text),
            ("4  坐标配置", (540, 161), 24, normal_text),
            ("轨道四角和三个目标横坐标", (540, 193), 16, normal_text),
            ("停止", (456, 241), 25, (255, 255, 255)),
            ("安全停止并请求电机回到零位", (378, 271), 14, (255, 255, 255)),
            (f"状态：{state_names.get(self.state, self.state)}", (20, 307), 21, state_color),
            (fit_text(self.detail, 16, 495), (20, 340), 16, (220, 220, 220)),
            (fit_text(self.coordinates, 14, 495), (20, 371), 14, (160, 220, 255)),
            ("运行摄像头画面", (540, 304), 18, (220, 220, 220)),
        ]
        if self.preview_frame is None:
            waiting = (
                "点击1、2或3后显示实时画面"
                if self.active_mode is None
                else "正在等待摄像头画面……"
            )
            texts.append((waiting, (590, 478), 18, (190, 190, 190)))
        y = 410
        for line in self.output_lines[-11:]:
            texts.append((fit_text(line, 13, 495), (20, y), 13, (165, 185, 165)))
            y += 21
        texts.append(("按键1到4：启动    X：停止    Q/Esc：关闭面板", (20, 657), 13, (150, 150, 150)))
        frame = render_texts(frame, texts)
        self.base_frame_cache = frame.copy()
        self.base_frame_signature = signature
        if self.preview_frame is not None:
            frame[PREVIEW_Y:PREVIEW_Y + PREVIEW_H, PREVIEW_X:PREVIEW_X + PREVIEW_W] = cv2.resize(
                self.preview_frame,
                (PREVIEW_W, PREVIEW_H),
                interpolation=cv2.INTER_AREA,
            )
        cv2.rectangle(
            frame,
            (PREVIEW_X, PREVIEW_Y),
            (PREVIEW_X + PREVIEW_W, PREVIEW_Y + PREVIEW_H),
            (115, 120, 135),
            2,
        )
        return frame

    def run(self) -> None:
        # GUI_NORMAL去掉OpenCV Qt工具栏，避免在1024x768屏幕底部被裁掉。
        cv2.namedWindow(
            WINDOW_NAME,
            cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL,
        )
        cv2.moveWindow(WINDOW_NAME, 5, 5)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        last_render = 0.0
        try:
            while not self.exit_requested:
                # OpenCV不会自动结束Python循环；必须检测标题栏叉号后的
                # WND_PROP_VISIBLE，否则下一次imshow会让窗口看似关不掉。
                try:
                    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
                if self.exit_after_stop and not self.is_busy():
                    break
                now = time.monotonic()
                if now - last_render >= UI_REFRESH_INTERVAL:
                    self.update_camera_preview()
                    cv2.imshow(WINDOW_NAME, self.draw())
                    last_render = time.monotonic()
                key = cv2.waitKey(20) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    if self.is_busy():
                        self.exit_after_stop = True
                        self.stop()
                        self.detail = "已请求停止；子程序结束后将自动关闭面板"
                    else:
                        break
                elif key in (ord("1"), ord("2"), ord("3"), ord("4")):
                    mode = {
                        "1": "sequence",
                        "2": "centre",
                        "3": "initial",
                        "4": "config",
                    }[chr(key)]
                    self.launch(mode)
                elif key in (ord("x"), ord("X")):
                    self.stop()
                time.sleep(0.01)
        finally:
            if self.is_busy():
                self.stop()
                # 覆盖5秒优雅停止、2秒强制结束和少量串口回零余量。
                deadline = time.monotonic() + 8.5
                while self.is_busy() and time.monotonic() < deadline:
                    time.sleep(0.05)
            cv2.destroyAllWindows()


def main() -> None:
    # 防止比赛时连续双击桌面图标打开两个面板，进而争用摄像头/串口。
    lock_file = PANEL_LOCK_PATH.open("w", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        try:
            subprocess.Popen(
                ["notify-send", "小球控制面板", "控制面板已经在运行，请勿重复打开"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass
        return
    lock_file.seek(0)
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    panel = ControlPanel()
    panel.run()


if __name__ == "__main__":
    main()
