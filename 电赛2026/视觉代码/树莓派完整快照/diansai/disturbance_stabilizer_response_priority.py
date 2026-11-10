#!/usr/bin/env python3
"""低延迟、透视校正的中心抗扰控制器。

设计重点：
1. 电机命令速度从原来的 20 rpm 提高到 120 rpm，解决机械动作慢半拍。
2. 不再使用三帧中值滤波，而采用 alpha-beta 状态估计，减少约 1~2 帧延迟。
3. 利用轨道四点做透视变换，控制量使用矫正后的轨道坐标，而非原始像素 X。
4. 利用当前速度预测 85 ms 后的位置，提前制动，而不是撞边后才反向。
5. 唯一角度位置限幅是机械安全限制 +/-30 度；反向只要求先经过零，不额外等待。
"""
import os
import time
from pathlib import Path

import cv2
import numpy as np

from ball_controller import CAMERA_ID, CAM_H, CAM_W, FixedBallDetector
from vision_calibration import QUAD, TARGET_POINTS, TRACK_ENDPOINTS
from zdt_motor import ZDTMotor, angle_to_pulses


# 中心目标直接来自可视化界面保存的 vision_calibration.json。
# TARGET_POINTS 的固定顺序是 centre、left、right。
TARGET_RAW_X = float(TARGET_POINTS[0][0])
TARGET_RAW_Y = float(TARGET_POINTS[0][1])
# 0 表示持续运行，直到人工发送 Ctrl+C/SIGINT；停止时 finally 会快速回零。
RUN_SECONDS = 0.0

CONTROL_INTERVAL = 0.030
PREDICTION_HORIZON_SECONDS = 0.085
ALPHA = 0.82
BETA = 0.32
MAX_RECTIFIED_JUMP = 42.0
MAX_ESTIMATED_SPEED = 800.0

MOTOR_SPEED_RPM = 120
MOTOR_ACCEL = 220
LOW_MOTOR_SPEED_RPM = 90
LOW_MOTOR_ACCEL = 180
MAX_ANGLE_DEG = 30.0
NORMAL_STEP_DEG = 8.0
HIGH_SPEED_STEP_DEG = 12.0
HIGH_SPEED_THRESHOLD = 150.0

# 接近目标后使用滞环进入柔和模式；退出阈值更大，避免模式来回切换。
LOW_MODE_ENTER_SPEED = 18.0
LOW_MODE_ENTER_ERROR = 7.0
LOW_MODE_EXIT_SPEED = 28.0
LOW_MODE_EXIT_ERROR = 12.0
LOW_CONTROL_INTERVAL = 0.040
LOW_ANGLE_STEP_DEG = 4.0
LOW_COMMAND_THRESHOLD_DEG = 0.4

# 左右并不对称：左侧修正更强；右侧因成像放大而降低比例增益。
KP_BALL_LEFT = 0.075
KP_BALL_RIGHT = 0.060
KD_MOVING_LEFT = 0.055
KD_MOVING_RIGHT = 0.045
POSITION_DEADBAND = 5.0
VELOCITY_DEADBAND = 8.0
# 静摩擦补偿不再突然加入 2.5 度，而是确认卡住后逐渐增加。
STUCK_CONFIRM_SECONDS = 0.35
STUCK_SPEED_THRESHOLD = 25.0
BREAKAWAY_START_DEG = 0.5
BREAKAWAY_MAX_DEG = 8.0
BREAKAWAY_RAMP_DEG_PER_SECOND = 3.0

# 透视矫正后轨道宽度固定为 500，接近端点时提前最大反向制动。
RECTIFIED_WIDTH = 500.0
EDGE_GUARD_INNER = 55.0
EDGE_PREDICT_LIMIT = 35.0

# 控制器独占摄像头，通过低频缩略图文件给桌面控制面板提供画面。
# 这样不会由第二个进程重复打开 /dev/video0，也尽量不影响闭环时延。
LIVE_FRAME_PATH = Path(os.environ.get("DIANSAI_LIVE_FRAME", "/tmp/diansai_live.jpg"))
LIVE_FRAME_INTERVAL = 0.12
_last_live_frame_time = 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


# 标定点顺序：左上、右上、右下、左下。
_dst = np.array(
    [[0.0, 0.0], [RECTIFIED_WIDTH, 0.0],
     [RECTIFIED_WIDTH, 100.0], [0.0, 100.0]],
    dtype=np.float32,
)
_perspective = cv2.getPerspectiveTransform(
    TRACK_ENDPOINTS.astype(np.float32), _dst
)


def rectify_point(x, y):
    """把受梯形透视影响的像素坐标转换为统一轨道坐标。"""
    point = np.array([[[float(x), float(y)]]], dtype=np.float32)
    return float(cv2.perspectiveTransform(point, _perspective)[0, 0, 0])


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


TARGET_X = rectify_point(TARGET_RAW_X, TARGET_RAW_Y)


class FastState:
    """低延迟 alpha-beta 位置/速度估计，并拒绝单帧识别突变。"""

    def __init__(self):
        self.x = None
        self.raw_x = None
        self.velocity = 0.0
        self.last_time = None
        self.lost_frames = 0

    def update(self, measurement, now):
        if measurement is None:
            self.lost_frames += 1
            self.velocity *= 0.92
            return False

        measured = rectify_point(measurement.x, measurement.y)
        if self.x is None or self.last_time is None:
            self.x = measured
            self.raw_x = float(measurement.x)
            self.velocity = 0.0
            self.last_time = now
            self.lost_frames = 0
            return True

        dt = clamp(now - self.last_time, 0.008, 0.10)
        predicted = self.x + self.velocity * dt
        residual = measured - predicted
        if abs(residual) > MAX_RECTIFIED_JUMP:
            self.lost_frames += 1
            self.velocity *= 0.90
            self.last_time = now
            return False

        self.x = predicted + ALPHA * residual
        self.velocity = clamp(
            self.velocity + BETA * residual / dt,
            -MAX_ESTIMATED_SPEED,
            MAX_ESTIMATED_SPEED,
        )
        self.raw_x = float(measurement.x)
        self.last_time = now
        self.lost_frames = 0
        return True


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
state = FastState()
motor = None
command_angle = 0.0
last_control = 0.0
last_report = 0.0
low_mode = False
stuck_since = None

try:
    # 电机串口打开前先连续取得可靠位置，避免无视觉时误动作。
    deadline = time.monotonic() + 8.0
    good_frames = 0
    while time.monotonic() < deadline and good_frames < 8:
        ok, frame = camera.read()
        if not ok or frame is None:
            continue
        measurement = detector.detect(frame)
        publish_live_frame(frame, measurement)
        if state.update(measurement, time.perf_counter()):
            good_frames += 1
    if state.x is None or good_frames < 8:
        raise RuntimeError("ball not detected; motor was not opened")

    motor = ZDTMotor(port="/dev/ttyUSB0")
    motor.enable()
    time.sleep(0.35)
    motor.move_absolute_signed(0, MOTOR_SPEED_RPM, MOTOR_ACCEL)
    time.sleep(0.15)
    print(
        f"PREDICTIVE_ACTIVE target={TARGET_X:.1f} "
        f"start={state.x:.1f} raw={state.raw_x:.1f} motor={MOTOR_SPEED_RPM}rpm",
        flush=True,
    )

    started = time.monotonic()
    while RUN_SECONDS <= 0.0 or time.monotonic() - started < RUN_SECONDS:
        ok, frame = camera.read()
        if not ok or frame is None:
            continue
        now_perf = time.perf_counter()
        measurement = detector.detect(frame)
        publish_live_frame(frame, measurement)
        state.update(measurement, now_perf)
        now = time.monotonic()

        active_interval = LOW_CONTROL_INTERVAL if low_mode else CONTROL_INTERVAL
        if now - last_control < active_interval:
            continue
        last_control = now

        if state.x is None or state.lost_frames >= 4:
            desired = 0.0
            predicted_x = state.x
            status = "VISION_LOST"
        else:
            velocity = float(state.velocity)
            predicted_x = state.x + velocity * PREDICTION_HORIZON_SECONDS
            error = TARGET_X - predicted_x

            # 带滞环的高/低速模式切换。明显扰动立即退出柔和模式；
            # 只有真正接近目标且低速后才重新进入，避免临界点抖动。
            if low_mode:
                if (
                    abs(velocity) > LOW_MODE_EXIT_SPEED
                    or abs(error) > LOW_MODE_EXIT_ERROR
                ):
                    low_mode = False
            elif (
                abs(velocity) < LOW_MODE_ENTER_SPEED
                and abs(error) < LOW_MODE_ENTER_ERROR
            ):
                low_mode = True

            if state.x < EDGE_GUARD_INNER or predicted_x < EDGE_PREDICT_LIMIT:
                desired = +MAX_ANGLE_DEG
                status = "LEFT_EDGE_BRAKE"
            elif (
                state.x > RECTIFIED_WIDTH - EDGE_GUARD_INNER
                or predicted_x > RECTIFIED_WIDTH - EDGE_PREDICT_LIMIT
            ):
                desired = -MAX_ANGLE_DEG
                status = "RIGHT_EDGE_BRAKE"
            else:
                kp = KP_BALL_LEFT if predicted_x < TARGET_X else KP_BALL_RIGHT
                kd = KD_MOVING_LEFT if velocity < 0.0 else KD_MOVING_RIGHT
                if low_mode:
                    # 只削弱中心附近的细小动作；高速扰动参数完全不变。
                    kp *= 0.80
                    kd *= 0.85
                position_term = 0.0 if abs(error) <= POSITION_DEADBAND else kp * error
                damping_term = 0.0 if abs(velocity) <= VELOCITY_DEADBAND else -kd * velocity
                desired = position_term + damping_term

                # 确认持续静止后才逐渐增加静摩擦补偿，避免原先的突然踢动。
                if abs(error) > 8.0 and abs(velocity) < STUCK_SPEED_THRESHOLD:
                    if stuck_since is None:
                        stuck_since = now
                    stuck_time = now - stuck_since
                    if stuck_time >= STUCK_CONFIRM_SECONDS:
                        breakaway = min(
                            BREAKAWAY_MAX_DEG,
                            BREAKAWAY_START_DEG
                            + (stuck_time - STUCK_CONFIRM_SECONDS)
                            * BREAKAWAY_RAMP_DEG_PER_SECOND,
                        )
                        desired += breakaway if error > 0.0 else -breakaway
                else:
                    stuck_since = None

                desired = clamp(desired, -MAX_ANGLE_DEG, MAX_ANGLE_DEG)
                if abs(error) <= POSITION_DEADBAND and abs(velocity) < VELOCITY_DEADBAND:
                    desired = 0.0
                status = "SOFT_HOLD" if low_mode else "PREDICTIVE_PD"

                # 反向时只要求快速经过零位，不再额外停留 0.1 秒。
                if command_angle * desired < 0.0 and abs(command_angle) > 0.8:
                    desired = 0.0
                    status = "FAST_ZERO_CROSS"

        if low_mode:
            step_limit = LOW_ANGLE_STEP_DEG
        elif state.x is not None and abs(state.velocity) >= HIGH_SPEED_THRESHOLD:
            step_limit = HIGH_SPEED_STEP_DEG
        else:
            step_limit = NORMAL_STEP_DEG
        new_angle = clamp(
            command_angle + clamp(desired - command_angle, -step_limit, step_limit),
            -MAX_ANGLE_DEG,
            MAX_ANGLE_DEG,
        )
        command_threshold = LOW_COMMAND_THRESHOLD_DEG if low_mode else 0.20
        if abs(new_angle - command_angle) >= command_threshold:
            command_angle = new_angle
            motor_speed = LOW_MOTOR_SPEED_RPM if low_mode else MOTOR_SPEED_RPM
            motor_accel = LOW_MOTOR_ACCEL if low_mode else MOTOR_ACCEL
            motor.move_absolute_signed(
                angle_to_pulses(command_angle), motor_speed, motor_accel
            )

        if now - last_report >= 0.25:
            print(
                f"LIVE t={now-started:5.1f}s raw={state.raw_x} x={state.x} "
                f"pred={predicted_x} v={state.velocity:+.1f} "
                f"angle={command_angle:+.2f} mode={'LOW' if low_mode else 'HIGH'} "
                f"status={status}",
                flush=True,
            )
            last_report = now
finally:
    if motor is not None:
        motor.move_absolute_signed(0, MOTOR_SPEED_RPM, MOTOR_ACCEL)
        time.sleep(0.30)
        motor.close()
    camera.release()
    print("PREDICTIVE_STOPPED_HOLDING_ZERO", flush=True)
