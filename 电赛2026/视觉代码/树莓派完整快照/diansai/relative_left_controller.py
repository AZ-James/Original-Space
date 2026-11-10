#!/usr/bin/env python3
"""钢球相对轨道光流补偿控制器（当前为实验调试版）。

============================== 先看这里 ==============================

本文件解决的核心问题不是“找不到钢球”，而是：

1. 电机转动时，轨道会靠近或远离镜头；
2. 摄像头存在畸变，因此轨道移动会让一个没有滚动的钢球也产生像素位移；
3. 如果直接使用钢球的绝对 x 坐标，会把轨道投影变化误判为钢球滚动；
4. 本程序使用轨道上的多个特征点估计轨道在钢球位置处的画面位移，
   然后从钢球原始位移中减掉轨道位移：

       钢球相对轨道位移 = 钢球画面位移 - 同位置轨道画面位移

程序当前只执行“移动到左目标并用非零角度保持”的实验流程。
它不是最终比赛程序。最近实测表明，轨道阻尼很小，脉冲后回到 0 度仍可能
产生很大的往复振荡，因此下一步应改成限速、预测制动和连续非零保持。

安全说明：

- MOTOR_ARMED 默认为 False，直接按 F5 只进行启动视觉检查，不会打开电机串口；
- 只有你明确把 MOTOR_ARMED 改成 True 后，程序才会控制 /dev/ttyUSB0；
- direction = +1 表示希望钢球向右，direction = -1 表示希望钢球向左；
- 电机异常、视觉丢失、进入边缘区或未到达目标时，finally 会发绝对 0 度；
- 只有左目标确认到达并通过 5 秒保持检查后，才保留 FINAL_HOLD_ANGLE。

调试建议：先在 relative_pulse() 内给 relative_total、track_dx、ball_dx
设置断点，确认光流补偿正确，再允许电机运行。
"""
import os
import time

import cv2
import numpy as np

from ball_controller import CAMERA_ID, CAM_H, CAM_W, FixedBallDetector, PositionFilter
from pulse_controller import read_measurement, wait_for_settle
from vision_calibration import QUAD, TARGET_POINTS
from zdt_motor import ZDTMotor, angle_to_pulses

# ---------------------------------------------------------------------
# 总开关：这是最重要的安全开关。
# False：只检查摄像头、钢球检测和稳定状态，然后退出，不打开电机。
# True ：允许下面的程序打开 /dev/ttyUSB0 并实际驱动电机。
# 在 VS Code 中调试时，建议先保持 False。
# ---------------------------------------------------------------------
MOTOR_ARMED = False

# TARGET_POINTS 的顺序来自 vision_calibration.json：中、左、右。
# 索引 1 是左目标；当前标定值大约为 x=211。
TARGET_X = float(TARGET_POINTS[1][0])  # left

# 最终落点允许误差。只有回到水平状态并稳定后才用这个容差判断目标，
# 因为倾斜状态下绝对像素坐标会受到轨道深度和镜头畸变影响。
TARGET_TOLERANCE = 15.0

# 当前锁定情况下，左向静摩擦较大，因此实验从 11 度开始搜索。
# 这两个值只是实验搜索范围，不是“角度到位移”的精确模型。
INITIAL_ANGLE = 11.0
MAX_ANGLE = 15.0

# 光流补偿后的累计相对位移达到 1 px，并且速度方向一致，才认为钢球启动。
RELATIVE_TRIGGER_PX = 1.0

# 到左目标后不回到 0 度，而是用一个很小的负角度保持。
# 如果保持阶段发生明显漂移，程序仍会判定失败并回到 0 度。
FINAL_HOLD_ANGLE = -2.0


def make_feature_mask(shape, ball_x):
    """生成只允许在轨道上选择光流特征的掩膜。

    QUAD 是 vision_calibration.json 中给出的轨道四边形。
    钢球附近左右各 35 px 被排除，因为钢球自身会移动；如果把钢球的反光
    当成轨道特征，估计出的 track_dx 就会被钢球运动污染。
    """
    mask = np.zeros(shape, np.uint8)
    # 先把轨道四边形内部置为 255，其余背景保持 0。
    cv2.fillPoly(mask, [QUAD], 255)
    # 再挖掉钢球附近的竖条区域，只跟踪真正固定在轨道上的纹理、边缘和螺钉。
    mask[:, max(0, int(ball_x - 35)):min(shape[1], int(ball_x + 35))] = 0
    return mask


def begin_frame(camera, detector, filt):
    """取得一次可靠的钢球测量，并初始化轨道光流特征点。

    最多尝试 30 帧。至少需要 8 个轨道特征，否则本次动作不允许开始。
    返回：上一帧灰度图、钢球测量、要跟踪的轨道特征点。
    """
    for _ in range(30):
        ok, frame = camera.read()
        if not ok:
            continue
        measurement = detector.detect(frame)
        filt.update(measurement, time.perf_counter())
        if measurement is not None and filt.x is not None and not filt.lost_frames:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            points = cv2.goodFeaturesToTrack(
                gray,
                mask=make_feature_mask(gray.shape, measurement.x),
                maxCorners=100,       # 最多跟踪 100 个角点
                qualityLevel=0.01,
                minDistance=6,        # 特征之间至少相隔 6 px
                blockSize=7,
            )
            if points is not None and len(points) >= 8:
                return gray, measurement, points
    raise RuntimeError("cannot initialize relative tracker")


def relative_pulse(camera, detector, filt, motor, direction, angle):
    """执行一个带光流补偿的有限倾角脉冲。

    参数：
      direction：+1 希望向右，-1 希望向左。
      angle：本次允许达到的电机绝对角度幅值。

    控制流程：
      1. 在电机动作前选择轨道特征；
      2. 电机以 1 RPM 慢速接近目标角度；
      3. 每帧分别计算钢球位移 ball_dx 和轨道位移 track_dx；
      4. 累加 relative_total += ball_dx - track_dx；
      5. 相对位移和实际速度都表明钢球已启动时，立即快速回到 0 度；
      6. 无论正常、超时还是异常，finally 都负责发送回平命令。

    注意：当前“启动后立即回平”会激发轨道上的低阻尼振荡，这是本实验版的
    主要已知缺点，不能把它当成最终控制策略。
    """
    prev_gray, measurement, points = begin_frame(camera, detector, filt)
    prev_ball = float(measurement.x)
    relative_total = 0.0  # 光流补偿后的钢球累计相对位移
    raw_total = 0.0       # 未补偿的钢球像素累计位移，仅用于日志对照
    max_speed = 0.0
    reason = "timeout"
    # 绝对位置命令，1 RPM 慢速接近；硬件安全限位仍由底层程序保证。
    motor.move_absolute_signed(direction * abs(angle_to_pulses(angle)), 1, 10)

    # 1 RPM = 每秒 6 度。angle/6 是理论转动时间，再增加 1.5 秒观察窗口。
    deadline = time.monotonic() + angle / 6.0 + 1.5
    try:
        while time.monotonic() < deadline:
            ok, frame = camera.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            measurement = detector.detect(frame)
            filt.update(measurement, time.perf_counter())
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                prev_gray,
                gray,
                points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
            )
            if next_points is None or status is None:
                reason = "flow_lost"
                break
            good = status.reshape(-1) == 1
            old = points.reshape(-1, 2)[good]
            new = next_points.reshape(-1, 2)[good]
            if len(old) < 6:
                reason = "flow_lost"
                break
            if measurement is None:
                prev_gray = gray
                points = new.reshape(-1, 1, 2).astype(np.float32)
                continue

            # 每个轨道特征从 old 移动到 new，dx 是它在画面中的横向位移。
            dx = new[:, 0] - old[:, 0]

            # 镜头畸变造成的位移随横坐标变化：右侧通常比左侧变化更明显。
            # 因此不用单一的“全局平均位移”，而拟合：dx = slope*x + intercept。
            design = np.column_stack((old[:, 0], np.ones(len(old))))
            slope, intercept = np.linalg.lstsq(design, dx, rcond=None)[0]

            # 第一次拟合后，用 MAD（中位数绝对偏差）排除错误光流点，
            # 再用剩余内点拟合一次，防止背景反光或局部误匹配影响结果。
            residual = dx - (slope * old[:, 0] + intercept)
            centre = np.median(residual)
            mad = np.median(np.abs(residual - centre))
            inliers = np.abs(residual - centre) <= max(0.4, 4.0 * mad)
            if int(inliers.sum()) >= 5:
                slope, intercept = np.linalg.lstsq(design[inliers], dx[inliers], rcond=None)[0]
                new = new[inliers]
            # 把拟合模型代入“钢球上一帧的横坐标”，估计钢球所在位置处
            # 轨道本身应该产生多少像素位移。
            track_dx = float(slope * prev_ball + intercept)
            ball_x = float(measurement.x)
            ball_dx = ball_x - prev_ball
            raw_total += ball_dx

            # 关键公式：从钢球画面位移中减掉轨道投影位移。
            relative_total += ball_dx - track_dx
            prev_ball = ball_x
            prev_gray = gray
            points = new.reshape(-1, 1, 2).astype(np.float32)
            max_speed = max(max_speed, abs(float(filt.velocity)))

            # 独立安全保护1：进入排除边缘区域就回平。
            if filt.x is not None and not 105 <= filt.x <= 535:
                reason = "edge_guard"
                break
            # 独立安全保护2：滤波速度达到 80 px/s 就提前回平。
            if abs(float(filt.velocity)) >= 80:
                reason = "speed_guard"
                break
            # 把速度投影到目标方向。向左时 direction=-1，负速度乘以-1后为正。
            directed_speed = direction * float(filt.velocity)

            # 只有“相对轨道位移”和“钢球速度”同时同向，才判定真正启动。
            # 这样能排除约 1 px 的光流累计漂移造成的误触发。
            if (
                direction * relative_total >= RELATIVE_TRIGGER_PX
                and directed_speed >= 8.0
            ):
                reason = "relative_motion"
                break
    finally:
        # 本函数只负责一次脉冲；脉冲结束无条件快速回平。
        motor.move_absolute_signed(0, 80, 200)
    print(
        f"FLOW_TRIGGER reason={reason} raw={raw_total:+.2f}px "
        f"relative={relative_total:+.2f}px max_speed={max_speed:.1f}",
        flush=True,
    )
    return reason, max_speed


# =====================================================================
# 主程序：摄像头初始化
# =====================================================================
# v4l2-ctl 请求 MJPG 和较高帧率。实际处理速度约 45 FPS，具体取决于树莓派负载。
os.system(
    f"v4l2-ctl -d /dev/video{CAMERA_ID} "
    f"--set-fmt-video=width={CAM_W},height={CAM_H},pixelformat=MJPG "
    f"--set-parm=60 2>/dev/null"
)
camera = cv2.VideoCapture(CAMERA_ID)
# 再通过 OpenCV 设置一次参数，避免驱动忽略其中一部分 v4l2 设置。
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
camera.set(cv2.CAP_PROP_FPS, 60)
detector = FixedBallDetector()
filt = PositionFilter()
motor = None
# holding=True 只表示：已经到达左目标并确认非零保持稳定。
# 只有这种情况下 finally 才不会发送 0 度。
holding = False
try:
    # 启动时先观察钢球是否真正稳定。这个阶段完全不打开电机串口。
    x, settled, _ = wait_for_settle(camera, detector, filt, 12.0)
    if x is None or not settled:
        raise RuntimeError("ball not settled")
    print(f"START_LEFT x={x:.1f} target={TARGET_X:.1f}", flush=True)

    # 默认安全互锁：直接按 F5 会在这里退出。
    # 你确认摄像头、坐标和断点都正确后，才把文件顶部 MOTOR_ARMED 改成 True。
    if not MOTOR_ARMED:
        print(
            "VISION CHECK ONLY: MOTOR_ARMED=False，未打开电机。"
            "确认后把文件顶部 MOTOR_ARMED 改成 True。",
            flush=True,
        )
        raise SystemExit(0)

    # =================================================================
    # 电机启动与原保持角释放
    # =================================================================
    motor = ZDTMotor(port="/dev/ttyUSB0")
    motor.enable()
    # 当前驱动板没有ACK；实测 enable 后立刻发送位置命令可能被忽略，
    # 所以必须等待至少0.5秒，这里使用0.6秒。
    time.sleep(0.6)

    # 上一个实验在右目标保留了 +2 度保持；折返前明确释放到 0 度。
    motor.move_absolute_signed(0, 80, 200)
    time.sleep(0.4)

    # =================================================================
    # 脉冲—稳定—重新测量循环
    # =================================================================
    angle = INITIAL_ANGLE
    for attempt in range(12):
        # error<0 表示目标在钢球左侧，error>0 表示目标在右侧。
        error = TARGET_X - x
        if abs(error) <= TARGET_TOLERANCE:
            break
        direction = 1 if error > 0 else -1
        before = x
        print(
            f"LEFT_ATTEMPT {attempt+1}/12 x={x:.1f} error={error:+.1f} "
            f"angle={direction*angle:+.1f}",
            flush=True,
        )
        reason, pulse_speed = relative_pulse(
            camera, detector, filt, motor, direction, angle
        )

        # 光流丢失或接近边缘属于不可继续的故障，直接交给 finally 回平。
        if reason in ("flow_lost", "edge_guard"):
            raise RuntimeError(reason)

        # relative_pulse 已回到0度。现在必须等待真正稳定，再使用绝对像素坐标
        # 判断落点；倾斜过程中的绝对 x 坐标不能用于目标判断。
        x, settled, coast_speed = wait_for_settle(camera, detector, filt, 12.0)
        if x is None or not settled:
            raise RuntimeError("ball did not settle")
        # progress>0 表示最终落点确实朝目标方向移动；负值表示反弹或振荡后
        # 反而停到了错误方向。
        progress = direction * (x - before)
        print(
            f"LEFT_RESULT x={x:.1f} progress={progress:+.1f} "
            f"max_speed={max(pulse_speed,coast_speed):.1f}",
            flush=True,
        )
        if progress < 2.0:
            # 永久位移不足时才提高下一次角度。最高不超过 MAX_ANGLE。
            if angle >= MAX_ANGLE:
                raise RuntimeError("no progress at maximum pulse angle")
            angle = min(MAX_ANGLE, angle + 1.0)

        # 80 px/s 是提前回平阈值；若回平后的完整轨迹仍超过120 px/s，
        # 说明系统能量过大，本轮必须停止。
        if reason == "speed_guard" and max(pulse_speed, coast_speed) >= 120:
            raise RuntimeError("severe overspeed")
    else:
        raise RuntimeError("attempt budget exhausted")

    # 循环退出后再次独立检查目标，避免未来修改 break 条件时误判成功。
    if abs(TARGET_X - x) > TARGET_TOLERANCE:
        raise RuntimeError(f"left target not reached: x={x}")
    print(f"LEFT_TARGET level_x={x:.1f} error={TARGET_X-x:+.1f}", flush=True)

    # =================================================================
    # 左目标非零保持
    # =================================================================
    # 到目标后不回0度，而施加很小的 -2 度保持。注意这里的绝对画面坐标
    # 会因轨道倾斜略有变化，所以主要检查5秒内是否继续漂移以及是否超速。
    motor.move_absolute_signed(angle_to_pulses(FINAL_HOLD_ANGLE), 20, 80)
    time.sleep(0.3)
    hold_values = []
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        value = read_measurement(camera, detector, filt)
        if value is None:
            continue
        _, ball_x, velocity = value
        hold_values.append(ball_x)
        # 左目标保护范围和保持阶段速度保护。
        if not 170 <= ball_x <= 250 or abs(velocity) > 25:
            raise RuntimeError("left hold safety guard")
    if not hold_values or max(hold_values) - min(hold_values) > 5.0:
        raise RuntimeError("left hold drifted")
    print(
        f"LEFT_HOLD angle={FINAL_HOLD_ANGLE:+.1f} "
        f"range=({min(hold_values):.1f},{max(hold_values):.1f})",
        flush=True,
    )
    # 只有完整通过5秒检查，才允许 finally 保留非零角度。
    holding = True
finally:
    # =================================================================
    # 最终安全清理
    # =================================================================
    if motor is not None:
        if not holding:
            # 所有失败、异常、Ctrl+C和未到达目标的情况都回到绝对0度。
            motor.move_absolute_signed(0, 80, 200)
            time.sleep(0.4)
        # 关闭串口不会解锁电机。holding=True 时，电机会继续锁在-2度。
        motor.close()
    camera.release()
    print("HELD_LEFT_NONZERO" if holding else "RETURNED_ZERO", flush=True)
