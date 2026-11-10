
## 2026-08-01 当前推荐控制方案

当前推荐入口为 `pulse_controller.py`。它不再使用固定“角度×时间→位移”模型，而是采用轻量反馈：5.5°起步、未克服静摩擦时每次增加0.5°、最高7°；检测到运动立即回零，钢球稳定后重新测量实际落点。环境较稳定，因此只使用简单中位数/MAD滤噪和3秒稳定窗口，不进行复杂在线辨识。

```bash
python3 pulse_controller.py --target centre --max-pulses 8
python3 pulse_controller.py --target left --max-pulses 10
python3 pulse_controller.py --target right --max-pulses 10
```

轨道和目标坐标统一在 `vision_calibration.json` 中调整。完整实验依据、安全约束和文件说明见 `CONTROL_UPDATE_2026-08-01.md`。旧版已备份为 `pulse_controller_before_adaptive.py`，仅供回看。

注意：电机 `enable()` 后第一条位置命令前必须等待至少0.5秒；当前程序等待0.6秒。±30°硬限位、边缘保护、视觉丢失回零和超速中止始终有效。
