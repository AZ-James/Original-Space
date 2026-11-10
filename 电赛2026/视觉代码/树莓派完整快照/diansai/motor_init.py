#!/usr/bin/env python3
"""Motor lock/unlock panel.

Manual LOCK/UNLOCK is the currently verified function and is intentionally
preserved. Encoder polling is disabled while the installed RS485 motor-side
board cannot transmit responses; enable it after that board is replaced.
"""
import cv2, numpy as np, time, sys, math
from collections import deque

sys.path.insert(0, "/home/bg/diansai")
from zdt_motor import ZDTMotor

# Parameters
WIN_W, WIN_H = 400, 250
ENC_PORT = "/dev/ttyUSB0"
STABLE_DEG = 2.0
STABLE_SEC = 2.0
READ_INTERVAL = 0.1
ENC_RESOLUTION = 65536
READ_ENCODER = False

# Button geometry
BTN_LOCK   = (30, 170, 150, 50)
BTN_UNLOCK = (220, 170, 150, 50)

# State
locked = False
angle_samples = deque(maxlen=int(STABLE_SEC / READ_INTERVAL))
stable_count = 0
last_read_time = 0
current_angle = 0.0
status_msg = "Manual lock control"
read_ok = False

def enc_to_deg(enc):
    return enc / ENC_RESOLUTION * 360.0

def angle_diff(a, b):
    d = abs(a - b) % 360.0
    return d if d <= 180 else 360 - d

def is_stable():
    if len(angle_samples) < angle_samples.maxlen: return False
    arr = list(angle_samples)
    sin_sum = sum(math.sin(math.radians(a)) for a in arr)
    cos_sum = sum(math.cos(math.radians(a)) for a in arr)
    mean_angle = math.degrees(math.atan2(sin_sum, cos_sum)) % 360
    max_dev = max(angle_diff(a, mean_angle) for a in arr)
    return max_dev <= STABLE_DEG

def draw_button(frame, x, y, w, h, text, color, text_color=(255,255,255)):
    cv2.rectangle(frame, (x, y), (x+w, y+h), color, -1)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (255,255,255), 2)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    tx = x + (w - tw)//2; ty = y + (h + th)//2
    cv2.putText(frame, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

def mouse_callback(event, x, y, flags, param):
    global locked, stable_count, status_msg
    if event != cv2.EVENT_LBUTTONDOWN: return
    bx, by, bw, bh = BTN_LOCK
    if bx <= x <= bx+bw and by <= y <= by+bh:
        motor.enable(); locked = True; stable_count = 0
        status_msg = "MANUALLY LOCKED"
    bx, by, bw, bh = BTN_UNLOCK
    if bx <= x <= bx+bw and by <= y <= by+bh:
        motor.disable(); locked = False; angle_samples.clear()
        status_msg = "MANUALLY UNLOCKED"

# Init
motor = ZDTMotor(port=ENC_PORT)
print("[INFO] Motor connected")

# Start ENABLED to read encoder
motor.enable()
time.sleep(0.3)
locked = True  # enabled = locked
status_msg = "LOCKED - manual control" if not READ_ENCODER else "LOCKED - Reading encoder..."

cv2.namedWindow("Motor Auto-Lock", cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback("Motor Auto-Lock", mouse_callback)

while True:
    frame = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
    frame[:] = (40, 40, 50)

    # Read encoder (motor is enabled)
    now = time.time()
    if READ_ENCODER and now - last_read_time >= READ_INTERVAL:
        # Update even on failure so a missing response cannot hammer the port
        # on every GUI frame.
        last_read_time = now
        enc = motor.get_encoder()
        if enc is not None:
            current_angle = enc_to_deg(enc) % 360.0
            angle_samples.append(current_angle)
            read_ok = True
        else:
            read_ok = False

        # Check stability (only when we want auto-lock)
        if locked and read_ok and len(angle_samples) >= angle_samples.maxlen:
            if is_stable():
                stable_count += 1
                if stable_count >= angle_samples.maxlen:
                    status_msg = "AUTO-LOCKED! Angle stable."
            else:
                stable_count = max(0, stable_count - 1)

    # GUI
    cv2.rectangle(frame, (0, 0), (WIN_W, 35), (60, 60, 80), -1)
    cv2.putText(frame, "Motor Auto-Lock", (10, 25),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    angle_text = f"{current_angle:.1f}" if read_ok else "---"
    (tw, th), _ = cv2.getTextSize(angle_text, cv2.FONT_HERSHEY_DUPLEX, 1.8, 3)
    tx = (WIN_W - tw)//2
    cv2.putText(frame, angle_text, (tx, 90),
               cv2.FONT_HERSHEY_DUPLEX, 1.8, (0, 255, 255), 3)
    cv2.putText(frame, "deg", (tx+tw+5, 75),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

    lock_text = "LOCKED" if locked else "UNLOCKED"
    lock_col = (0, 255, 0) if locked else (0, 150, 255)
    cv2.putText(frame, lock_text, (10, 100),
               cv2.FONT_HERSHEY_SIMPLEX, 1.2, lock_col, 3)

    if not read_ok:
        encoder_text = "Encoder: disabled" if not READ_ENCODER else "Encoder: NO DATA"
        cv2.putText(frame, encoder_text, (10, 130),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    if locked and len(angle_samples) >= 3:
        pct = min(100, stable_count * 100 // angle_samples.maxlen)
        bar_x, bar_y, bar_w, bar_h = 10, 120, 200, 10
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+bar_w, bar_y+bar_h), (80,80,80), -1)
        fill_w = int(bar_w * pct / 100)
        bar_color = (0, 255, 0) if pct >= 90 else (0, 200, 255)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+fill_w, bar_y+bar_h), bar_color, -1)
        cv2.putText(frame, f"Stability: {pct}%", (10, 148),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

    cv2.putText(frame, status_msg[:50], (10, 160),
               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    btn_lc = (0, 150, 0) if locked else (0, 100, 0)
    btn_uc = (0, 0, 150) if not locked else (0, 0, 100)
    draw_button(frame, *BTN_LOCK, "LOCK", btn_lc)
    draw_button(frame, *BTN_UNLOCK, "UNLOCK", btn_uc)

    cv2.putText(frame, "Q=Quit  L=Lock  U=Unlock  Click buttons",
               (10, WIN_H-8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120,120,120), 1)

    cv2.imshow("Motor Auto-Lock", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or key == 27: break
    elif key == ord("l"):
        motor.enable(); locked = True; stable_count = 0
        status_msg = "MANUALLY LOCKED"
    elif key == ord("u"):
        motor.disable(); locked = False; angle_samples.clear()
        status_msg = "MANUALLY UNLOCKED"

motor.disable(); motor.close()
cv2.destroyAllWindows()
print("[INFO] Done.")
