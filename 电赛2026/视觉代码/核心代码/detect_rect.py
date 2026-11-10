#!/usr/bin/env python3
"""
White Bar + Steel Ball Detection - USB Camera (MJPG 120fps)
Phase 1: 4 quad corners -> ENTER
Phase 2: 3 reference points -> ENTER
Phase 3: Auto-detect 1 steel ball + distances at TOP CENTER
"""
import cv2, numpy as np, time, sys, math, os

THRESH = 140; CAM_W = 640; CAM_H = 480; CAM_ID = 0
PHASE_QUAD = 1; PHASE_REF = 2; PHASE_DETECT = 3
phase = PHASE_QUAD
quad_pts = []; ref_pts = []
confirmed_quad = None; confirmed_ref = None
stated = "Phase 1: Click 4 quadrilateral corners, or press T to type"
tt_input = ""; tt_mode = False; dist_text = ""
MIN_STAIN_FRAC = 0.05

def mouse_callback(event, x, y, flags, param):
    global quad_pts, ref_pts, stated, phase
    if event != cv2.EVENT_LBUTTONDOWN: return
    if phase == PHASE_QUAD and len(quad_pts) < 4:
        quad_pts.append((x, y)); n = len(quad_pts)
        pts_str = ", ".join([f"({px},{py})" for px, py in quad_pts])
        stated = f"Quad {n}/4: ({x},{y}) | All: {pts_str}" + (" | ENTER" if n==4 else "")
    elif phase == PHASE_REF and len(ref_pts) < 3:
        ref_pts.append((x, y)); n = len(ref_pts)
        pts_str = ", ".join([f"({px},{py})" for px, py in ref_pts])
        stated = f"Ref {n}/3: ({x},{y}) | All: {pts_str}" + (" | ENTER" if n==3 else "")

def draw_quad(frame, pts, color, t=2):
    if len(pts) < 4: return
    for i in range(4): cv2.line(frame, pts[i], pts[(i+1)%4], color, t)
    for p in pts: cv2.circle(frame, p, 4, color, -1)

def draw_point_labels(display, pts, color, prefix=""):
    for i, p in enumerate(pts):
        cv2.circle(display, p, 6, color, -1)
        outline = (min(255,color[0]+50), min(255,color[1]+50), min(255,color[2]+50))
        cv2.circle(display, p, 6, outline, 2)
        label = f"{prefix}{i+1}:({p[0]},{p[1]})"
        tx, ty = p[0]+12, p[1]-12
        if tx + 80 > CAM_W: tx = p[0] - 80
        if ty < 15: ty = p[1] + 20
        cv2.putText(display, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 2)

# ====== Open USB camera with MJPG (compressed, low bandwidth) ======
# Set format via v4l2 first, then open with OpenCV
os.system(f"v4l2-ctl -d /dev/video{CAM_ID} --set-fmt-video=width={CAM_W},height={CAM_H},pixelformat=MJPG 2>/dev/null")
cap = cv2.VideoCapture(CAM_ID)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
time.sleep(1)
if not cap.isOpened():
    print("[ERROR] Cannot open USB camera"); sys.exit(1)
print(f"[INFO] USB Camera MJPG: {CAM_W}x{CAM_H}")

cv2.namedWindow("Rectangle + Stain Detection", cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback("Rectangle + Stain Detection", mouse_callback)
print("[INFO] Phase 1: Click 4 quadrilateral corners")

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        time.sleep(0.005); continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = frame.copy()

    cq = confirmed_quad if confirmed_quad is not None else quad_pts
    if len(cq) >= 1:
        col = (0, 255, 0) if confirmed_quad is not None else (0, 255, 255)
        draw_point_labels(display, cq, col, "Q")
        if len(cq) >= 2: cv2.polylines(display, [np.array(cq[:len(cq)], np.int32)], False, col, 1)
        if len(cq) == 4: draw_quad(display, cq, col, 2)

    cr = confirmed_ref if confirmed_ref is not None else ref_pts
    if len(cr) >= 1:
        col = (0, 128, 255) if confirmed_ref is not None else (255, 150, 0)
        draw_point_labels(display, cr, col, "R")

    dist_text = ""
    if (phase == PHASE_DETECT and
        confirmed_quad is not None and len(confirmed_quad) == 4 and
        confirmed_ref is not None and len(confirmed_ref) == 3):

        pts_arr = np.array(confirmed_quad, np.int32)
        mask = np.zeros((CAM_H, CAM_W), np.uint8)
        cv2.fillPoly(mask, [pts_arr], 255)
        masked_gray = cv2.bitwise_and(gray, gray, mask=mask)

        mrect = cv2.minAreaRect(np.array(confirmed_quad, dtype=np.float32))
        (cx, cy), (rw_mr, rh_mr), angle = mrect
        if rw_mr < rh_mr: rw_mr, rh_mr = rh_mr, rw_mr; angle += 90

        M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
        warped_full = cv2.warpAffine(gray, M, (CAM_W, CAM_H))
        warped_mask = cv2.warpAffine(mask, M, (CAM_W, CAM_H))
        warped_roi = cv2.bitwise_and(warped_full, warped_full, mask=warped_mask)

        corners = cv2.boxPoints(mrect)
        corners_w = cv2.transform(np.array([corners], dtype=np.float32), M)[0]
        x_min = max(0, int(np.min(corners_w[:,0]))); x_max = min(CAM_W-1, int(np.max(corners_w[:,0])))
        y_min = max(0, int(np.min(corners_w[:,1]))); y_max = min(CAM_H-1, int(np.max(corners_w[:,1])))

        if x_max - x_min >= 30 and y_max - y_min >= 5:
            roi_w = warped_roi[y_min:y_max, x_min:x_max]
            h_roi, w_roi = roi_w.shape
            y1, y2 = h_roi//3, 2*h_roi//3
            if y2 - y1 >= 2:
                strip_w = roi_w[y1:y2, :]; profile = strip_w.mean(axis=0)
                wnd = max(3, w_roi//20); local_std = np.zeros(w_roi)
                for i in range(wnd, w_roi-wnd): local_std[i] = profile[i-wnd:i+wnd+1].std()
                std_base = np.median(local_std[wnd:w_roi-wnd])
                std_thresh = std_base + max(2.0, local_std.std()*2.0)
                candidates = (local_std > std_thresh).astype(np.uint8)
                kernel = np.ones(max(3, w_roi//40), np.uint8)
                candidates = cv2.morphologyEx(candidates, cv2.MORPH_CLOSE, kernel)

                stain_indices = np.where(candidates > 0)[0]
                if len(stain_indices) > 0:
                    gaps = np.diff(stain_indices)
                    split_points = np.where(gaps > max(2, w_roi//30))[0]
                    starts = np.insert(split_points+1, 0, 0)
                    ends = np.append(split_points, len(stain_indices)-1)
                    best_start, best_end, best_w = None, None, 0
                    min_w = max(3, int(w_roi * MIN_STAIN_FRAC))
                    for s, e in zip(starts, ends):
                        w = stain_indices[e] - stain_indices[s]
                        if w > best_w and w >= min_w:
                            best_w = w; best_start = s; best_end = e
                    if best_start is not None:
                        a, b, tx = M[0,0], M[0,1], M[0,2]
                        c, d, ty = M[1,0], M[1,1], M[1,2]
                        det = a*d - b*c
                        if abs(det) > 1e-10:
                            Mi = np.float32([[d/det, -b/det, (b*ty-d*tx)/det], [-c/det, a/det, (c*tx-a*ty)/det]])
                            xs_left = x_min + stain_indices[best_start]
                            xs_right = x_min + stain_indices[best_end]
                            xs_center = (xs_left + xs_right)//2
                            pt_w = np.array([[[float(xs_center), float((y_min+y_max)/2)]]], dtype=np.float32)
                            pt_orig = cv2.transform(pt_w, Mi)[0]
                            sx = int(pt_orig[0,0]); sy = int(pt_orig[0,1])
                            dists = [math.hypot(sx-rx, sy-ry) for rx, ry in confirmed_ref]
                            dist_text = f"d1={dists[0]:.0f}px  d2={dists[1]:.0f}px  d3={dists[2]:.0f}px"
                            cv2.rectangle(display, (xs_left, y_min+y1), (xs_right, y_min+y2), (255,0,255), 2)
                            cv2.drawMarker(display, (sx, sy), (255,0,255), cv2.MARKER_CROSS, 15, 2)
                            cv2.circle(display, (sx, sy), 5, (255,0,255), -1)

    overlay = display.copy()
    cv2.rectangle(overlay, (0,0), (CAM_W,70), (0,0,0), cv2.FILLED)
    display = cv2.addWeighted(overlay, 0.5, display, 0.5, 0)
    if dist_text:
        (tw, th), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        tx = (CAM_W - tw) // 2
        cv2.putText(display, dist_text, (tx, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
    status_text = stated
    if tt_mode: status_text = f"INPUT: {tt_input}_"
    cv2.putText(display, status_text[:130], (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,0), 1)
    cv2.putText(display, f"Phase: {phase}/3  |  R=Reset  Q=Quit", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,200,200), 1)

    cv2.imshow("Rectangle + Stain Detection", display)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or key == 27: break
    elif tt_mode:
        if key == 13:
            try:
                parts = tt_input.replace(","," ").split(); coords = [int(p) for p in parts]
                if phase == PHASE_QUAD and len(coords) == 8:
                    quad_pts = [(coords[i], coords[i+1]) for i in range(0,8,2)]
                    stated = "Quad typed. ENTER to confirm"
                elif phase == PHASE_REF and len(coords) == 6:
                    ref_pts = [(coords[i], coords[i+1]) for i in range(0,6,2)]
                    stated = "Ref typed. ENTER to confirm"
                else: stated = f"Need {8 if phase==PHASE_QUAD else 6} numbers"
            except: stated = "Invalid numbers"
            tt_mode = False; tt_input = ""
        elif key == 8: tt_input = tt_input[:-1]
        elif 32 <= key < 127: tt_input += chr(key)
    elif key == ord("t"):
        tt_mode = True; tt_input = ""
        n = 8 if phase==PHASE_QUAD else 6
        stated = f"Type {n} numbers: " + ("x1 y1 x2 y2 x3 y3 x4 y4" if phase==PHASE_QUAD else "x1 y1 x2 y2 x3 y3")
    elif key == ord("r"):
        quad_pts = []; ref_pts = []; confirmed_quad = None; confirmed_ref = None
        phase = PHASE_QUAD; dist_text = ""
        stated = "Phase 1: Click 4 quadrilateral corners, or press T to type"
    elif key == 13:
        if phase == PHASE_QUAD and len(quad_pts) == 4:
            confirmed_quad = quad_pts[:]; phase = PHASE_REF
            stated = "Phase 2: Click 3 reference points, or press T to type"
        elif phase == PHASE_REF and len(ref_pts) == 3:
            confirmed_ref = ref_pts[:]; phase = PHASE_DETECT
            stated = "Phase 3: DETECTING... R=Reset  Q=Quit"

cap.release(); cv2.destroyAllWindows()
print("[INFO] Done.")
