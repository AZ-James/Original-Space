import cv2

c = cv2.VideoCapture(0)
frame = None
for _ in range(30):
    ok, candidate = c.read()
    if ok:
        frame = candidate
c.release()
if frame is None:
    raise RuntimeError("camera read failed")
cv2.imwrite("current_frame.jpg", frame)
