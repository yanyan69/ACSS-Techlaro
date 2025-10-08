#!/usr/bin/env python3
"""
Fixed YOLO + Camera Detection Reference
- Auto-detects PiCamera2 or USB
- Converts 4-channel frame to 3-channel (for YOLO)
- Displays bounding boxes + FPS
"""

import time
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "my_model/my_model.pt"  # change this
CONF_THRESH = 0.5
CAM_SIZE = (640, 480)

# ===== CAMERA SETUP =====
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": CAM_SIZE})
    picam2.configure(config)
    picam2.start()
    use_picam = True
    print("📷 Using PiCamera2")
except Exception:
    cap = cv2.VideoCapture(0)
    cap.set(3, CAM_SIZE[0])
    cap.set(4, CAM_SIZE[1])
    use_picam = False
    print("🎥 Using USB camera")

# ===== MODEL =====
model = YOLO(MODEL_PATH)
labels = model.names
print("✅ YOLO model loaded.")

bbox_colors = {
    0: (68, 148, 228),
    1: (88, 159, 106),
    2: (164, 120, 87)
}

# ===== MAIN LOOP =====
print("Running live detection... Press 'q' to quit.")
frame_times = []

while True:
    t1 = time.time()
    frame = picam2.capture_array() if use_picam else cap.read()[1]
    if frame is None:
        print("⚠️ No frame captured.")
        break

    # 🔧 Fix: Convert 4-channel (RGBA) → 3-channel (BGR)
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

    # Run YOLO
    results = model(frame, verbose=False)
    detections = results[0].boxes

    for det in detections:
        conf = float(det.conf)
        if conf >= CONF_THRESH:
            cls_id = int(det.cls)
            label = labels[cls_id]
            color = bbox_colors.get(cls_id % 3, (0, 255, 0))
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = f"{label} {conf*100:.1f}%"
            (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - h - 6), (x1 + w, y1), color, -1)
            cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    # FPS overlay
    fps = 1.0 / (time.time() - t1)
    frame_times.append(fps)
    if len(frame_times) > 30:
        frame_times.pop(0)
    avg_fps = np.mean(frame_times)

    cv2.putText(frame, f"FPS: {avg_fps:.1f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.imshow("YOLO Live Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ===== CLEANUP =====
if use_picam:
    picam2.stop()
else:
    cap.release()
cv2.destroyAllWindows()
print("🛑 Detection stopped.")
