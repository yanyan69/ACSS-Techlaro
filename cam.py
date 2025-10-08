#!/usr/bin/env python3
"""
Minimal YOLO + Camera Detection Reference
- Works with PiCamera2 or USB camera
- Displays live preview with bounding boxes and FPS
"""

import time
import cv2
import numpy as np
from ultralytics import YOLO

# ===== SETTINGS =====
MODEL_PATH = "my_model/train/weights/best.pt"  # change to your model path
CONF_THRESH = 0.5
CAM_SIZE = (640, 480)

# Try to use PiCamera2; fallback to USB camera if not available
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

# Load YOLO model
model = YOLO(MODEL_PATH)
labels = model.names
print("✅ YOLO model loaded.")

# Bounding box colors (3 classes example)
bbox_colors = {
    0: (68, 148, 228),  # Blue
    1: (88, 159, 106),  # Green
    2: (164, 120, 87)   # Brown
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

    # Run YOLO detection
    results = model(frame, verbose=False)
    detections = results[0].boxes

    # Draw boxes
    for det in detections:
        conf = float(det.conf)
        if conf >= CONF_THRESH:
            cls_id = int(det.cls)
            label = labels[cls_id]
            color = bbox_colors.get(cls_id % 3, (0, 255, 0))
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            # Draw box + label
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label_text = f"{label} {conf*100:.1f}%"
            (w, h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - h - 6), (x1 + w, y1), color, -1)
            cv2.putText(frame, label_text, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    # Calculate FPS
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
