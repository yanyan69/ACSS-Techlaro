#!/usr/bin/env python3
"""
Fast YOLOv11n Live Detection (Raspberry Pi)
- Continuous detection (no cooldown)
- Optimized for low lag on PiCamera2
- Press 'q' to quit
"""

import time
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("[!] No PiCamera2 detected... using default webcam")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("[!] YOLO library missing, detection disabled")

# === SETTINGS ===
MODEL_PATH = "my_model/my_model.pt"   # your YOLOv11n path
FRAME_SIZE = (640, 480)
CONF_THRESH = 0.5

# Color presets
BOX_COLORS = {
    'raw-copra': (255, 0, 123),
    'standard-copra': (40, 167, 69),
    'overcooked-copra': (220, 53, 69)
}

def main():
    # === CAMERA INIT ===
    if PICAMERA2_AVAILABLE:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": FRAME_SIZE})
        picam2.configure(config)
        picam2.start()
        print("[+] PiCamera2 ready.")
    else:
        cap = cv2.VideoCapture(0)
        cap.set(3, FRAME_SIZE[0])
        cap.set(4, FRAME_SIZE[1])
        print("[+] Using default webcam")

    # === YOLO LOAD ===
    model = None
    if ULTRALYTICS_AVAILABLE:
        print("[+] Loading YOLO model...")
        model = YOLO(MODEL_PATH)
        print("[+] YOLOv11n ready to roll.")

    print("[i] Press 'q' to quit.")
    fps_time = time.time()

    while True:
        # === FRAME CAPTURE ===
        if PICAMERA2_AVAILABLE:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                print("[x] Camera error.")
                break

        # === YOLO DETECTION ===
        if model:
            results = model(frame, verbose=False, imgsz=320, conf=CONF_THRESH)
            dets = results[0].boxes

            for det in dets:
                conf = float(det.conf.item())
                cls_id = int(det.cls.item())
                label = model.names[cls_id].lower()
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
                color = BOX_COLORS.get(label, (255, 255, 255))

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label} {conf*100:.1f}%", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # === FPS COUNTER ===
        now = time.time()
        fps = 1 / (now - fps_time)
        fps_time = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # === DISPLAY ===
        cv2.imshow("YOLOv11n Live Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[i] Stopping...")
            break

    # === CLEANUP ===
    if PICAMERA2_AVAILABLE:
        picam2.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()
    print("[✓] Done.")

if __name__ == "__main__":
    main()