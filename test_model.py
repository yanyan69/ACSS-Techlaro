#!/usr/bin/env python3
"""
ACSS Model Tester (Camera + YOLO)
- Live camera preview + YOLO detection
- Bounding box colors for 3 copra classes:
    raw-copra       -> Blue
    standard-copra  -> Green
    overcooked-copra-> Red
- Press 'q' to exit
"""

import time
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("[!] No PiCamera detected... switching to default webcam")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("[!] YOLO library missing, detection disabled")

# ---- SETTINGS ----
YOLO_MODEL_PATH = "my_model/my_model.pt"  # change this
CAM_SIZE = (640, 480)
CONF_THRESH = 0.5

# Bounding box color codes (BGR)
BOX_COLORS = {
    'raw-copra': (255, 0, 123),       # Blue
    'standard-copra': (40, 167, 69),  # Green
    'overcooked-copra': (220, 53, 69) # Red
}

def main():
    # --- Camera Init ---
    if PICAMERA2_AVAILABLE:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": CAM_SIZE})
        picam2.configure(config)
        picam2.start()
        time.sleep(1)
        print("[+] Camera ready. Watching everything in real time...")
    else:
        # fallback to webcam
        cap = cv2.VideoCapture(0)
        cap.set(3, CAM_SIZE[0])
        cap.set(4, CAM_SIZE[1])
        print("[+] Using built-in webcam")

    # --- YOLO Init ---
    model = None
    if ULTRALYTICS_AVAILABLE:
        try:
            print("[+] Loading YOLO model...")
            model = YOLO(YOLO_MODEL_PATH)
            print("[+] Model loaded. Show me what you’ve got.")
        except Exception as e:
            print(f"[x] Couldn’t load YOLO: {e}")
            model = None

    print("[i] Press 'q' to quit.")

    last_frame = None
    last_detection_time = 0
    cooldown = 0.4  # seconds

    while True:
        # --- Capture frame ---
        if PICAMERA2_AVAILABLE:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                print("[x] Camera feed lost...")
                break

        current_time = time.time()

        # --- YOLO Detection ---
        if model and (current_time - last_detection_time >= cooldown):
            results = model(frame, verbose=False, imgsz=640, conf=CONF_THRESH, max_det=3)
            dets = results[0].boxes
            if dets:
                for det in dets:
                    conf = float(det.conf.item())
                    if conf < CONF_THRESH:
                        continue
                    class_id = int(det.cls.item())
                    name = model.names[class_id].lower()

                    xyxy = det.xyxy.cpu().numpy().astype(int).squeeze()
                    color = BOX_COLORS.get(name, (255, 255, 255))

                    cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                    label = f"{name} {int(conf*100)}%"
                    cv2.putText(frame, label, (xyxy[0], xyxy[1]-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                last_frame = frame.copy()
                last_detection_time = current_time

        # --- Display last frame with detection if no new detections yet ---
        display = frame if last_frame is None else last_frame
        cv2.imshow("ACSS Model Tester", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("[i] Exiting...")
            break

    # --- Cleanup ---
    if PICAMERA2_AVAILABLE:
        picam2.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()
    print("[✓] Done.")

if __name__ == "__main__":
    main()