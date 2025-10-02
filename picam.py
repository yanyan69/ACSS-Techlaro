#!/usr/bin/env python3
"""
Simple Camera + YOLO Test (No GUI)
- Captures frames from PiCamera2
- Runs YOLO inference on frames (if model available)
- Shows result with cv2.imshow
Press 'q' to exit
"""

import time
import cv2

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("picamera2 not available")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("ultralytics not available")

YOLO_MODEL_PATH = "/my_model/train/weights/best.pt" #change
CAM_SIZE = (640, 480)

def main():
    if not PICAMERA2_AVAILABLE:
        print("Camera not available. Exiting.")
        return

    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": CAM_SIZE})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    # Load YOLO if available
    yolo = None
    if ULTRALYTICS_AVAILABLE:
        try:
            print("Loading YOLO model...")
            yolo = YOLO(YOLO_MODEL_PATH)
            print("YOLO model loaded.")
        except Exception as e:
            print("YOLO load failed:", e)

    print("Press 'q' to quit.")
    while True:
        frame = picam2.capture_array()  # BGR numpy frame

        if yolo:
            results = yolo.predict(source=frame, imgsz=640, conf=0.25, max_det=5, verbose=False)
            frame = results[0].plot()

        cv2.imshow("Camera + YOLO Test", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()
    print("Camera stopped. Exiting.")

if __name__ == "__main__":
    main()