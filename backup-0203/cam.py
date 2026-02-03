#!/usr/bin/env python3
"""
ACSS Camera + YOLO Test (Non-GUI)
- Captures frames from PiCamera2
- Runs YOLO detection and displays bounding boxes
- No serial, no sorting, no GUI
"""

import time
import cv2
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2

# ---------- SETTINGS ----------
YOLO_MODEL_PATH = "my_model/my_model.pt"
TRACKER_PATH = "bytetrack.yaml"
DISPLAY_SIZE = (640, 480)

def main():
    # Load YOLO model
    print(f"Loading YOLO model: {YOLO_MODEL_PATH}")
    model = YOLO(YOLO_MODEL_PATH)

    # Initialize camera
    print("Initializing PiCamera2...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": DISPLAY_SIZE})
    picam2.configure(config)
    picam2.start()
    print("Camera started. Press 'q' to quit.")

    frame_counter = 0
    fps_time = time.time()

    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            # Run YOLO detection
            results = model.track(frame, persist=True, tracker=TRACKER_PATH, conf=0.25, verbose=False)
            annotated = results[0].plot()

            # FPS counter
            frame_counter += 1
            if frame_counter >= 10:
                fps = 10 / (time.time() - fps_time)
                fps_time = time.time()
                frame_counter = 0
                cv2.putText(annotated, f"FPS: {fps:.1f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Show annotated frame
            cv2.imshow("ACSS Camera + YOLO", annotated)

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print("Camera stopped.")

if __name__ == "__main__":
    main()
