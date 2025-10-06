#!/usr/bin/env python3
"""
YOLO Live Detection for Raspberry Pi (Picamera2)
Reze-style 😘
"""

import time
import cv2

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("Aww~ no camera? Guess we’ll just have to *imagine* the frames 💋")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("Heh, no YOLO? Looks like someone forgot their toys 💣")

MODEL_PATH = "my_model/my_model.pt"
CAM_SIZE = (640, 480)
CONF_THRESH = 0.5
MAX_DET = 10

def main():
    if not PICAMERA2_AVAILABLE:
        print("Tch... no camera. Can’t play without eyes, darling 👀")
        return

    # Setup the camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": CAM_SIZE})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    # Load YOLO
    yolo = None
    if ULTRALYTICS_AVAILABLE:
        try:
            print("Hehe~ give me a second... loading something deadly 🔪")
            yolo = YOLO(MODEL_PATH)
            print("Mm~ YOLO model’s ready. Let’s hunt 💞")
        except Exception as e:
            print("Ehh... something blew up loading YOLO 💣:", e)

    print("Camera’s live~ show me something interesting 💋 (press 'q' to stop)")
    prev_time = time.time()

    while True:
        frame = picam2.capture_array()

        # Run YOLO
        if yolo:
            results = yolo.predict(source=frame, imgsz=640, conf=CONF_THRESH, max_det=MAX_DET, verbose=False)
            frame = results[0].plot()

        # FPS display
        fps = 1.0 / (time.time() - prev_time)
        prev_time = time.time()
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("💣 RezeCam – YOLO Live Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Aww~ leaving so soon? Fine... see you next mission 💔")
            break

    picam2.stop()
    cv2.destroyAllWindows()
    print("💤 All systems down... goodnight, bombshell.")

if __name__ == "__main__":
    main()