import os
import sys
import time
import cv2
import numpy as np
import serial
from ultralytics import YOLO
from picamera2 import Picamera2

# -------------------------------
# User settings
# -------------------------------
MODEL_PATH = "runs/detect/train/weights/best.pt"
RESOLUTION = (640, 480)
CONF_THRESH = 0.5
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
# -------------------------------

if not os.path.exists(MODEL_PATH):
    print("ERROR: Model not found.")
    sys.exit(0)

# Setup Serial
arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)  # allow Arduino reset

# Load model
model = YOLO(MODEL_PATH, task="detect")
labels = model.names

# Setup camera
picam = Picamera2()
picam.configure(
    picam.create_video_configuration(main={"format": "RGB888", "size": RESOLUTION})
)
picam.start()

# Colors for boxes
bbox_colors = [
    (164, 120, 87), (68, 148, 228), (93, 97, 209), (178, 182, 133),
    (88, 159, 106), (96, 202, 231), (159, 124, 168), (169, 162, 241),
    (98, 118, 150), (172, 176, 184)
]

# FPS averaging
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200

# Virtual line
frame_h = RESOLUTION[1]
line_y = frame_h // 2   # horizontal middle

print("System ready. Watching virtual line...")

while True:
    t_start = time.perf_counter()
    frame = picam.capture_array()
    if frame is None:
        print("Camera error. Exiting...")
        break

    results = model(frame, verbose=False)
    detections = results[0].boxes

    object_count = 0

    for det in detections:
        xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
        xmin, ymin, xmax, ymax = xyxy
        classidx = int(det.cls.item())
        classname = labels[classidx]
        conf = det.conf.item()

        if conf > CONF_THRESH:
            # Draw box
            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            label = f"{classname}: {int(conf*100)}%"
            cv2.putText(frame, label, (xmin, ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            object_count += 1

            # Compute object center
            cx = int((xmin + xmax) / 2)
            cy = int((ymin + ymax) / 2)
            cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

            # Check if object crosses line
            if abs(cy - line_y) < 5:  # within ~5px of line
                if classname == "ObjectA":
                    arduino.write(b"A")
                elif classname == "ObjectB":
                    arduino.write(b"B")
                elif classname == "ObjectC":
                    arduino.write(b"C")
                print(f"{classname} crossed line → sent to Arduino")

    # Draw the virtual line
    cv2.line(frame, (0, line_y), (RESOLUTION[0], line_y), (0, 255, 255), 2)

    # FPS calculation
    t_stop = time.perf_counter()
    frame_rate = 1 / (t_stop - t_start)
    frame_rate_buffer.append(frame_rate)
    if len(frame_rate_buffer) > fps_avg_len:
        frame_rate_buffer.pop(0)
    avg_frame_rate = np.mean(frame_rate_buffer)

    cv2.putText(frame, f"FPS: {avg_frame_rate:.2f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f"Objects: {object_count}", (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("YOLO Virtual Line Detection", frame)

    key = cv2.waitKey(5)
    if key in [ord("q"), ord("Q")]:
        break
    elif key in [ord("p"), ord("P")]:
        cv2.imwrite("capture.png", frame)
        print("Saved capture.png")

print(f"Average pipeline FPS: {avg_frame_rate:.2f}")
picam.stop()
cv2.destroyAllWindows()
arduino.close()