import os
import sys
import time
import cv2
import numpy as np
import serial
import lgpio
from ultralytics import YOLO
from picamera2 import Picamera2

# --- settings ---
MODEL_PATH = "data/yolo11n.pt"
RESOLUTION = (640, 480)
CONF_THRESH = 0.5
IR1_PIN = 17
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

# check model
if not os.path.exists(MODEL_PATH):
    print("Model missing.")
    sys.exit(0)

# gpio
chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(chip, IR1_PIN)

# serial
arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)

# yolo
model = YOLO(MODEL_PATH, task="detect")
labels = model.names

# camera
picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"format": "RGB888", "size": RESOLUTION}))
picam.start()

# box colors
bbox_colors = [
    (164, 120, 87), (68, 148, 228), (93, 97, 209), (178, 182, 133),
    (88, 159, 106), (96, 202, 231), (159, 124, 168), (169, 162, 241),
    (98, 118, 150), (172, 176, 184)
]

# fps buffer
frame_rate_buffer = []
fps_avg_len = 200

print("System ready...")

while True:
    t_start = time.perf_counter()
    frame = picam.capture_array()
    if frame is None:
        print("Camera error.")
        break

    object_label = None

    # run only if IR hit
    if lgpio.gpio_read(chip, IR1_PIN) == 0:
        results = model(frame, verbose=False)
        detections = results[0].boxes

        for det in detections:
            xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
            xmin, ymin, xmax, ymax = xyxy
            classidx = int(det.cls.item())
            classname = labels[classidx]
            conf = det.conf.item()

            if conf > CONF_THRESH:
                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                cv2.putText(frame, f"{classname}: {int(conf*100)}%",
                            (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 2)
                object_label = classname
                break

        # send to arduino
        if object_label:
            if object_label == "apple":
                arduino.write(b"A")
            elif object_label == "toothbrush":
                arduino.write(b"B")
            elif object_label == "remote":
                arduino.write(b"C")
            print(f"Sent {object_label}")

    # fps calc
    t_stop = time.perf_counter()
    frame_rate = 1 / (t_stop - t_start)
    frame_rate_buffer.append(frame_rate)
    if len(frame_rate_buffer) > fps_avg_len:
        frame_rate_buffer.pop(0)
    avg_frame_rate = np.mean(frame_rate_buffer)

    # overlay fps
    cv2.putText(frame, f"FPS: {avg_frame_rate:.2f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("YOLO PiCamera Detection", frame)

    key = cv2.waitKey(5)
    if key in [ord("q"), ord("Q")]:
        break
    elif key in [ord("p"), ord("P")]:
        cv2.imwrite("capture.png", frame)
        print("Saved capture.png")

# cleanup
print(f"Avg FPS: {avg_frame_rate:.2f}")
picam.stop()
cv2.destroyAllWindows()
lgpio.gpiochip_close(chip)
arduino.close()