import os, sys, time, cv2, numpy as np, serial
from ultralytics import YOLO
from picamera2 import Picamera2

# User settings
MODEL_PATH = "data/yolo11n.pt"
RESOLUTION = (640, 480)
CONF_THRESH = 0.5
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

if not os.path.exists(MODEL_PATH):
    print("ERROR: Model not found.")
    sys.exit(0)

# Serial
arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)

# Model
model = YOLO(MODEL_PATH, task="detect")
labels = model.names

# Camera
picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"format": "RGB888", "size": RESOLUTION}))
picam.start()

bbox_colors = [(164,120,87),(68,148,228),(93,97,209),(178,182,133),
               (88,159,106),(96,202,231),(159,124,168),(169,162,241),
               (98,118,150),(172,176,184)]

frame_rate_buffer = []
fps_avg_len = 200

# --- Debounce control ---
last_sent = None
cooldown = 2.0  # seconds between sends
last_send_time = 0

print("System ready. Running detection...")

while True:
    t_start = time.perf_counter()
    frame = picam.capture_array()
    if frame is None:
        break

    object_label = None

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
                        (xmin, ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            object_label = classname
            break

    # --- Debounce send ---
    if object_label and (time.time() - last_send_time > cooldown):
        if object_label != last_sent:  # avoid same spam
            if object_label == "apple":
                arduino.write(b"A")
            elif object_label == "toothbrush":
                arduino.write(b"B")
            elif object_label == "remote":
                arduino.write(b"C")
            print(f"Sent {object_label} to Arduino")
            last_sent = object_label
            last_send_time = time.time()

    # FPS
    t_stop = time.perf_counter()
    frame_rate = 1 / (t_stop - t_start)
    frame_rate_buffer.append(frame_rate)
    if len(frame_rate_buffer) > fps_avg_len:
        frame_rate_buffer.pop(0)
    avg_frame_rate = np.mean(frame_rate_buffer)

    cv2.putText(frame, f"FPS: {avg_frame_rate:.2f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("YOLO PiCamera Detection", frame)
    key = cv2.waitKey(5)
    if key in [ord("q"), ord("Q")]:
        break

picam.stop()
cv2.destroyAllWindows()
arduino.close()