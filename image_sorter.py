#!/usr/bin/env python3
"""
ACSS Moisture Sorting GUI for Raspberry Pi
- Reads AS7263 values via Arduino on request (sensor mode)
- Captures camera frame and runs YOLO detection on request (camera mode)
- Computes moisture (sensor) or classifies via YOLO (camera)
- Classifies raw/standard/overcooked
- Sends servo sorting commands
- Controls motor (ON/OFF)
- Logs readings to CSV
- Simple serial log window for debugging
"""

import sys, time
import tkinter as tk
from tkinter import scrolledtext
import csv
import os
import cv2
import numpy as np

try:
    import serial
except Exception:
    serial = None
    print("ugh, pyserial isn’t even installed—go fix that with `pip install pyserial`, okay?")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("no PiCamera2? fine, i’ll just grab your default webcam instead. lazy setup 😑")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("yolo lib’s missing... guess we’re blind now. install it before you cry later 😒")

# ================= SETTINGS =================
SERIAL_PORT = '/dev/ttyUSB0'  # Change to 'COM6' for PC testing
SERIAL_BAUD = 9600  # Match Arduino
LOG_FILE = "data_gathering/data/moisture_log.csv"

# Thresholds for sensor-based analysis
OVERCOOKED_MAX = 5.9
STANDARD_MIN = 6.0
STANDARD_MAX = 7.0
RAW_MIN = 7.1

# Servo positions and commands
SERVO_CMDS = {"overcooked": ("L", 60), "standard": ("C", 90), "raw": ("R", 120)}

# Camera/YOLO settings
MODEL_PATH = "my_model/my_model.pt"   # your YOLOv11n path
FRAME_SIZE = (640, 480)
CONF_THRESH = 0.5

# YOLO label mapping (remove '-copra' suffix)
YOLO_LABEL_MAP = {
    'raw-copra': 'raw',
    'standard-copra': 'standard',
    'overcooked-copra': 'overcooked'
}

# Bounding box colors
BOUNDING_BOX_COLORS = {
    'overcooked-copra': (0, 0, 255),  # red
    'standard-copra': (0, 255, 0),    # green
    'raw-copra': (255, 0, 0)          # blue
}
# ======================================================

# Setup CSV file
os.makedirs("data_gathering/data", exist_ok=True)
try:
    with open(LOG_FILE, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Score/Moisture", "Class", "Method", "CH_R","CH_S","CH_T","CH_U","CH_V","CH_W"])
except FileExistsError:
    pass


class MoistureSorterGUI:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Moisture Sorter GUI")
        self.serial = None
        self.running = True

        # Camera init
        self.picam2 = None
        self.cap = None
        if PICAMERA2_AVAILABLE:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"size": FRAME_SIZE})
            self.picam2.configure(config)
            self.picam2.start()
            print("camera’s alive. try not to block the lens this time, yeah?")
        else:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, FRAME_SIZE[0])
            self.cap.set(4, FRAME_SIZE[1])
            print("ugh, using the default webcam... this better not lag.")

        # YOLO init
        self.model = None
        if ULTRALYTICS_AVAILABLE:
            print("loading YOLO... don’t panic if it takes a sec.")
            self.model = YOLO(MODEL_PATH)
            print("okay, YOLOv11n’s ready. don’t mess it up.")
        else:
            print("no YOLO? fine, we’ll pretend we can’t see anything.")

        # ---------- Layout ----------
        frm = tk.Frame(root)
        frm.pack(padx=10, pady=10)

        conn_frame = tk.LabelFrame(frm, text="Arduino Connection")
        conn_frame.pack(fill='x', padx=5, pady=5)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT} @ {SERIAL_BAUD}").pack(side='left', padx=4)
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).pack(side='left', padx=4)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).pack(side='left', padx=4)

        ctrl_frame = tk.LabelFrame(frm, text="Servo Control (Manual)")
        ctrl_frame.pack(fill='x', padx=5, pady=5)
        tk.Button(ctrl_frame, text="LEFT (Overcooked)", command=lambda: self.send_servo_cmd("overcooked")).pack(side='left', padx=10, pady=5)
        tk.Button(ctrl_frame, text="CENTER (Standard)", command=lambda: self.send_servo_cmd("standard")).pack(side='left', padx=10, pady=5)
        tk.Button(ctrl_frame, text="RIGHT (Raw)", command=lambda: self.send_servo_cmd("raw")).pack(side='left', padx=10, pady=5)

        motor_frame = tk.LabelFrame(frm, text="Motor Control")
        motor_frame.pack(fill='x', padx=5, pady=5)
        tk.Button(motor_frame, text="MOTOR ON", command=lambda: self.send_cmd("MOTOR,ON")).pack(side='left', padx=10, pady=5)
        tk.Button(motor_frame, text="MOTOR OFF", command=lambda: self.send_cmd("MOTOR,OFF")).pack(side='left', padx=10, pady=5)

        auto_frame = tk.LabelFrame(frm, text="Auto Sort")
        auto_frame.pack(fill='x', padx=5, pady=5)
        tk.Button(auto_frame, text="Sensor Reading & Sort (AS7263)", command=self.sensor_sort).pack(side='left', padx=10, pady=5)
        tk.Button(auto_frame, text="Camera Detect & Sort (YOLO)", command=self.camera_detect_and_sort).pack(side='left', padx=10, pady=5)
        tk.Button(auto_frame, text="Start Process (Motor + Camera Sort)", command=self.start_process).pack(side='left', padx=10, pady=5)

        log_frame = tk.LabelFrame(frm, text="Serial Log")
        log_frame.pack(fill='both', expand=True, padx=5, pady=5)
        self.log = scrolledtext.ScrolledText(log_frame, height=10, state='disabled', wrap='none')
        self.log.pack(fill='both', expand=True)

        # Auto open serial
        self.logmsg("Attempting to open serial automatically...")
        self.open_serial()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Logging ----------
    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert('end', full + "\n")
        self.log.see('end')
        self.log.config(state='disabled')
        print(full)

    # ---------- Serial ----------
    def open_serial(self):
        try:
            if serial is None:
                self.logmsg("ugh. pyserial’s missing. can’t open anything.")
                return
            if self.serial and self.serial.is_open:
                self.logmsg("serial’s already open, genius.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            time.sleep(3)  # Wait for Arduino reset
            self.logmsg(f"serial’s up at {SERIAL_PORT} @ {SERIAL_BAUD}. try not to unplug it mid-run again.")
        except Exception as e:
            self.logmsg(f"couldn’t open serial: {e}. nice job breaking it.")

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("serial link closed. we’re done here.")
        except Exception as e:
            self.logmsg(f"error closing serial: {e}. that’s on you.")

    def send_cmd(self, text):
        """Send command to Arduino with newline."""
        try:
            if not self.serial or not self.serial.is_open:
                self.logmsg(f"serial’s not open — can’t send '{text}', smart move.")
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            self.serial.write(full_cmd)
            self.serial.flush()
            print(f"> sent: {text.strip().upper()} ({full_cmd})")
        except Exception as e:
            print(f"send failed—{e}. try again maybe?")

    # ---------- Servo Commands ----------
    def send_servo_cmd(self, label):
        if label in SERVO_CMDS:
            cmd_letter, pos = SERVO_CMDS[label]
            cmd = f"SORT,{cmd_letter}"
            self.send_cmd(cmd)
            print(f"servo went {label.upper()} (pos={pos}) — clean movement, nice.")

    # ---------- Sensor Sort (AS7263) ----------
    def sensor_sort(self):
        print("reading AS7263... don’t breathe too hard, it’s sensitive.")
        self.send_cmd("REQ_AS")
        time.sleep(0.5)
        try:
            line = self.serial.readline().decode(errors='ignore').strip()
            if not line:
                print("no AS7263 response. it’s ignoring us again 🙄")
                return
            print(f"< got sensor data: {line}")
            ch_vals = [float(x) for x in line.split(",")]
            moisture = ch_vals[0]

            if moisture == 0:
                print("moisture = 0, which means… yeah, that’s broken.")
                return

            label = self.classify_moisture(moisture)
            print(f"moisture={moisture:.2f}% -> class={label}")

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(LOG_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, moisture, label, "sensor"] + ch_vals)

            self.send_servo_cmd(label)

        except Exception as e:
            print(f"sensor sort messed up: {e}")

    def classify_moisture(self, m):
        if m <= OVERCOOKED_MAX:
            return "overcooked"
        elif STANDARD_MIN <= m <= STANDARD_MAX:
            return "standard"
        elif m >= RAW_MIN:
            return "raw"
        else:
            return "unknown"

    # ---------- Camera Detect & Sort (YOLO) ----------
    def camera_detect_and_sort(self):
        print("capturing frame... smile for the camera~")
        if self.picam2:
            frame = self.picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif self.cap:
            ret, frame = self.cap.read()
            if not ret:
                print("camera failed. again.")
                return
        else:
            print("no camera found. what are you even running this on?")
            return

        if not self.model:
            print("yolo model missing, can’t detect squat.")
            return

        results = self.model(frame, verbose=False, imgsz=320, conf=CONF_THRESH)
        dets = results[0].boxes

        for det in dets:
            x1, y1, x2, y2 = map(int, det.xyxy[0])
            cls_id = int(det.cls.item())
            yolo_label = self.model.names[cls_id].lower()
            color = BOUNDING_BOX_COLORS.get(yolo_label, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{yolo_label} {det.conf.item():.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

        if len(dets) > 0:
            confs = [float(det.conf.item()) for det in dets]
            max_conf_idx = np.argmax(confs)
            det = dets[max_conf_idx]
            conf = confs[max_conf_idx]
            cls_id = int(det.cls.item())
            yolo_label = self.model.names[cls_id].lower()
            label = YOLO_LABEL_MAP.get(yolo_label, "unknown")

            if label == "unknown":
                print(f"unrecognized label: {yolo_label}. not touching that.")
                return

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(LOG_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, conf * 100, label, "camera", 0, 0, 0, 0, 0, 0])

            print(f"{label.upper()} detected ({conf*100:.2f}% sure). sending servo cmd.")
            self.send_servo_cmd(label)
        else:
            print("no detections. empty frame, boring as hell.")

    def start_process(self):
        print("starting full process... don’t blink.")
        self.send_cmd("MOTOR,ON")
        self.camera_detect_and_sort()
        self.send_cmd("MOTOR,OFF")
        print("done. smooth run, hopefully.")

    def on_close(self):
        print("shutting down—don’t cry, i’ll be back.")
        self.running = False
        self.close_serial()
        if self.picam2:
            self.picam2.stop()
        elif self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("all systems cleared. good job not breaking anything this time.")
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = MoistureSorterGUI(root)
    root.mainloop()