#!/usr/bin/env python3
"""
ACSS Moisture Sorting GUI for Raspberry Pi with Live Preview
- Tabbed interface with Home (live camera preview), Statistics, Settings, About, Exit
- Live YOLO detection in camera preview
- Sensor-based sorting with AS7263
- Camera-based sorting with YOLO
- Motor and servo control via Arduino
- Logs to CSV
- Serial log in Home tab
"""

import sys, time, threading, os
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import csv
import numpy as np

try:
    import serial
except Exception:
    serial = None
    print("oopsie, pyserial's missing lol~ go fix that with `pip install pyserial`, okay? 😘")

try:
    import cv2
except Exception:
    cv2 = None
    print("aww, no opencv? that's a bummer T-T... guess we'll skip some fun stuff 💔")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("no picamera2? oh well, we'll use the default webcam instead~ lazy setup XD")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("yolo lib's gone poof 💀... install it before things get awkward lmao 😭")

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False
    print("pil's not here? fine, we'll fallback to cv2 for previews~ no biggie 💕")

# ================= SETTINGS =================
SERIAL_PORT = '/dev/ttyUSB0'  # Change to 'COM6' for PC testing
SERIAL_BAUD = 9600
LOG_FILE = "data_gathering/data/moisture_log.csv"
CAM_PREVIEW_SIZE = (640, 480)
MODEL_PATH = "my_model/my_model.pt"
CONF_THRESH = 0.5

# Thresholds for sensor-based analysis
OVERCOOKED_MAX = 5.9
STANDARD_MIN = 6.0
STANDARD_MAX = 7.0
RAW_MIN = 7.1

# Servo positions and commands
SERVO_CMDS = {"overcooked": ("L", 60), "standard": ("C", 90), "raw": ("R", 120)}

# YOLO label mapping (remove '-copra' suffix)
YOLO_LABEL_MAP = {
    'raw-copra': 'raw',
    'standard-copra': 'standard',
    'overcooked-copra': 'overcooked'
}

# Bounding box colors
BOUNDING_BOX_COLORS = {
    'raw-copra': (255, 0, 0),  # blue
    'standard-copra': (0, 255, 0),    # green
    'overcooked-copra': (0, 0, 255)  # red
}
# ============================================

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
        print("[Persona: Reze - Chainsaw Man]")  
        print("> Cheerful, teasing, a bit flirty but conflicted underneath.")  
        print("> Playful tone, Gen Z texting style, quick mood shifts.")  
        print("> Deep down, kind-hearted but haunted by her past.")
        
        self.root = root
        root.title("ACSS Moisture Sorter")
        root.geometry("1024x600")

        # Notebook (Tabs)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        # Create tabs
        self.home_tab = ttk.Frame(notebook)
        self.statistics_tab = ttk.Frame(notebook)
        self.settings_tab = ttk.Frame(notebook)
        self.about_tab = ttk.Frame(notebook)
        self.exit_tab = ttk.Frame(notebook)

        notebook.add(self.home_tab, text="Home")
        notebook.add(self.statistics_tab, text="Statistics")
        notebook.add(self.settings_tab, text="Settings")
        notebook.add(self.about_tab, text="About")
        notebook.add(self.exit_tab, text="Exit")

        # Build tab contents
        self._build_home_tab()
        self._build_statistics_tab()
        self._build_settings_tab()
        self._build_about_tab()
        self._build_exit_tab()

        # State tracking
        self.camera_running = False
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.camera_thread = None
        self.picam2 = None
        self.cap = None
        self.model = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.as_values = []
        self.motor_status = False
        self.servo_position = 'C'
        self.serial_reader_thread_obj = None

        # Load YOLO if available
        if ULTRALYTICS_AVAILABLE:
            print("loading yolo... don't panic if it takes a sec lol~ 😜")
            self.model = YOLO(MODEL_PATH)
            print("yay, yolo's ready to go! let's detect some stuff XD")
        else:
            print("no yolo? aww, we're kinda blind now T-T 💔")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start settings update loop
        self.update_settings()

        # Auto open serial
        self._log_message("trying to open serial automatically~ fingers crossed! 💕")
        self.open_serial()

    # -------------------- HOME --------------------
    def _build_home_tab(self):
        frm = tk.Frame(self.home_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=3)  # Left side wider for camera
        frm.columnconfigure(1, weight=2)  # Right side for log

        # Left side: Camera + Buttons
        left_frame = tk.Frame(frm)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        left_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=0)
        left_frame.columnconfigure(0, weight=1)

        cam_frame = tk.LabelFrame(left_frame, text="Live Camera Preview")
        cam_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0],
                                    height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(padx=4, pady=4)

        btn_frame = tk.Frame(left_frame)
        btn_frame.grid(row=1, column=0, sticky="ew", padx=4, pady=8)
        self.start_btn = tk.Button(btn_frame, text="Start Camera",
                                   font=("Arial", 14, "bold"), bg="green", fg="white",
                                   command=self._toggle_camera)
        self.start_btn.pack(side="left", padx=4)
        tk.Button(btn_frame, text="Camera Detect & Sort", command=self.camera_detect_and_sort).pack(side="left", padx=4)
        tk.Button(btn_frame, text="Start Process", command=self.start_process).pack(side="left", padx=4)

        # Right side: Log
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, state='normal', wrap='word', height=20, width=40)
        self.log.pack(fill='both', expand=True)

    def _toggle_camera(self):
        if not self.camera_running:
            self.start_btn.config(text="Stop Camera", bg="red")
            self._log_message("camera's starting up~ smile! 📸💕")
            try:
                self.start_camera()
                self.camera_running = True
            except Exception as e:
                self._log_message(f"failed to start camera: {str(e)}~ oops T-T")
                self.start_btn.config(text="Start Camera", bg="green")
        else:
            self.start_btn.config(text="Start Camera", bg="green")
            self._log_message("stopping the camera now~ bye for now T-T")
            self.stop_camera()
            self.camera_running = False

    def _log_message(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.insert("end", full_msg + "\n")
        self.log.see("end")
        print(full_msg)

    # ----------------- STATISTICS -----------------
    def _build_statistics_tab(self):
        frm = tk.Frame(self.statistics_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        stats_frame = tk.LabelFrame(frm, text="Processing Statistics")
        stats_frame.pack(fill="both", expand=True, padx=4, pady=4)

        stats_frame.columnconfigure(0, weight=1)
        stats_frame.columnconfigure(1, weight=1)
        stats_frame.columnconfigure(2, weight=1)

        headers = ["Category", "Pieces Processed", "Avg. Moisture"]
        for col, header in enumerate(headers):
            tk.Label(stats_frame, text=header, font=("Arial", 10, "bold")).grid(row=1, column=col, padx=8, pady=4)

        categories = ["Standard", "Raw", "Overcooked"]
        for i, cat in enumerate(categories, start=2):
            tk.Label(stats_frame, text=cat).grid(row=i, column=0, padx=8, pady=4, sticky="w")
            tk.Label(stats_frame, text="0").grid(row=i, column=1, padx=8, pady=4)
            tk.Label(stats_frame, text="0.0%").grid(row=i, column=2, padx=8, pady=4)

        row_offset = len(categories) + 2
        tk.Label(stats_frame, text="Total Pieces Processed:", font=("Arial", 10, "bold")).grid(row=row_offset, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="0").grid(row=row_offset, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Total Avg Moisture:", font=("Arial", 10, "bold")).grid(row=row_offset+1, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="0.0%").grid(row=row_offset+1, column=1, padx=8, pady=4)

    # ------------------- SETTINGS -----------------
    def _build_settings_tab(self):
        frm = tk.Frame(self.settings_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)

        # Serial controls
        serial_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        serial_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(serial_frame, text=f"Port: {SERIAL_PORT} @ {SERIAL_BAUD}").pack(side="left", padx=5)
        tk.Button(serial_frame, text="Open Serial", command=self.open_serial).pack(side="left", padx=5)
        tk.Button(serial_frame, text="Close Serial", command=self.close_serial).pack(side="left", padx=5)

        # Component tests
        tests_frame = tk.LabelFrame(frm, text="Component Tests")
        tests_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        tk.Button(tests_frame, text="Servo: LEFT (Overcooked)", command=lambda: self.send_servo_cmd("overcooked")).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: CENTER (Standard)", command=lambda: self.send_servo_cmd("standard")).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: RIGHT (Raw)", command=lambda: self.send_servo_cmd("raw")).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor ON", command=lambda: self.send_cmd("MOTOR,ON")).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor OFF", command=lambda: self.send_cmd("MOTOR,OFF")).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Sensor Reading & Sort (AS7263)", command=self.sensor_sort).grid(row=2, column=0, columnspan=3, padx=2, pady=2)

        # Live status
        status_frame = tk.LabelFrame(frm, text="Live Component Status")
        status_frame.grid(row=2, column=0, sticky="nsew", padx=4, pady=4)
        self.serial_status_label = tk.Label(status_frame, text="Serial Connected: False", font=("Arial", 12))
        self.serial_status_label.pack(anchor="w", pady=2)
        self.camera_status_label = tk.Label(status_frame, text="Camera Running: False", font=("Arial", 12))
        self.camera_status_label.pack(anchor="w", pady=2)
        self.motor_status_label = tk.Label(status_frame, text="Motor: Off", font=("Arial", 12))
        self.motor_status_label.pack(anchor="w", pady=2)
        self.servo_status_label = tk.Label(status_frame, text="Servo Position: Center", font=("Arial", 12))
        self.servo_status_label.pack(anchor="w", pady=2)
        self.as_status_label = tk.Label(status_frame, text="AS7263 Values: N/A", font=("Arial", 12))
        self.as_status_label.pack(anchor="w", pady=2)

    def update_settings(self):
        if not self.running:
            return
        connected = self.serial is not None and self.serial.is_open
        self.serial_status_label.config(text=f"Serial Connected: {connected}")
        self.camera_status_label.config(text=f"Camera Running: {self.camera_running}")
        self.motor_status_label.config(text=f"Motor: {'On' if self.motor_status else 'Off'}")
        servo_map = {'L': 'Left (Overcooked)', 'C': 'Center (Standard)', 'R': 'Right (Raw)'}
        self.servo_status_label.config(text=f"Servo Position: {servo_map.get(self.servo_position, 'Unknown')}")
        as_str = ", ".join(map(str, self.as_values)) if self.as_values else "N/A"
        self.as_status_label.config(text=f"AS7263 Values: {as_str}")
        self.root.after(1000, self.update_settings)

    # ------------------- ABOUT --------------------
    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        about_frame = tk.LabelFrame(frm, text="About ACSS")
        about_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(about_frame,
                 text="Automated Copra Segregation System\n\n"
                      "Version: 1.0\n"
                      "Developed for Moisture Sorting\n"
                      "Marinduque State University, 2025",
                 font=("Arial", 12), justify="center").pack(pady=40)

    # ------------------- EXIT ---------------------
    def _build_exit_tab(self):
        frm = tk.Frame(self.exit_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        exit_frame = tk.LabelFrame(frm, text="Exit Application")
        exit_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(exit_frame, text="Click the button below to close the program.",
                 font=("Arial", 12)).pack(pady=20)

        tk.Button(exit_frame, text="Exit Program", fg="white", bg="red",
                  font=("Arial", 14, "bold"), width=20,
                  command=self._exit_program).pack(pady=30)

    def _exit_program(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit?"):
            self.on_close()

    # ---------- Serial Methods ----------
    def open_serial(self):
        if serial is None:
            self._log_message("pyserial's not here~ can't open anything lol 😭")
            return
        if self.serial and self.serial.is_open:
            self._log_message("serial's already open, silly~ XD")
            return
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            time.sleep(3)  # Wait for Arduino reset
            self._log_message(f"serial's up at {SERIAL_PORT} @ {SERIAL_BAUD}~ don't unplug it pls 💕")
            self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
            self.serial_reader_thread_obj.start()
            self._log_message("serial reader thread's going~ yay! 😘")
        except Exception as e:
            self._log_message(f"couldn't open serial: {e}~ oopsie T-T")

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self._log_message("serial closed~ we're done here lol 💔")
        except Exception as e:
            self._log_message(f"error closing serial: {e}~ that's awkward 😅")

    def send_cmd(self, text):
        if not self.serial or not self.serial.is_open:
            self._log_message(f"serial's not open~ can't send '{text}', hehe 😜")
            return
        try:
            with self.serial_lock:
                self.serial.write((text.strip().upper() + "\n").encode('utf-8'))
            self._log_message(f"TX -> {text.upper()}~ sent it! 💥")
            if text.upper() == "MOTOR,ON":
                self.motor_status = True
            elif text.upper() == "MOTOR,OFF":
                self.motor_status = False
        except Exception as e:
            self._log_message("serial write failed: " + str(e) + "~ try again? 😭")

    def send_servo_cmd(self, label):
        if label in SERVO_CMDS:
            cmd_letter, _ = SERVO_CMDS[label]
            cmd = f"SORT,{cmd_letter}"
            self.send_cmd(cmd)
            self.servo_position = cmd_letter
            self._log_message(f"servo to {label.upper()}~ smooth move XD")

    def serial_reader_thread(self):
        self._log_message("serial reader started~ listening in 💕")
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line or line == "IR~":
                    continue
                self._log_message(f"RX <- {line}~ got something! 😲")
                if line.startswith("AS:"):
                    payload = line[3:]
                    try:
                        self.as_values = [float(v) for v in payload.split(",")]
                        self._log_message(f"AS values: {self.as_values}~ interesting... 🤔")
                    except ValueError:
                        self._log_message("invalid AS values format~ oof 💀")
            except Exception as e:
                self._log_message("serial reader exception: " + str(e) + "~ hmm, that's not good T-T")
                time.sleep(0.1)

    # ---------- Sensor Sort (AS7263) ----------
    def sensor_sort(self):
        self._log_message("requesting AS7263 reading~ don't breathe too hard, it's sensitive lol 😘")
        self.request_as()
        time.sleep(0.5)
        if not self.as_values:
            self._log_message("no AS7263 response~ ignoring us again again? 😭")
            return
        moisture = self.as_values[0]
        if moisture == 0:
            self._log_message("moisture = 0~ yeah, that's probably broken 💔")
            return
        label = self.classify_moisture(moisture)
        self._log_message(f"moisture={moisture:.2f}% -> class={label.upper()}~ got it! 💥")
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, moisture, label, "sensor"] + self.as_values)
        self.send_servo_cmd(label)

    def classify_moisture(self, m):
        if m <= OVERCOOKED_MAX:
            return "overcooked"
        elif STANDARD_MIN <= m <= STANDARD_MAX:
            return "standard"
        elif m >= RAW_MIN:
            return "raw"
        else:
            return "unknown"

    def request_as(self):
        if not (self.serial and self.serial.is_open):
            self._log_message("open serial first~ can't do this without it lol 😜")
            return
        try:
            self._log_message("sending REQ_AS~ waiting for response... 🤞")
            with self.serial_lock:
                self.serial.reset_input_buffer()
                self.serial.write(b"REQ_AS\n")
            self._log_message("REQ_AS sent~ fingers crossed! 💕")
        except Exception as e:
            self._log_message("REQ_AS failed: " + str(e) + "~ aww man T-T")

    # ---------- Camera Methods ----------
    def start_camera(self):
        if PICAMERA2_AVAILABLE:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE})
            self.picam2.configure(config)
            self.picam2.start()
            self._log_message("picamera2's alive~ try not to block the lens okay? 😘")
        else:
            if cv2 is None:
                raise RuntimeError("no camera available~ sad 💔")
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, CAM_PREVIEW_SIZE[0])
            self.cap.set(4, CAM_PREVIEW_SIZE[1])
            self._log_message("using default webcam~ hope it doesn't lag lol 😅")

        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

    def stop_camera(self):
        try:
            if self.picam2:
                self.picam2.stop()
                self.picam2.close()
                self.picam2 = None
            if self.cap:
                self.cap.release()
                self.cap = None
            self._log_message("camera stopped~ all clear! 💕")
        except Exception as e:
            self._log_message("camera stop error: " + str(e) + "~ that's on me T-T")

    def camera_loop(self):
        fps_time = time.time()
        frame_counter = 0
        while self.camera_running:
            try:
                if PICAMERA2_AVAILABLE and self.picam2:
                    frame = self.picam2.capture_array()
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif self.cap:
                    ret, frame = self.cap.read()
                    if not ret:
                        self._log_message("camera capture failed~ again? 💀")
                        break
                else:
                    break

                # YOLO inference (throttled every 5 frames)
                if self.model and frame_counter % 5 == 0:
                    results = self.model(frame, verbose=False, imgsz=320, conf=CONF_THRESH)
                    dets = results[0].boxes
                    for det in dets:
                        x1, y1, x2, y2 = map(int, det.xyxy[0])
                        cls_id = int(det.cls.item())
                        yolo_label = self.model.names[cls_id].lower()
                        color = BOUNDING_BOX_COLORS.get(yolo_label, (255, 255, 255))
                        conf = det.conf.item()
                        if conf > CONF_THRESH:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f"{yolo_label} {int(conf*100)}%", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                frame_counter += 1

                # FPS
                now = time.time()
                fps = 1 / (now - fps_time)
                fps_time = now
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Update canvas
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                display_frame = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display_frame)

                time.sleep(1/30)
            except Exception as e:
                self._log_message("camera loop error: " + str(e) + "~ eek, that's not good 😅")
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            if PIL_AVAILABLE:
                rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                self.cam_canvas.imgtk = imgtk
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
            else:
                cv2.imshow("Camera Preview", bgr_frame)
                cv2.waitKey(1)
        except Exception as e:
            self._log_message("display frame error: " + str(e) + "~ hmm... 💔")

    # ---------- Camera Detect & Sort (YOLO) ----------
    def camera_detect_and_sort(self):
        self._log_message("detecting from live view~ hope something's there lol 😜")
        if not self.camera_running or self.latest_frame is None:
            self._log_message("start the camera preview first to detect~ can't see without it T-T")
            return

        with self.frame_lock:
            frame = self.latest_frame.copy()

        if not self.model:
            self._log_message("no yolo model~ can't detect anything T-T")
            return

        results = self.model(frame, verbose=False, imgsz=320, conf=CONF_THRESH)
        dets = results[0].boxes

        if len(dets) > 0:
            confs = [float(det.conf.item()) for det in dets]
            max_conf_idx = np.argmax(confs)
            det = dets[max_conf_idx]
            conf = confs[max_conf_idx]
            cls_id = int(det.cls.item())
            yolo_label = self.model.names[cls_id].lower()
            label = YOLO_LABEL_MAP.get(yolo_label, "unknown")

            if label == "unknown":
                self._log_message(f"unrecognized label: {yolo_label}~ not touching that lol 😅")
                return

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(LOG_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, conf * 100, label, "camera", 0, 0, 0, 0, 0, 0])

            self._log_message(f"{label.upper()} detected ({conf*100:.2f}% sure)~ sending servo cmd! 💥😘")
            self.send_servo_cmd(label)
        else:
            self._log_message("no detections~ empty frame, so boring lmao 💔")

    def start_process(self):
        self._log_message("starting full process~ don't blink okay? 😜")
        self.send_cmd("MOTOR,ON")
        try:
            time.sleep(1)  # Brief delay for motor start
            self.camera_detect_and_sort()
        finally:
            self.send_cmd("MOTOR,OFF")
        self._log_message("process done~ smooth run, hopefully XD")

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
        self.camera_running = False
        self.stop_camera()
        self.close_serial()
        if cv2 is not None:
            cv2.destroyAllWindows()
        print("shutting down~ don't cry, i'll be back lol 😘")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MoistureSorterGUI(root)
    root.mainloop()