#!/usr/bin/env python3
"""
Automated Copra Segregation System (ACSS) GUI
- Provides a user interface for controlling and monitoring the copra segregation process.
- Integrates with Arduino via serial for automated detection and sorting using ultrasonic sensors and servos.
- Uses YOLO for copra classification (Raw, Standard, Overcooked) based on camera frames triggered by Arduino.
- Displays real-time camera preview, logs (classification results, system events), and statistics.
- About tab with project description, objectives, and team profiles.
- Exit tab for safe shutdown.
- Arduino logs (ACK, TRIG, ERR, DBG) are shown in terminal; GUI log shows classification and system events.

Expected Functionalities:
- Home Tab: Start/Stop process, camera preview with live bounding boxes, log display with classification results and sync errors.
- Statistics Tab: Tracks processed copra, average moisture, total counts, timestamps.
- About Tab: Project info, objectives, team profiles with images; scrollable; min/max button.
- Exit Tab: Graceful exit with confirmation, stopping processes and closing serial/camera.
- Serial Interaction: Sends AUTO_ENABLE/DISABLE, RESET, PING, classifications; receives ACK,AT_CAM, TRIG, ERR, DBG.
- Moisture Estimation: Based on YOLO confidence (Raw: 7.1-60%, Standard: 6-7%, Overcooked: 4-5.9%).

Setup Instructions:
1. Install dependencies: pip install pyserial opencv-python numpy picamera2 ultralytics pillow
2. Connect Arduino to Raspberry Pi via USB (ensure SERIAL_PORT matches, e.g., /dev/ttyUSB0).
3. Place YOLO model file at YOLO_MODEL_PATH (train or download a copra-specific model).
4. Place team profile images in resources/profiles/ (christian.png, marielle.png, jerald.png, johnpaul.png).
5. Run on Raspberry Pi with camera module enabled (raspi-config > Interface > Camera > Enable).
6. Adjust SERIAL_PORT and YOLO_MODEL_PATH if needed.
7. Start the script: python3 this_script.py
- Note: Runs in full-screen (1024x600); use About tab button to minimize/maximize.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time

# Optional imports (graceful degradation)
try:
    import serial
    SERIAL_AVAILABLE = True
except:
    serial = None
    SERIAL_AVAILABLE = False
try:
    import cv2
    import numpy as np
except:
    cv2 = None
    np = None
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except:
    PICAMERA2_AVAILABLE = False
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except:
    ULTRALYTICS_AVAILABLE = False
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except:
    PIL_AVAILABLE = False

# ------------------ USER SETTINGS ------------------
CAM_PREVIEW_SIZE = (640, 480)  # Higher resolution as requested
USERNAME = "Copra Buyer 01"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200  # Matches Arduino
YOLO_MODEL_PATH = "my_model/my_model.pt"  # Using yolov11n.pt
TRACKER_PATH = "bytetrack.yaml"  # Added for live tracking in preview
CLASSIFICATION_TIMEOUT_S = 2.0  # Within Arduino's CLASS_WAIT_MS = 3000ms
MAX_FRAME_AGE_S = 0.7  # Increased slightly to account for system load
PING_INTERVAL_S = 5.0  # Send PING every 5 seconds
CLASSIFICATION_RETRIES = 2  # Retry classification if frame is stale
YOLO_FRAME_SKIP = 2  # Run YOLO every 2nd frame to balance performance

# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System")
        self.root.attributes('-fullscreen', True)

        # Notebook (Tabs)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        # Create tabs
        self.home_tab = ttk.Frame(notebook)
        self.statistics_tab = ttk.Frame(notebook)
        self.about_tab = ttk.Frame(notebook)
        self.exit_tab = ttk.Frame(notebook)

        notebook.add(self.home_tab, text="Home")
        notebook.add(self.statistics_tab, text="Statistics")
        notebook.add(self.about_tab, text="About")
        notebook.add(self.exit_tab, text="Exit")

        # Build tab contents
        self._build_home_tab()
        self._build_statistics_tab()
        self._build_about_tab()
        self._build_exit_tab()

        # State tracking
        self.camera_running = False
        self.process_running = False
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.camera_thread = None
        self.ping_thread = None
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_frame_time = 0
        self.serial_reader_thread_obj = None
        self.serial_error_logged = False
        self.frame_drop_logged = False
        self.moisture_sums = {'Raw': 0.0, 'Standard': 0.0, 'Overcooked': 0.0}
        self.class_to_sort = {0: 'L', 1: 'C', 2: 'R'}  # Arduino maps 'C' to no servo action
        self.category_map = {0: 'Raw', 1: 'Standard', 2: 'Overcooked'}
        self.stats = {
            'Raw': 0,
            'Standard': 0,
            'Overcooked': 0,
            'total': 0,
            'start_time': None,
            'end_time': None
        }
        self.copra_counter = 0
        self.last_ping_time = 0
        self.frame_counter = 0  # For YOLO frame skipping

        # Load YOLO
        if ULTRALYTICS_AVAILABLE:
            try:
                print("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                print(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message(f"YOLO load failed: {ex}", console_only=True)
                self.process_btn.config(state='disabled')

        # Auto-start serial and camera
        self.open_serial()
        if PICAMERA2_AVAILABLE:
            self.start_camera()
        else:
            self.process_btn.config(state='disabled')

        if not SERIAL_AVAILABLE:
            self.process_btn.config(state='disabled')

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Bind tab change to show/hide min/max button
        notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        # Min/Max button (initially hidden)
        self.min_max_btn = tk.Button(root, text="Minimize", command=self._toggle_fullscreen,
                                     font=("Arial", 10), bg="gray", fg="white")
        self.min_max_btn.place(relx=0.95, rely=0.95, anchor='se')
        self.min_max_btn.place_forget()

    def _on_tab_changed(self, event):
        notebook = event.widget
        selected_tab = notebook.select()
        if notebook.tab(selected_tab, "text") == "About":
            self.min_max_btn.place(relx=0.95, rely=0.95, anchor='se')
        else:
            self.min_max_btn.place_forget()

    def _toggle_fullscreen(self):
        is_fullscreen = self.root.attributes('-fullscreen')
        self.root.attributes('-fullscreen', not is_fullscreen)
        self.min_max_btn.config(text="Minimize" if not is_fullscreen else "Maximize")
        if is_fullscreen:
            self.root.geometry("1024x600")

    def _build_home_tab(self):
        frm = tk.Frame(self.home_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=3)
        frm.columnconfigure(1, weight=2)

        left_frame = tk.Frame(frm)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        left_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=0)
        left_frame.columnconfigure(0, weight=1)

        cam_frame = tk.LabelFrame(left_frame, text="Camera Preview")
        cam_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0],
                                    height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(padx=4, pady=4)

        self.process_btn = tk.Button(left_frame, text="Start Process",
                                    font=("Arial", 14, "bold"), bg="green", fg="white",
                                    command=self._toggle_process)
        self.process_btn.grid(row=1, column=0, sticky="ew", padx=4, pady=8)

        if not PICAMERA2_AVAILABLE or not ULTRALYTICS_AVAILABLE:
            self.process_btn.config(state='disabled')

        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, state='disabled', wrap='word', height=20, width=40)
        self.log.pack(fill='both', expand=True)

    def _toggle_process(self):
        if not self.process_running:
            if not (self.serial and self.serial.is_open):
                self._log_message("Error: Serial port not open. Check connection.")
                return
            if not self.yolo:
                self._log_message("Error: YOLO model not loaded.")
                return
            if not self.camera_running:
                self._log_message("Error: Camera not running.")
                return
            self.process_btn.config(text="Stop Process", bg="red")
            self._log_message("System Started")
            self.start_process()
            self.process_running = True
        else:
            self.process_btn.config(text="Start Process", bg="green")
            self._log_message("System Stopped")
            self.stop_process()
            self.process_running = False

    def _log_message(self, msg, console_only=False):
        print(msg)
        if console_only:
            return
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert("end", full_msg + "\n")
        self.log.see("end")
        self.log.config(state='disabled')

    def _build_statistics_tab(self):
        frm = tk.Frame(self.statistics_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        stats_frame = tk.LabelFrame(frm, text="Processing Statistics")
        stats_frame.pack(fill="both", expand=True, padx=4, pady=4)

        stats_frame.columnconfigure(0, weight=1)
        stats_frame.columnconfigure(1, weight=1)
        stats_frame.columnconfigure(2, weight=1)

        tk.Label(stats_frame, text=f"Username: {USERNAME}", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)

        headers = ["Category", "Pieces Processed", "Avg. Moisture"]
        for col, header in enumerate(headers):
            tk.Label(stats_frame, text=header, font=("Arial", 10, "bold")).grid(row=1, column=col, padx=8, pady=4)

        categories = ["Standard", "Raw", "Overcooked"]
        self.stat_pieces = {}
        self.stat_moisture = {}
        for i, cat in enumerate(categories, start=2):
            tk.Label(stats_frame, text=cat).grid(row=i, column=0, padx=8, pady=4, sticky="w")
            self.stat_pieces[cat] = tk.Label(stats_frame, text="0")
            self.stat_pieces[cat].grid(row=i, column=1, padx=8, pady=4)
            self.stat_moisture[cat] = tk.Label(stats_frame, text="0.0%")
            self.stat_moisture[cat].grid(row=i, column=2, padx=8, pady=4)

        row_offset = len(categories) + 2
        tk.Label(stats_frame, text="Total Pieces Processed:", font=("Arial", 10, "bold")).grid(row=row_offset, column=0, padx=8, pady=4, sticky="w")
        self.total_pieces_label = tk.Label(stats_frame, text="0")
        self.total_pieces_label.grid(row=row_offset, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Total Avg Moisture:", font=("Arial", 10, "bold")).grid(row=row_offset+1, column=0, padx=8, pady=4, sticky="w")
        self.total_moisture_label = tk.Label(stats_frame, text="0.0%")
        self.total_moisture_label.grid(row=row_offset+1, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing Start Time:", font=("Arial", 10, "bold")).grid(row=row_offset+2, column=0, padx=8, pady=4, sticky="w")
        self.start_time_label = tk.Label(stats_frame, text="N/A")
        self.start_time_label.grid(row=row_offset+2, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing End Time:", font=("Arial", 10, "bold")).grid(row=row_offset+3, column=0, padx=8, pady=4, sticky="w")
        self.end_time_label = tk.Label(stats_frame, text="N/A")
        self.end_time_label.grid(row=row_offset+3, column=1, padx=8, pady=4)

        tk.Button(stats_frame, text="Reset Statistics", font=("Arial", 10), command=self._reset_stats).grid(row=row_offset+4, column=0, columnspan=3, pady=10)

    def _reset_stats(self):
        self.send_cmd("RESET")
        for cat in ["Raw", "Standard", "Overcooked"]:
            self.stats[cat] = 0
            self.moisture_sums[cat] = 0.0
        self.stats['total'] = 0
        self.stats['start_time'] = None
        self.stats['end_time'] = None
        self.copra_counter = 0
        self.update_stats()
        self._log_message("Statistics and Arduino state reset.")

    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=10, pady=10)

        canvas = tk.Canvas(frm)
        scrollbar = ttk.Scrollbar(frm, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        about_frame = tk.LabelFrame(scrollable_frame, text="About ACSS", font=("Arial", 14, "bold"))
        about_frame.pack(fill="both", expand=True, padx=10, pady=10)

        tk.Label(about_frame, text="Automated Copra Segregation System", font=("Arial", 16, "bold")).pack(pady=5)
        tk.Label(about_frame, text="Version: 1.0\nDeveloped for Practice and Design 2\nMarinduque State University, 2025",
                font=("Arial", 12), justify="center").pack(pady=5)

        try:
            img = Image.open("resources/profiles/acss-3d.png")
            img = img.resize((200, 200), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            img_label = tk.Label(about_frame, image=photo)
            img_label.image = photo
            img_label.pack(pady=10)
        except Exception as e:
            tk.Label(about_frame, text="Image placeholder (acss-3d.png)", bg="gray", font=("Arial", 12)).pack(pady=10)

        tk.Label(about_frame, text="Project Description", font=("Arial", 14, "bold")).pack(anchor="w", pady=10)
        tk.Label(about_frame,
                text=" The Automated Copra Segregation System (ACSS) is a prototype designed to help sort copra based on its quality using a camera and sensors. The system classifies copra into three types: Raw, Standard, and Overcooked, as it moves along a conveyor. By automating this process, the system aims to lessen manual work, make sorting faster, and provide a more consistent way to assess copra quality.",
                font=("Arial", 12), justify="left", wraplength=900).pack(pady=5)

        tk.Label(about_frame, text="Objectives", font=("Arial", 14, "bold")).pack(anchor="w", pady=10)
        objectives = [
            "a) Improve efficiency in copra processing by automating classification and sorting.",
            "b) Reduce manual labor and human error in quality assessment.",
            "c) Enhance accuracy using computer vision (YOLO) and sensor data for real-time decisions."
        ]
        for obj in objectives:
            tk.Label(about_frame, text=obj, font=("Arial", 12), justify="left").pack(anchor="w", padx=20, pady=2)

        tk.Label(about_frame, text="Development Team", font=("Arial", 14, "bold")).pack(anchor="w", pady=10)
        team_frame = tk.Frame(about_frame)
        team_frame.pack(pady=10)

        team_members = [
            ("jerald.png", "Jerald", "Project Lead & Systems Integrator"),
            ("christian.png", "Christian", "Software & Electronics Developer"),
            ("johnpaul.png", "John Paul", "Mechanical Design & Fabrication Engineer"),
            ("marielle.png", "Marielle", "Technical Documentation & Quality Analyst")
        ]

        for i, (img_path, name, title) in enumerate(team_members):
            member_frame = tk.Frame(team_frame)
            member_frame.grid(row=i//2, column=i%2, padx=20, pady=10, sticky="n")

            try:
                img = Image.open(f"resources/profiles/{img_path}")
                img = img.resize((100, 100), Image.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                img_label = tk.Label(member_frame, image=photo)
                img_label.image = photo
                img_label.pack()
            except Exception as e:
                tk.Label(member_frame, text=f"{name} Image", bg="gray", font=("Arial", 10)).pack()

            tk.Label(member_frame, text=name, font=("Arial", 12, "bold")).pack(pady=2)
            tk.Label(member_frame, text=title, font=("Arial", 10)).pack()

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
            self.stop_process()
            self.stop_camera()
            self.close_serial()
            self.on_close()

    def open_serial(self):
        try:
            if not SERIAL_AVAILABLE:
                self._log_message("Error: pyserial not installed.")
                return
            if self.serial and self.serial.is_open:
                print("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self._log_message(f"Connected to Arduino on {SERIAL_PORT}@{SERIAL_BAUD}")
            self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
            self.serial_reader_thread_obj.start()
            self.ping_thread = threading.Thread(target=self.ping_loop, daemon=True)
            self.ping_thread.start()
            print("Serial reader and ping threads started.")
        except Exception as e:
            self._log_message(f"Failed to connect to Arduino: {e}")

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.send_cmd("RESET")
                self.serial.close()
                self._log_message("Serial connection closed.")
        except Exception as e:
            self._log_message(f"Error closing serial: {e}")

    def send_cmd(self, text):
        try:
            if not self.serial or not self.serial.is_open:
                self._log_message(f"Serial not open — can't send: {text}")
                return False
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            print(f"Sent to Arduino: {text}")
            return True
        except Exception as e:
            self._log_message(f"Failed to send command '{text}': {e}")
            return False

    def ping_loop(self):
        """Send periodic PING commands to ensure Arduino connection."""
        while self.running and self.serial and self.serial.is_open:
            if time.time() - self.last_ping_time >= PING_INTERVAL_S:
                self.send_cmd("PING")
                self.last_ping_time = time.time()
            time.sleep(0.5)

    def serial_reader_thread(self):
        print("Serial reader started.")
        try:
            while self.serial and self.serial.is_open and self.running:
                try:
                    with self.serial_lock:
                        self.serial.reset_input_buffer()  # Clear stale data
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    print(f"ARDUINO: {line}")
                    if line.startswith("ACK,AT_CAM"):
                        self.perform_classification()
                    elif line == "TRIG,FLAP_OBJECT_DETECTED":
                        self._log_message("Copra detected at flapper, centering for sorting.")
                    elif line == "ERR,MOTOR_TIMEOUT":
                        self._log_message("Error: Camera sensor missed object. Check alignment, distance (<10cm), or jump (>27cm).")
                    elif line.startswith("ERR,FIFO_FULL"):
                        self._log_message("Error: Arduino queue full. Resetting system.")
                        self._reset_stats()
                    elif line.startswith("ACK,CLASS,"):
                        self._log_message(f"Arduino confirmed classification: {line.split(',')[-1]}")
                    elif line == "ACK,SORT,L":
                        self._log_message("Sorted Overcooked (left servo).")
                    elif line == "ACK,SORT,R":
                        self._log_message("Sorted Raw (right servo).")
                    elif line == "ACK,MOTOR,START,2000":
                        self._log_message("Moving Standard copra (conveyor).")
                    elif line == "ACK,PING":
                        print("Arduino responded to PING: Connection alive.")
                    elif line.startswith("DBG,CAM_ULTRA,DIST="):
                        # Parse camera debug messages
                        parts = line.split(",")
                        if len(parts) >= 3:
                            if "DETECTED_TIMEOUT" in line:
                                self._log_message("Camera sensor: Timeout detection (999cm).", console_only=True)
                            elif "DETECTED_JUMP" in line:
                                prev_dist = parts[-1].split("=")[1] if "=" in parts[-1] else "N/A"
                                self._log_message(f"Camera sensor: Distance jump detected (prev={prev_dist}cm).", console_only=True)
                            elif "DETECTED" in line:
                                self._log_message("Camera sensor: Close object detected (<10cm).", console_only=True)
                except Exception as e:
                    if not self.serial_error_logged:
                        self._log_message(f"Serial reader error: {e}")
                        self.serial_error_logged = True
                    time.sleep(0.2)
        except Exception as outer:
            self._log_message(f"Serial reader crashed: {outer}")

    def perform_classification(self):
        """Perform YOLO detection and send classification to Arduino within timeout."""
        start_time = time.time()
        for attempt in range(CLASSIFICATION_RETRIES):
            try:
                with self.frame_lock:
                    if self.latest_frame is None or (time.time() - self.latest_frame_time) > MAX_FRAME_AGE_S:
                        if attempt < CLASSIFICATION_RETRIES - 1:
                            print(f"Stale frame on attempt {attempt + 1}, retrying...")
                            time.sleep(0.1)
                            continue
                        self._log_message("No recent frame available after retries. Defaulting to OVERCOOKED.")
                        self.send_cmd("OVERCOOKED")
                        return
                    frame = self.latest_frame.copy()

                if time.time() - start_time > CLASSIFICATION_TIMEOUT_S:
                    self._log_message("Classification timeout: Frame capture too slow. Defaulting to OVERCOOKED.")
                    self.send_cmd("OVERCOOKED")
                    return

                results = self.yolo.predict(
                    source=frame,
                    conf=0.3,
                    max_det=3,
                    verbose=False
                )

                if time.time() - start_time > CLASSIFICATION_TIMEOUT_S:
                    self._log_message("Classification timeout: YOLO prediction too slow. Defaulting to OVERCOOKED.")
                    self.send_cmd("OVERCOOKED")
                    return

                has_detection = results and results[0].boxes and len(results[0].boxes) > 0

                if has_detection:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                    conf_tensor = results[0].boxes.conf.cpu().numpy()
                    candidates = []
                    for i in range(len(cls_tensor)):
                        cls = cls_tensor[i]
                        conf = conf_tensor[i]
                        if conf > 0.3:
                            if cls == 0:  # Raw
                                moisture = 7.1 + (conf - 0.3) * (60.0 - 7.1) / 0.7
                            elif cls == 1:  # Standard
                                moisture = 6.0 + (conf - 0.3) * (7.0 - 6.0) / 0.7
                            else:  # Overcooked or unknown
                                cls = 2
                                moisture = 4.0 + (conf - 0.3) * (5.9 - 4.0) / 0.7
                            moisture = round(moisture, 1)
                            candidates.append((cls, conf, moisture))

                    if candidates:
                        candidates.sort(key=lambda x: x[1], reverse=True)
                        cls, conf, moisture = candidates[0]
                        category = self.category_map.get(cls, 'Overcooked')
                        class_str = category.upper()
                        if self.send_cmd(class_str):
                            self.copra_counter += 1
                            self._log_message(f"{category} Copra # {self.copra_counter:04d}")
                            self._log_message(f"Moisture: {moisture}%")
                            self.stats[category] += 1
                            self.stats['total'] += 1
                            self.moisture_sums[category] += moisture
                            self.root.after(0, self.update_stats)
                        return
                    else:
                        self._log_message("No confident detection. Defaulting to OVERCOOKED.")
                        self.send_cmd("OVERCOOKED")
                else:
                    self._log_message("No objects detected by YOLO. Defaulting to OVERCOOKED.")
                    self.send_cmd("OVERCOOKED")
                return
            except Exception as e:
                if attempt < CLASSIFICATION_RETRIES - 1:
                    print(f"Classification error on attempt {attempt + 1}: {e}, retrying...")
                    time.sleep(0.1)
                    continue
                self._log_message(f"Classification failed after retries: {e}. Defaulting to OVERCOOKED.")
                self.send_cmd("OVERCOOKED")
                return

    def draw_bounding_boxes(self, frame, results):
        """Draw bounding boxes, class labels, and confidences on the frame."""
        if not results or not results[0].boxes or len(results[0].boxes) == 0:
            return frame
        boxes = results[0].boxes.xyxy.cpu().numpy()
        cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
        conf_tensor = results[0].boxes.conf.cpu().numpy()
        for i in range(len(boxes)):
            if conf_tensor[i] > 0.3:
                x1, y1, x2, y2 = map(int, boxes[i])
                cls = cls_tensor[i]
                conf = conf_tensor[i]
                category = self.category_map.get(cls, 'Overcooked')
                color = {
                    'Raw': (0, 255, 0),       # Green
                    'Standard': (255, 255, 0), # Yellow
                    'Overcooked': (0, 0, 255)  # Red
                }.get(category, (0, 0, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{category} {conf:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return frame

    def start_process(self):
        try:
            if not (self.serial and self.serial.is_open):
                self._log_message("Error: Serial port not open. Check connection.")
                return
            if not self.yolo:
                self._log_message("Error: YOLO model not loaded.")
                return
            self.send_cmd("AUTO_ENABLE")
            self.stats['start_time'] = time.time()
            self.stats['end_time'] = None
            self.serial_error_logged = False
            self.frame_drop_logged = False
            self.last_ping_time = time.time()
            self._log_message("Process started: Auto mode enabled.")
        except Exception as e:
            self._log_message(f"Start process error: {e}")

    def stop_process(self):
        try:
            self.send_cmd("AUTO_DISABLE")
            self.stats['end_time'] = time.time()
            self.serial_error_logged = False
            self.frame_drop_logged = False
            self._log_message("Process stopped: Auto mode disabled.")
            self.root.after(0, self.update_stats)
        except Exception as e:
            self._log_message(f"Stop process error: {e}")

    def start_camera(self):
        if self.camera_running:
            print("Camera already running.")
            return
        if not PICAMERA2_AVAILABLE:
            self._log_message("Error: picamera2 not available.")
            self.process_btn.config(state='disabled')
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": CAM_PREVIEW_SIZE}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self._log_message("Camera started.")
        except Exception as e:
            self._log_message(f"Camera start failed: {e}")
            self.process_btn.config(state='disabled')

    def stop_camera(self):
        try:
            if not self.camera_running:
                print("Camera not running.")
                return
            self.camera_running = False
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self._log_message("Camera stopped.")
        except Exception as e:
            self._log_message(f"Camera stop error: {e}")

    def camera_loop(self):
        frame_counter = 0
        fps_time = time.time()
        last_frame_time = time.time()
        last_fps_log = time.time()
        display_skip = 0
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                current_time = time.time()
                if current_time - last_frame_time > 0.2:
                    if not self.frame_drop_logged:
                        print(f"Frame drop detected: {current_time - last_frame_time:.3f}s")
                        self.frame_drop_logged = True
                last_frame_time = current_time

                # Run YOLO inference every YOLO_FRAME_SKIP frames
                results = None
                if self.yolo and self.frame_counter % YOLO_FRAME_SKIP == 0:
                    results = self.yolo.track(
                        source=frame,
                        persist=True,
                        tracker=TRACKER_PATH,
                        conf=0.3,
                        max_det=3,
                        verbose=False
                    )
                    frame = self.draw_bounding_boxes(frame, results)

                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_frame_time = current_time

                display_skip = (display_skip + 1) % 2
                if display_skip == 0:
                    display_frame = frame.copy()
                    display_frame = cv2.resize(display_frame, CAM_PREVIEW_SIZE)
                    self.update_canvas_with_frame(display_frame)

                self.frame_counter += 1
                frame_counter += 1
                if frame_counter >= 50 and current_time - last_fps_log > 5.0:
                    fps = frame_counter / (current_time - fps_time)
                    print(f"FPS: {fps:.1f}")
                    fps_time = current_time
                    frame_counter = 0

                time.sleep(0.01)
            except Exception as e:
                self._log_message(f"Camera loop error: {e}")
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            if PIL_AVAILABLE:
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                self.cam_canvas.imgtk = imgtk
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
            else:
                cv2.imshow("Camera Preview", bgr_frame)
                cv2.waitKey(1)
        except Exception as e:
            self._log_message(f"Display frame error: {e}")

    def update_stats(self):
        try:
            for cat in ["Standard", "Raw", "Overcooked"]:
                pieces = self.stats[cat]
                self.stat_pieces[cat].config(text=str(pieces))
                moisture_avg = (self.moisture_sums[cat] / pieces) if pieces > 0 else 0.0
                self.stat_moisture[cat].config(text=f"{moisture_avg:.1f}%")
            self.total_pieces_label.config(text=str(self.stats['total']))
            total_pieces = self.stats['total']
            total_moisture = sum(self.moisture_sums.values())
            total_avg = (total_moisture / total_pieces) if total_pieces > 0 else 0.0
            self.total_moisture_label.config(text=f"{total_avg:.1f}%")
            start_str = time.strftime("%H:%M:%S", time.localtime(self.stats['start_time'])) if self.stats['start_time'] else "N/A"
            end_str = time.strftime("%H:%M:%S", time.localtime(self.stats['end_time'])) if self.stats['end_time'] else "N/A"
            self.start_time_label.config(text=start_str)
            self.end_time_label.config(text=end_str)
            self.root.update()
        except Exception as e:
            self._log_message(f"Stats update error: {e}")

    def on_close(self):
        self.running = False
        self.process_running = False
        self.camera_running = False
        self.stop_process()
        self.stop_camera()
        self.close_serial()
        self._log_message("Application closed.")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()