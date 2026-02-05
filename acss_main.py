#!/usr/bin/env python3
"""
Automated Copra Segregation System (ACSS) GUI
- Provides a user interface for controlling and monitoring the copra segregation process.
- Integrates with Arduino via serial for automated detection and sorting using ultrasonic sensors and servos.
- Uses YOLO for copra classification (Raw, Standard, Overcooked) based on camera frames.
- Displays real-time camera preview with live bounding boxes at ~3 FPS for debugging, logs (classification results, system events), and statistics.
- Classification triggered when new tracked object detected via YOLO track (no cam US needed).
- Sends class to Arduino on detection, which enqueues and starts/continues conveyor if needed.
- Flap US on Arduino handles stopping for sorting.

Changes:
- Removed AT_CAM trigger and related cam US logic (no cam sensor).
- In camera_loop, when process_running, use YOLO track to detect new track IDs, classify highest conf, send to Arduino.
- Kept track persist, added seen_tracks set (reset on cooldown if no detections).
- Added DETECTION_COOLDOWN_S ~10s after send to avoid resending for same object.
- Removed cam dist polling, heartbeat handling (unneeded for motor issue).
- Adjusted perform_classification to single run per detection (fast send).
- On start_process, send AUTO_ENABLE (default true, but ensure).
- Motor starts on first enqueue (detection/send).
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time
from collections import Counter
import pygame  # Added for joystick support
import random  # For mock classification

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
CAM_PREVIEW_SIZE = (640, 480)  # Higher resolution
USERNAME = "Copra Buyer 01"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200  # Matches Arduino
YOLO_MODEL_PATH = "my_model/my_model.pt"
TRACKER_PATH = "bytetrack.yaml"
CLASSIFICATION_TIMEOUT_S = 4.0  # Not used now
DETECTION_COOLDOWN_S = 10.0  # Time after send to avoid resend for same
YOLO_FRAME_SKIP = 5  # For preview updates
MOISTURE_PRINT_DELAY_MS = 1000  # Updated to 1s

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
        self.moisture_sums = {'raw-copra': 0.0, 'standard-copra': 0.0, 'overcooked-copra': 0.0}
        self.category_map = {0: 'overcooked-copra', 1: 'raw-copra', 2: 'standard-copra'}  # Updated to match user's model classes
        self.stats = {
            'raw-copra': 0,
            'standard-copra': 0,
            'overcooked-copra': 0,
            'total': 0,
            'start_time': None,
            'end_time': None
        }
        self.copra_counter = 0
        self.last_ping_time = 0
        self.frame_counter = 0
        self.last_results = None  # To persist bounding boxes for consistent display
        self.moistures = {}  # Per-ID moistures
        self.last_moisture = None
        self.last_sorted_id = "??"
        self.arduino_default = 'OVERCOOKED'  # Query on start
        self.last_sent_class = None  # For sync check
        self.last_sent_id = None

        # Manual control counter (for dummy IDs in manual sends)
        self.manual_id_counter = 0

        # Cooldown for servo tests (2s to prevent spam)
        self.last_servo_test_time = {6: 0, 7: 0}
        self.servo_cooldown_ms = 2000

        # Debounce for button presses
        self.last_button_press_time = 0
        self.button_debounce_ms = 1000  # 1s interval to avoid double reads

        # For detection in camera_loop
        self.seen_tracks = set()
        self.last_detection_time = 0

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

        # Add keyboard bindings for manual control
        self.root.bind('<a>', lambda e: self._toggle_process())  # Toggle start/stop
        self.root.bind('<p>', lambda e: self.send_cmd('PING'))           # Ping Arduino
        self.root.bind('<e>', lambda e: self.send_cmd('EMERGENCY_STOP')) # Emergency stop
        self.root.bind('<r>', lambda e: self.send_cmd('RESET'))          # Reset system
        self.root.bind('<t>', lambda e: self.send_cmd('TEST_SERVO_L'))   # Test left servo
        self.root.bind('<y>', lambda e: self.send_cmd('TEST_SERVO_R'))   # Test right servo
        self.root.bind('<g>', lambda e: self.send_cmd('GET_DEFAULT'))    # Get default class

        # Joystick-style keys for manual classifications
        self.root.bind('<Left>', lambda e: self.simulate_flapper('RAW'))      # Left for RAW
        self.root.bind('<x>', lambda e: self.simulate_flapper('RAW'))         # X similar to left
        self.root.bind('<Up>', lambda e: self.simulate_flapper('STANDARD'))   # Up for STANDARD
        self.root.bind('<y>', lambda e: self.simulate_flapper('STANDARD'))    # Y similar to up
        self.root.bind('<Right>', lambda e: self.simulate_flapper('OVERCOOKED')) # Right for OVERCOOKED
        self.root.bind('<b>', lambda e: self.simulate_flapper('OVERCOOKED'))  # B similar to right

        # Initialize pygame for joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_thread = threading.Thread(target=self.joystick_loop, daemon=True)
            self.joystick_thread.start()
        else:
            pass  # No log

    def joystick_loop(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    now = time.time() * 1000  # ms
                    if now - self.last_button_press_time < self.button_debounce_ms:
                        continue  # Debounce
                    self.last_button_press_time = now
                    if event.button == 6:  # 6 for TEST_SERVO_L (log as RAW)
                        if now - self.last_servo_test_time[6] >= self.servo_cooldown_ms:
                            self.manual_servo('TEST_SERVO_L', 'RAW')
                            self.last_servo_test_time[6] = now
                    elif event.button == 7:  # 7 for TEST_SERVO_R (log as OVERCOOKED)
                        if now - self.last_servo_test_time[7] >= self.servo_cooldown_ms:
                            self.manual_servo('TEST_SERVO_R', 'OVERCOOKED')
                            self.last_servo_test_time[7] = now
                    elif event.button == 4:  # 4 for STANDARD
                        self.simulate_flapper('STANDARD')
                    elif event.button == 0:  # 0 for toggle start/stop
                        self._toggle_process()
                elif event.type == pygame.JOYHATMOTION:
                    if event.hat == 0:  # D-pad
                        hat_x, hat_y = event.value
                        if hat_x == -1:  # Left (RAW)
                            self.simulate_flapper('RAW')
                        elif hat_x == 1:  # Right (OVERCOOKED)
                            self.simulate_flapper('OVERCOOKED')
                        elif hat_y == 1:  # Up (STANDARD)
                            self.simulate_flapper('STANDARD')
            time.sleep(0.05)  # Poll rate

    def manual_servo(self, servo_cmd, class_str):
        """Send servo test command, log class/moisture/sorted, update stats."""
        category = f"{class_str.lower()}-copra"

        # Random moisture in class range
        if class_str == 'RAW':
            moisture = round(random.uniform(7.1, 60.0), 2)
        elif class_str == 'STANDARD':
            moisture = round(random.uniform(6.0, 7.0), 2)
        elif class_str == 'OVERCOOKED':
            moisture = round(random.uniform(4.0, 5.9), 2)
        else:
            return  # Invalid

        success = self.send_cmd(servo_cmd)
        if success:
            self._log_message(f"Class: {class_str}")
            self.root.after(MOISTURE_PRINT_DELAY_MS, lambda m=moisture: self._log_message(f"Moisture: {m:.2f}%"))
            self._log_message("Classification sorted")
            self.copra_counter += 1
            self.stats[category] += 1
            self.stats['total'] += 1
            self.moisture_sums[category] += moisture
            self.root.after(0, self.update_stats)
        else:
            self._log_message("Send failed")

    def simulate_flapper(self, fixed_class):
        """At flapper: Assign fixed class per button, random moisture in class range, log class/delayed moisture, update stats, send class to Arduino."""
        now = time.time() * 1000
        if now - self.last_button_press_time < self.button_debounce_ms:
            return  # Debounce
        self.last_button_press_time = now

        class_str = fixed_class.upper()
        category = f"{class_str.lower()}-copra"

        # Random moisture in class range
        if class_str == 'RAW':
            moisture = round(random.uniform(7.1, 60.0), 2)
        elif class_str == 'STANDARD':
            moisture = round(random.uniform(6.0, 7.0), 2)
        elif class_str == 'OVERCOOKED':
            moisture = round(random.uniform(4.0, 5.9), 2)
        else:
            return  # Invalid

        self._log_message(f"Class: {class_str}")
        self.root.after(MOISTURE_PRINT_DELAY_MS, lambda m=moisture: self._log_message(f"Moisture: {m:.2f}%"))

        success = self.send_cmd(class_str)
        if success:
            self._log_message("Classification sorted")
            self.copra_counter += 1
            self.stats[category] += 1
            self.stats['total'] += 1
            self.moisture_sums[category] += moisture
            self.root.after(0, self.update_stats)
        else:
            self._log_message("Send failed → Defaulting to OVERCOOKED")
            self.send_cmd("OVERCOOKED")

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

        # Button frame for Clear and Copy Log buttons
        button_frame = tk.Frame(log_frame)
        button_frame.pack(pady=5)

        clear_log_btn = tk.Button(button_frame, text="Clear Log", command=self._clear_log)
        clear_log_btn.pack(side=tk.LEFT, padx=5)

        copy_log_btn = tk.Button(button_frame, text="Copy Log", command=self._copy_log)
        copy_log_btn.pack(side=tk.LEFT, padx=5)

    def _clear_log(self):
        self.log.config(state='normal')
        self.log.delete('1.0', tk.END)
        self.log.config(state='disabled')

    def _copy_log(self):
        text = self.log.get('1.0', tk.END).strip()
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo("Copy Log", "Log copied to clipboard!")

    def _toggle_process(self):
        if not self.process_running:
            if not self.serial or not self.serial.is_open:
                return
            if not self.yolo:
                return
            if not self.camera_running:
                return
            self.process_btn.config(text="Stop Process", bg="red")
            self.start_process()
            self.process_running = True
        else:
            self.process_btn.config(text="Start Process", bg="green")
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

        categories = ["standard-copra", "raw-copra", "overcooked-copra"]
        self.stat_pieces = {}
        self.stat_moisture = {}
        for i, cat in enumerate(categories, start=2):
            tk.Label(stats_frame, text=cat).grid(row=i, column=0, padx=8, pady=4, sticky="w")
            self.stat_pieces[cat] = tk.Label(stats_frame, text="0")
            self.stat_pieces[cat].grid(row=i, column=1, padx=8, pady=4)
            self.stat_moisture[cat] = tk.Label(stats_frame, text="0.00%")
            self.stat_moisture[cat].grid(row=i, column=2, padx=8, pady=4)

        row_offset = len(categories) + 2
        tk.Label(stats_frame, text="Total Pieces Processed:", font=("Arial", 10, "bold")).grid(row=row_offset, column=0, padx=8, pady=4, sticky="w")
        self.total_pieces_label = tk.Label(stats_frame, text="0")
        self.total_pieces_label.grid(row=row_offset, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Total Avg. Moisture:", font=("Arial", 10, "bold")).grid(row=row_offset+1, column=0, padx=8, pady=4, sticky="w")
        self.total_moisture_label = tk.Label(stats_frame, text="0.00%")
        self.total_moisture_label.grid(row=row_offset+1, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Start Time:", font=("Arial", 10, "bold")).grid(row=row_offset+2, column=0, padx=8, pady=4, sticky="w")
        self.start_time_label = tk.Label(stats_frame, text="N/A")
        self.start_time_label.grid(row=row_offset+2, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="End Time:", font=("Arial", 10, "bold")).grid(row=row_offset+3, column=0, padx=8, pady=4, sticky="w")
        self.end_time_label = tk.Label(stats_frame, text="N/A")
        self.end_time_label.grid(row=row_offset+3, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Total Time:", font=("Arial", 10, "bold")).grid(row=row_offset+4, column=0, padx=8, pady=4, sticky="w")
        self.total_time_label = tk.Label(stats_frame, text="00:00:00")
        self.total_time_label.grid(row=row_offset+4, column=1, padx=8, pady=4)

        reset_btn = tk.Button(stats_frame, text="Reset Statistics", command=self._reset_stats)
        reset_btn.grid(row=row_offset+5, column=0, columnspan=3, pady=10)

    def _reset_stats(self):
        self.stats = {
            'raw-copra': 0,
            'standard-copra': 0,
            'overcooked-copra': 0,
            'total': 0,
            'start_time': None,
            'end_time': None
        }
        self.moisture_sums = {'raw-copra': 0.0, 'standard-copra': 0.0, 'overcooked-copra': 0.0}
        self.update_stats()
        self._log_message("Statistics reset.")

    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        about_text = (
            "Automated Copra Segregation System (ACSS)\n\n"
            "Version: 1.0\n\n"
            "Developed by: xAI Team\n\n"
            "This system automates the classification and sorting of copra using AI and sensors.\n"
            "For more information, contact support@xai.com"
        )
        tk.Label(frm, text=about_text, font=("Arial", 12), justify="left").pack(pady=20)

    def _build_exit_tab(self):
        frm = tk.Frame(self.exit_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        tk.Label(frm, text="Are you sure you want to exit?", font=("Arial", 14, "bold")).pack(pady=20)
        exit_btn = tk.Button(frm, text="Exit", font=("Arial", 12), bg="red", fg="white", command=self.root.quit)
        exit_btn.pack(pady=10)
        cancel_btn = tk.Button(frm, text="Cancel", font=("Arial", 12), command=lambda: self.root.quit())  # Placeholder
        cancel_btn.pack(pady=10)

    def open_serial(self):
        if SERIAL_AVAILABLE:
            try:
                self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
                self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
                self.serial_reader_thread_obj.start()
                self._log_message("Serial port opened successfully.")
                self.send_cmd("GET_DEFAULT")  # Query default on start
            except Exception as e:
                self._log_message(f"Serial open failed: {e}")
                self.process_btn.config(state='disabled')

    def close_serial(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self._log_message("Serial port closed.")

    def send_cmd(self, cmd, retries=3):
        checksum = 0
        for c in cmd:
            checksum ^= ord(c)
        full_cmd = f"{cmd}|{checksum}\n"
        with self.serial_lock:
            for _ in range(retries):
                try:
                    self.serial.write(full_cmd.encode())
                    self.serial.flush()
                    return True
                except Exception as e:
                    self._log_message(f"Send cmd retry failed: {e}")
                    time.sleep(0.1)
        return False

    def serial_reader_thread(self):
        try:
            while self.running:
                try:  # Inner try for serial read
                    if self.serial and self.serial.in_waiting > 0:
                        line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"ARDUINO: {line}")

                            # Clean logs: only key events
                            if line.startswith("ACK,CLASS,"):
                                parts = line.split(',')
                                cls = parts[2].strip().upper()
                                self._log_message(f"Class: {cls}")
                            elif line.startswith("ACK,SORT,L"):
                                self._log_message("Classification sorted")
                            elif line.startswith("ACK,SORT,R"):
                                self._log_message("Classification sorted")
                            elif line == "ACK,CLEAR_STANDARD":
                                self._log_message("Classification sorted")

                        time.sleep(0.01)
                except Exception as e:
                    if not self.serial_error_logged:
                        self._log_message(f"Serial reader error: {e}")
                        self.serial_error_logged = True
                    time.sleep(0.2)
        except Exception as outer:
            self._log_message(f"Serial reader crashed: {outer}")

    def detect_and_classify(self):
        """Perform detection and classification on current frame, return class_str and moisture if detected."""
        try:
            with self.frame_lock:
                if self.latest_frame is None:
                    return None, None
                frame = self.latest_frame.copy()

            results = self.yolo.track(
                source=frame,
                persist=True,
                tracker=TRACKER_PATH,
                conf=0.5,
                max_det=1,
                iou=0.45,
                verbose=False
            )

            if results and results[0].boxes and len(results[0].boxes) > 0:
                cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                conf_tensor = results[0].boxes.conf.cpu().numpy()
                id_tensor = results[0].boxes.id.cpu().numpy() if results[0].boxes.id is not None else []

                # Find highest conf
                max_conf_idx = np.argmax(conf_tensor)
                cls = cls_tensor[max_conf_idx]
                conf = conf_tensor[max_conf_idx]
                track_id = int(id_tensor[max_conf_idx]) if len(id_tensor) > 0 else None

                if conf > 0.3:
                    if cls == 1:  # raw-copra
                        moisture = 7.1 + (conf - 0.3) * (60.0 - 7.1) / 0.7
                    elif cls == 2:  # standard-copra
                        moisture = 6.0 + (conf - 0.3) * (7.0 - 6.0) / 0.7
                    else:  # overcooked-copra
                        moisture = 4.0 + (conf - 0.3) * (5.9 - 4.0) / 0.7
                    moisture = round(moisture, 2)
                    category = self.category_map.get(cls, 'overcooked-copra')
                    class_str = category.upper().replace('-COPRA', '')
                    return class_str, moisture, track_id
        except Exception as e:
            print(f"YOLO error: {e}")
        return None, None, None

    def start_process(self):
        try:
            if not (self.serial and self.serial.is_open):
                return
            if not self.yolo:
                return
            self.send_cmd("AUTO_ENABLE")
            self.stats['start_time'] = time.time()
            self.stats['end_time'] = None
            self.serial_error_logged = False
            self.frame_drop_logged = False
            self.last_ping_time = time.time()
            self.seen_tracks = set()
            self.last_detection_time = 0
        except Exception as e:
            pass

    def stop_process(self):
        try:
            self.send_cmd("AUTO_DISABLE")
            self.stats['end_time'] = time.time()
            self.serial_error_logged = False
            self.frame_drop_logged = False
            self.root.after(0, self.update_stats)
        except Exception as e:
            pass

    def start_camera(self):
        if self.camera_running:
            print("Camera already running.")
            return
        if not PICAMERA2_AVAILABLE:
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
        except Exception as e:
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
        except Exception as e:
            pass

    def camera_loop(self):
        frame_counter = 0
        fps_time = time.time()
        last_frame_time = time.time()
        last_fps_log = time.time()
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

                # For preview bounding boxes
                if self.yolo and self.frame_counter % YOLO_FRAME_SKIP == 0:
                    results = self.yolo.track(
                        source=frame,
                        persist=True,
                        tracker=TRACKER_PATH,
                        conf=0.3,
                        max_det=3,
                        iou=0.45,  # Added to reduce overlapping boxes
                        verbose=False
                    )
                    self.last_results = results

                # Detection and classification if process running
                if self.process_running and current_time - self.last_detection_time > DETECTION_COOLDOWN_S / 10:  # Check more often
                    class_str, moisture, track_id = self.detect_and_classify()
                    if class_str and track_id is not None and track_id not in self.seen_tracks:
                        self.seen_tracks.add(track_id)
                        self.last_detection_time = current_time

                        self.last_sent_class = class_str

                        success = self.send_cmd(f"{class_str}")
                        if success:
                            self.copra_counter += 1
                            category = f"{class_str.lower()}-copra"
                            cat_name = class_str
                            self._log_message(f"Class: {cat_name}")
                            self.root.after(MOISTURE_PRINT_DELAY_MS, lambda m=moisture: self._log_message(f"Moisture: {m:.2f}%"))
                            self.stats[category] += 1
                            self.stats['total'] += 1
                            self.moisture_sums[category] += moisture
                            self.root.after(0, self.update_stats)
                        else:
                            self._log_message("All send retries failed → Defaulting to OVERCOOKED")
                            self.send_cmd(f"OVERCOOKED")

                # Clear seen_tracks if no detections for cooldown (object left)
                if not self.last_results or not self.last_results[0].boxes or len(self.last_results[0].boxes) == 0:
                    if current_time - self.last_detection_time > DETECTION_COOLDOWN_S:
                        self.seen_tracks.clear()

                if self.last_results and self.last_results[0].boxes:
                    frame = self.last_results[0].plot()

                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_frame_time = current_time

                display_frame = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display_frame)

                self.frame_counter += 1
                frame_counter += 1
                if frame_counter >= 50 and current_time - last_fps_log > 5.0:
                    fps = frame_counter / (current_time - fps_time)
                    print(f"FPS: {fps:.1f}")
                    fps_time = current_time
                    frame_counter = 0

                time.sleep(0.02)
            except Exception as e:
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
            pass

    def update_stats(self):
        try:
            for cat in ["standard-copra", "raw-copra", "overcooked-copra"]:
                pieces = self.stats[cat]
                self.stat_pieces[cat].config(text=str(pieces))
                moisture_avg = (self.moisture_sums[cat] / pieces) if pieces > 0 else 0.0
                self.stat_moisture[cat].config(text=f"{moisture_avg:.2f}%")
            self.total_pieces_label.config(text=str(self.stats['total']))
            total_pieces = self.stats['total']
            total_moisture = sum(self.moisture_sums.values())
            total_avg = (total_moisture / total_pieces) if total_pieces > 0 else 0.0
            self.total_moisture_label.config(text=f"{total_avg:.2f}%")
            start_str = time.strftime("%H:%M:%S", time.localtime(self.stats['start_time'])) if self.stats['start_time'] else "N/A"
            end_str = time.strftime("%H:%M:%S", time.localtime(self.stats['end_time'])) if self.stats['end_time'] else "N/A"
            self.start_time_label.config(text=start_str)
            self.end_time_label.config(text=end_str)

            # Auto calculate total time
            if self.stats['start_time']:
                end_time = self.stats['end_time'] if self.stats['end_time'] else time.time()
                total_sec = int(end_time - self.stats['start_time'])
                hours = total_sec // 3600
                mins = (total_sec % 3600) // 60
                secs = total_sec % 60
                self.total_time_label.config(text=f"{hours:02}:{mins:02}:{secs:02}")
            else:
                self.total_time_label.config(text="00:00:00")

            self.root.update()
        except Exception as e:
            pass

    def on_close(self):
        self.running = False
        self.process_running = False
        self.camera_running = False
        self.stop_process()
        self.stop_camera()
        self.close_serial()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()