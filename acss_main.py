#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading, time
from collections import Counter, deque  # Added deque for queue

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
CLASSIFICATION_TIMEOUT_S = 4.0  # Increased for better sync
MAX_FRAME_AGE_S = 0.7  # Increased slightly to account for system load
PING_INTERVAL_S = 5.0  # Send PING every 5 seconds
CLASSIFICATION_RETRIES = 1  # Retry classification if frame is stale
YOLO_FRAME_SKIP = 10  # Adjust here: Lower for more frequent bounding box updates (e.g., 1 for every frame, may cause lag); higher for less frequent (e.g., 10 for ~3 FPS if camera ~30 FPS). Set to 1 for constant detection without skip.
CAM_DIST_POLL_INTERVAL_S = 10.0  # Poll cam dist every 10s
MOISTURE_PRINT_DELAY_MS = 500  # Updated to 0.5s
QUEUE_CLEAR_DELAY_MS = 1000  # 1s clear after last sort
OBJECT_DEBOUNCE_S = 1.0  # Debounce time to avoid multiple sends for same object

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
        self.cam_poll_thread = None
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
        self.last_detection_time = 0  # For debounce

        # Queue simulation with tweakable list
        self.copra_queue = deque()
        # Tweakable hardcoded list for simulation/testing (edit this as needed, up to 50 items)
        # Format: list of tuples (id, class_str, moisture) - IDs can be sequential or custom
        self.INITIAL_COPRA_LIST = [
            (1, 'RAW', 53.4),
            (2, 'RAW', 26.7),
            (3, 'RAW', 27.1),
            (4, 'RAW', 45.9),
            (5, 'RAW', 24.6),
            (6, 'RAW', 26.3),
            (7, 'RAW', 58.2),
            (8, 'RAW', 26.9),
            (9, 'RAW', 49.5),
            (10, 'RAW', 30.4),
        ]

        # Preload the queue with the initial list for simulation (comment out to disable auto-load)
        self.copra_queue.extend(self.INITIAL_COPRA_LIST)

        self.sorted_count = 0  # Track sorted items for auto-stop

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

        # Keyboard bindings (only keep toggle and debug)
        self.root.bind('<a>', lambda e: self._toggle_process())  # Toggle start/stop
        self.root.bind('<p>', lambda e: self.send_cmd('PING'))           # Ping Arduino
        self.root.bind('<e>', lambda e: self.send_cmd('EMERGENCY_STOP')) # Emergency stop
        self.root.bind('<r>', lambda e: self.send_cmd('RESET'))          # Reset system
        self.root.bind('<t>', lambda e: self.send_cmd('TEST_SERVO_L'))   # Test left servo
        self.root.bind('<y>', lambda e: self.send_cmd('TEST_SERVO_R'))   # Test right servo
        self.root.bind('<g>', lambda e: self.send_cmd('GET_CAM_DIST'))   # Get camera distance
        self.root.bind('<h>', lambda e: self.send_cmd('GET_DEFAULT'))    # Get default class

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
        self.send_cmd("RESET")
        for cat in ["raw-copra", "standard-copra", "overcooked-copra"]:
            self.stats[cat] = 0
            self.moisture_sums[cat] = 0.0
        self.stats['total'] = 0
        self.stats['start_time'] = None
        self.stats['end_time'] = None
        self.copra_counter = 0
        self.moistures = {}
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
            img = img.resize((200, 200), Image.Resampling.LANCZOS)
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
                img = img.resize((100, 100), Image.Resampling.LANCZOS)
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
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            self._log_message(f"Connected to Arduino on {SERIAL_PORT}@{SERIAL_BAUD}")
            self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
            self.serial_reader_thread_obj.start()
            self.ping_thread = threading.Thread(target=self.ping_loop, daemon=True)
            self.ping_thread.start()
            self.cam_poll_thread = threading.Thread(target=self.cam_dist_poll_loop, daemon=True)
            self.cam_poll_thread.start()
            print("Serial reader, ping, and cam poll threads started.")
            # Query Arduino default on connect
            self.send_cmd("GET_DEFAULT")
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

    def send_cmd(self, text, retries=3):
        attempt = 0
        while attempt < retries:
            try:
                if not self.serial or not self.serial.is_open:
                    self._log_message(f"Serial not open — can't send: {text}")
                    return False
                checksum = 0
                for char in text:
                    checksum ^= ord(char)
                full_cmd = (text.strip().upper() + f"|{checksum}\n").encode('utf-8')
                with self.serial_lock:
                    self.serial.write(full_cmd)
                    self.serial.flush()
                print(f"Sent to Arduino: {text} with checksum {checksum}")
                return True
            except Exception as e:
                self._log_message(f"Failed to send command '{text}' (attempt {attempt+1}): {e}")
                attempt += 1
                time.sleep(0.5)
        return False

    def ping_loop(self):
        while self.running and self.serial and self.serial.is_open:
            if time.time() - self.last_ping_time >= PING_INTERVAL_S:
                self.send_cmd("PING")
                self.last_ping_time = time.time()
            time.sleep(0.5)

    def cam_dist_poll_loop(self):
        while self.running and self.serial and self.serial.is_open:
            self.send_cmd("GET_CAM_DIST")
            time.sleep(CAM_DIST_POLL_INTERVAL_S)

    def serial_reader_thread(self):
        print("Serial reader started.")
        try:
            while self.serial and self.serial.is_open and self.running:
                try:
                    while self.serial.in_waiting > 0:
                        line = self.serial.readline().decode(errors='ignore').strip()
                        if not line:
                            continue
                        print(f"ARDUINO: {line}")

                        # === START US1: Copra detected at entrance ===
                        if line.startswith("ACK,ENQUEUE_PLACEHOLDER"):
                            parts = line.split(",")
                            idx = "??"
                            cid = "??"
                            for p in parts:
                                if "idx=" in p:
                                    idx = p.split("=")[1]
                                if "id=" in p:
                                    cid = p.split("=")[1]
                            display_id = str(int(cid) + 1).zfill(4)
                            self._log_message(f"Copra detected at Start (US1) → #{display_id} queued (slot {idx})")

                        # === COPRA ARRIVED AT CAMERA ===
                        elif line.startswith("ACK,AT_CAM"):
                            parts = line.split(",")
                            id = None
                            for part in parts:
                                if 'id=' in part:
                                    id = int(part.split('=')[1])
                                    break
                            if id is not None:
                                display_id = str(id + 1).zfill(4)
                                self._log_message(f"Copra #{display_id} at Camera → Classifying...")
                            self.perform_classification(id)

                        # === CLASSIFICATION RECEIVED BY ARDUINO ===
                        elif line.startswith("ACK,CLASS_RECEIVED,id="):
                            cid = line.split("=")[-1]
                            self._log_message(f"Classification received for Copra #{cid}")

                        # === STANDARD CLEAR ===
                        elif line == "ACK,CLEAR_STANDARD":
                            display_id = str(int(self.last_sorted_id) + 1).zfill(4)
                            self._log_message(f"Moving Copra #{display_id} → Standard (conveyor forward)")
                            self.sorted_count += 1
                            self.check_queue_complete()

                        # === SORTING ACTIONS ===
                        elif line.startswith("ACK,SORT,L"):
                            parts = line.split(",")
                            cid = "??"
                            for p in parts:
                                if "id=" in p:
                                    cid = p.split("=")[1]
                                    break
                            if cid == "-1":
                                continue
                            self.last_sorted_id = cid
                            display_id = str(int(cid) + 1).zfill(4)
                            self._log_message(f"Sorting Copra #{display_id} → Overcooked (left servo)")
                            self.sorted_count += 1
                            self.check_queue_complete()
                        elif line.startswith("ACK,SORT,R"):
                            parts = line.split(",")
                            cid = "??"
                            for p in parts:
                                if "id=" in p:
                                    cid = p.split("=")[1]
                                    break
                            if cid == "-1":
                                continue
                            self.last_sorted_id = cid
                            display_id = str(int(cid) + 1).zfill(4)
                            self._log_message(f"Sorting Copra #{display_id} → Raw (right servo)")
                            self.sorted_count += 1
                            self.check_queue_complete()

                        # === CLASS ECHO FROM ARDUINO ===
                        elif line.startswith("ACK,CLASS,"):
                            parts = line.split(',')
                            if len(parts) >= 3:
                                cls = parts[2]
                                remaining = ','.join(parts[3:])
                                cid = "??"
                                extra = ""
                                if 'id=' in remaining:
                                    id_part = remaining.split('id=')[1]
                                    cid = id_part.split()[0]  # Take until space for extras like "(default ...)"
                                    extra = remaining.replace(f",id={cid}", "")
                                elif ' (' in remaining:  # For extras like "(default failsafe)"
                                    extra = remaining
                                else:
                                    cid = None
                                cls = cls.strip().upper().replace('-COPRA', '')  # Clean for consistency
                                log_msg = f"Arduino echoed class: {cls}"
                                if cid != "??":
                                    log_msg += f",id={cid}"
                                if extra:
                                    log_msg += f" {extra}"
                                self._log_message(log_msg)
                                if cid != "??" and self.last_sent_id == cid and self.last_sent_class:
                                    if self.last_sent_class.lower() not in cls.lower():
                                        self._log_message(f"Warning: Sent class {self.last_sent_class} for ID {cid} not matched in echo.")

                        # === EXISTING MESSAGES ===
                        elif line == "ERR,MOTOR_TIMEOUT":
                            self._log_message("Error: Camera sensor missed object. Check alignment/distance.")
                        elif line.startswith("ERR,FIFO_FULL"):
                            self._log_message("Error: Arduino queue full. System halted.")
                            self._reset_stats()
                        elif line == "ACK,PING":
                            print("Arduino PING OK")
                        elif line.startswith("ACK,CAM_DIST,"):
                            dist = line.split(",")[-1]
                            self._log_message(f"Camera US distance: {dist}cm", console_only=True)
                        elif line == "ERR,CAM_SENSOR_FAIL":
                            self._log_message("CRITICAL: Camera ultrasonic sensor failed! Check wiring/hardware.")
                        elif line == "ACK,HEARTBEAT":
                            print("Arduino heartbeat OK")
                        elif line == "ERR,MISSED_CAM":
                            self._log_message("Missed camera detection – check sensor/copra alignment")
                        elif line == "ERR,SYSTEM_PAUSED":
                            self._log_message("CRITICAL: Arduino system paused due to errors. Reset required.")
                            self.stop_process()

                    time.sleep(0.01)
                except Exception as e:
                    if not self.serial_error_logged:
                        self._log_message(f"Serial reader error: {e}")
                        self.serial_error_logged = True
                    time.sleep(0.2)
        except Exception as outer:
            self._log_message(f"Serial reader crashed: {outer}")

    def perform_classification(self, id=None):
        # This is kept for compatibility, but since Arduino doesn't send "ACK,AT_CAM", it's not triggered. Detection is now in camera_loop.
        pass

    def send_from_queue(self, id):
        if self.copra_queue:
            item_id, class_str, moisture = self.copra_queue.popleft()

            category = class_str.lower() + '-copra'
            display_id = str(item_id) if item_id is not None else str(id)
            self._log_message(f"Class: {class_str} (ID: {display_id})")
            self.root.after(MOISTURE_PRINT_DELAY_MS, lambda m=moisture: self._log_message(f"Moisture: {m:.2f}%"))

            success = self.send_cmd(f"{class_str},{id}")
            if success:
                self.stats[category] += 1
                self.stats['total'] += 1
                self.moisture_sums[category] += moisture
                self.root.after(0, self.update_stats)
            else:
                self._log_message("Send failed → Defaulting to OVERCOOKED")
                self.send_cmd(f"OVERCOOKED,{id}")
        else:
            self._log_message("Queue empty → Sending DEFAULT to Arduino")
            self.send_cmd(f"DEFAULT,{id}")

    def check_queue_complete(self):
        if self.sorted_count >= len(self.INITIAL_COPRA_LIST):
            self.root.after(QUEUE_CLEAR_DELAY_MS, self.stop_process)

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
            self.sorted_count = 0
            # Detection and sending now handled in camera_loop
        except Exception as e:
            self._log_message(f"Start process error: {e}")

    def stop_process(self):
        try:
            self.send_cmd("AUTO_DISABLE")
            self.stats['end_time'] = time.time()
            self.serial_error_logged = False
            self.frame_drop_logged = False
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
        object_id_counter = 0  # Simple incremental ID for sends
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

                    # Detect object and send from queue if debounced
                    if self.process_running and self.last_results and self.last_results[0].boxes and current_time - self.last_detection_time >= OBJECT_DEBOUNCE_S:
                        self._log_message("Object detected in camera preview → Sending from queue")
                        self.send_from_queue(object_id_counter)
                        object_id_counter += 1
                        self.last_detection_time = current_time

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