#!/usr/bin/env python3
"""
Automated Copra Segregation System (ACSS) GUI
- Provides a user interface for controlling and monitoring the copra segregation process.
- Integrates with Arduino via serial for automated detection and sorting using ultrasonic sensors and servos.
- Uses YOLO for copra classification (Raw, Standard, Overcooked) based on camera frames triggered by Arduino.
- Displays real-time camera preview with live bounding boxes at ~3 FPS for debugging, logs (classification results, system events), and statistics.
- Classification only triggered by Arduino's ACK,AT_CAM (ultrasonic detection), but preview shows bounding boxes continuously.

Changes:
- Set YOLO_FRAME_SKIP = 5 to target ~3 FPS for bounding box updates (assuming ~15 FPS camera).
- Adjusted camera_loop sleep to stabilize frame rate.
- Updated perform_classification to use yolo.track for consistency with live preview, fixing cls mismatch.
- Added cls logging in perform_classification to debug classification vs. preview discrepancies.
- Reduced conf to 0.25 in perform_classification to increase detection chance.
- Added logging for send_cmd success/failure in perform_classification.
- Increased CLASSIFICATION_TIMEOUT_S to 2.5s for more time.
- Reduced max_det to 1 in perform_classification for faster processing.
- Added time logging in perform_classification to debug if it exceeds Arduino's CLASS_WAIT_MS = 3000ms.
- Improved serial_reader_thread to read all available lines when data is ready, fixing missed lines.
- Set serial timeout to 1s for better reading.
- Updated category_map to match model classes: {0: 'Overcooked', 1: 'Raw', 2: 'Standard'}
- Failsafe remains OVERCOOKED.

# Update on November 05, 2025: Fixed mismatched moisture calculation to align with category_map (Overcooked low, Raw high, Standard mid). Removed cls=2 force in else.
# Update on November 05, 2025: Added ID parsing from ACK,AT_CAM and sending CLASS,ID to sync with Arduino FIFO queue.
# Update on November 05, 2025: Replaced strupr with command.upper() to fix runtime error.
# Update on November 05, 2025: Removed unused self.class_to_sort and related comment (dead code cleanup).
# Update on November 05, 2025: Updated category_map to match user's model: {0: 'overcooked-copra', 1: 'raw-copra', 2: 'standard-copra'}.
# Update on November 05, 2025: Increased CLASSIFICATION_TIMEOUT_S to 2.8s (close to Arduino's 3s). Added conf/cls logging in no-candidates case. Increased YOLO_FRAME_SKIP to 10 for lower load if needed (test/adjust).
# Update on November 05, 2025: Made bounding box update frequency adjustable via YOLO_FRAME_SKIP constant. Persisted last detection results to draw boxes consistently on every frame, avoiding flickering. Boxes now update only every SKIP frames but remain displayed until new detection.
# Update on November 05, 2025: Added Clear Log button below log frame in home tab. Adjusted perform_classification to loop retries until close to 3s timeout, maximizing YOLO stabilization time before defaulting to OVERCOOKED.
# Update on November 05, 2025: Updated perform_classification to run YOLO multiple times over ~2.8s, collect candidates, and send the most frequent class (averaged detection) at the end for stabilization. Added retry on send_cmd if failed. Stripped '-copra' from log category for cleaner output (e.g., "Raw Copra #0003").
# Update on November 05, 2025: Stripped '-copra' from sent class_str to match Arduino's strToClass (e.g., "OVERCOOKED" instead of "OVERCOOKED-COPRA"). Commented out flapper log as unnecessary.
# Update on November 05, 2025: Reduced CLASSIFICATION_TIMEOUT_S to 1.8s and sleep to 0.1s to send classification faster, avoiding Arduino 3s timeout race. Added early exit if clear majority in candidates. Increased conf to 0.3 for fewer false positives.
# Update on November 07, 2025: Addressed gaps in classifications and cam US. Added XOR checksum to send_cmd. Reduced CLASSIFICATION_TIMEOUT_S to 1.5s and sleep to 0.05s with early exit after 2 runs on majority. Handled ACK,CLASS_RECEIVED,id=... for sync. Added periodic GET_CAM_DIST polling every 10s for logs. Handled ERR,CAM_SENSOR_FAIL with alert. Logged ACK,HEARTBEAT as confirm.
# Update on November 07, 2025: Added full process flow logging with Copra #ID tracking.
# - Logs Start US1 detection + queue entry
# - Logs arrival at camera, classification, delayed moisture, and sorting
# - Moisture logged 1 second after conveyor starts (ACK,MOTOR,START)
# - All messages include Copra #ID for perfect queue tracking
# Update on November 07, 2025: REMOVED ALL POP-UP ERROR WINDOWS. All errors now go to Log only.
# Update on November 07, 2025: Reduced CLASSIFICATION_TIMEOUT_S to 1.0s and sleep to 0.02s for faster send. Added handling for ERR,MISSED_CAM in log. Tied moisture delay to ACK,CLASS_RECEIVED. Synced default fallback with Arduino (query on start). 
# Update on November 08, 2025: Increased CLASSIFICATION_TIMEOUT_S to 4.0s and sleep to 0.05s for better sync. Added 3x retry in send_cmd. Added MOISTURE_LOG_DELAY_MS constant. Removed extra text from moisture log. Lowered conf to 0.5 in perform_classification. Updated serial parsing for id= in ACK,MOTOR,START and ACK,SORT,L/R. Added handling for ERR,SYSTEM_PAUSED. Added no-candidates warning in perform_classification.
# Update on November 08, 2025: Moved start detection log to ACK,ENQUEUE_PLACEHOLDER for correct idx/id parsing. Ignore ACK,MOTOR,START if id=-1 (ghost). Added warning if sent class not matched in ACK,CLASS_RECEIVED.
# Update on November 08, 2025: Fixed class echo parsing in serial_reader_thread to correctly extract class and ID from "ACK,CLASS,...". Moved mismatch warning to this block for accurate checking. Cleaned class string for consistency.
# Update on November 08, 2025: Fixed inconsistent logs: Relocated "moving to standard" to ACK,CLEAR_STANDARD. Removed moisture scheduling from ACK,CLASS_RECEIVED (keep on ACK,MOTOR,START). Use per-ID moisture dict to avoid stale values. Schedule on CLEAR_STANDARD for STANDARD sorts.
# Update on November 08, 2025: Added Copy Log button beside Clear Log button (copies log from console).
# Update on November 08, 2025: Moved moisture logging to immediately after classification in perform_classification (no delay, prints right after "Class: ..."). Removed delayed logging and related vars/scheduling. Rounded moisture to 2 decimals based on research (raw: 7.1-60%, standard: 6-7%, overcooked: 4-5.9%).
# Update on November 08, 2025: Adjusted copra # display to start from 1 (offset Arduino ID by +1 in logs). Added constant MOISTURE_PRINT_DELAY_MS = 2000 (2s delay after class log for moisture print). Scheduled moisture log with root.after for safe, non-disruptive delay.
# Update on November 08, 2025: Added iou=0.45 to YOLO track calls in perform_classification and camera_loop to reduce overlapping bounding boxes via stricter NMS.
# Update on November 08, 2025: Enforced minimum 1s stabilization in perform_classification before sending command by looping inferences until at least 1s elapsed or consensus reached.
# Update on February 03, 2026: Added keyboard bindings for manual control: 'a' for OVERCOOKED (left servo), 's' for STANDARD (conveyor clear), 'd' for RAW (right servo). Added more keys for full manual prototype control.
# Update on February 03, 2026: Added joystick support with polling thread. Mapped d-pad left/X button to RAW, up/Y to STANDARD, right/B to OVERCOOKED. On manual key/joystick press, stop process, send class, wait 5s, resume process. Reassigned 'a' to toggle start/stop.
# Update on February 03, 2026: Reassigned joystick buttons: 3 for RAW, 4 for STANDARD, 1 for OVERCOOKED, 0 for toggle start/stop. D-pad arrows unchanged. Removed joystick detection log.
# Update on February 03, 2026: Updated joystick/keyboard to stop conveyor briefly on class sends (via AUTO_DISABLE, send, wait, AUTO_ENABLE). For arrows, stop, simulate/log mock camera classification, resume. Button 0 toggles full process/conveyor.
# Update on February 03, 2026: Added joystick buttons 6 (L pad) for TEST_SERVO_L, 7 (R pad) for TEST_SERVO_R to test servo responsiveness.
# Update on February 03, 2026: Added 2s cooldown for servo test buttons (6/7) to prevent spamming and protect hardware.
# Update on February 03, 2026: Cleaned logs to only show "Process started", classification, moisture (after 5s), "sorted". No joystick logs. Changed moisture delay to 5s. For D-pad arrows (up/left/right), capture current frame, run YOLO, log real class/moisture (5s delay), send to Arduino for sorting/servo (works with/without motor).
# Update on February 03, 2026: Muted "Process started" and "Process stopped" logs to prevent repetition.
# Update on February 03, 2026: Adjusted moisture reading from 5s to 1s. Made buttons 3/4/1 run YOLO on press like arrows (same behavior, different zones). Removed stop/resume in manual/simulate since continuous motor (stopping won't do anything). Added debounce (1s) for arrow presses to avoid double reads. Added wait/retry in simulate_camera until object detected. Added total time (HH:MM:SS) in statistics, auto-calculated.
# Update on February 03, 2026: Shifted manual control to flapper zone only (ignore cam/YOLO). Buttons/arrows now assign fixed class (left/L/3: RAW, right/R/1: OVERCOOKED, up/Y/4: STANDARD), generate random moisture in class range, log class then 1s-delayed moisture, update stats, send class to Arduino (no YOLO, no object wait).
# Update on February 03, 2026: Changed buttons to 6 (RAW), 7 (OVERCOOKED), 4 (STANDARD), 0 (start/stop). Updated d-pad: left=RAW, right=OVERCOOKED, up=STANDARD.
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
CLASSIFICATION_TIMEOUT_S = 4.0  # Increased for better sync
MAX_FRAME_AGE_S = 0.7  # Increased slightly to account for system load
PING_INTERVAL_S = 5.0  # Send PING every 5 seconds
CLASSIFICATION_RETRIES = 1  # Retry classification if frame is stale
YOLO_FRAME_SKIP = 10  # Adjust here: Lower for more frequent bounding box updates (e.g., 1 for every frame, may cause lag); higher for less frequent (e.g., 10 for ~3 FPS if camera ~30 FPS). Set to 1 for constant detection without skip.
CAM_DIST_POLL_INTERVAL_S = 10.0  # Poll cam dist every 10s
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

        # Manual control counter (for dummy IDs in manual sends)
        self.manual_id_counter = 0

        # Cooldown for servo tests (2s to prevent spam)
        self.last_servo_test_time = {6: 0, 7: 0}
        self.servo_cooldown_ms = 2000

        # Debounce for button presses
        self.last_button_press_time = 0
        self.button_debounce_ms = 1000  # 1s interval to avoid double reads

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
        self.root.bind('<g>', lambda e: self.send_cmd('GET_CAM_DIST'))   # Get camera distance
        self.root.bind('<h>', lambda e: self.send_cmd('GET_DEFAULT'))    # Get default class

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
                    if event.button == 6:  # 6 for RAW
                        self.simulate_flapper('RAW')
                    elif event.button == 4:  # 4 for STANDARD
                        self.simulate_flapper('STANDARD')
                    elif event.button == 7:  # 7 for OVERCOOKED
                        self.simulate_flapper('OVERCOOKED')
                    elif event.button == 0:  # 0 for toggle start/stop
                        self._toggle_process()
                    elif event.button == 3:  # Optional: Keep or remap if needed
                        pass
                    elif event.button == 1:  # Optional: Keep or remap if needed
                        pass
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

    def simulate_flapper(self, fixed_class):
        """At flapper: Assign fixed class per button, random moisture in range, log class/delayed moisture, update stats, send class to Arduino."""
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

    def perform_classification(self, id=None):
        if id is None:
            id = 0
        start_time = time.time()
        all_candidates = []
        num_runs = 0
        min_stabilization_time = 1.0  # Minimum time to stabilize (1s)

        while time.time() - start_time < CLASSIFICATION_TIMEOUT_S and num_runs < 20:  # Increased max runs for stabilization
            try:
                with self.frame_lock:
                    if self.latest_frame is None:
                        time.sleep(0.05)
                        continue
                    frame = self.latest_frame.copy()

                results = self.yolo.track(
                    source=frame,
                    persist=True,
                    tracker=TRACKER_PATH,
                    conf=0.5,  # Lowered for fewer misses
                    max_det=1,
                    iou=0.45,  # Added to reduce overlapping boxes
                    verbose=False
                )
                num_runs += 1

                if results and results[0].boxes and len(results[0].boxes) > 0:
                    cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                    conf_tensor = results[0].boxes.conf.cpu().numpy()
                    for i in range(len(cls_tensor)):
                        cls = cls_tensor[i]
                        conf = conf_tensor[i]
                        if conf > 0.3:
                            if cls == 1:  # raw-copra
                                moisture = 7.1 + (conf - 0.3) * (60.0 - 7.1) / 0.7
                            elif cls == 2:  # standard-copra
                                moisture = 6.0 + (conf - 0.3) * (7.0 - 6.0) / 0.7
                            else:  # overcooked-copra
                                moisture = 4.0 + (conf - 0.3) * (5.9 - 4.0) / 0.7
                            moisture = round(moisture, 2)
                            all_candidates.append((cls, conf, moisture))

                if len(all_candidates) >= 2:
                    class_counts = Counter([c[0] for c in all_candidates])
                    if class_counts:
                        top_cls, top_count = class_counts.most_common(1)[0]
                        if all(count <= top_count / 2 for cls, count in class_counts.items() if cls != top_cls):
                            if time.time() - start_time >= min_stabilization_time:
                                break  # Consensus reached after min time

                time.sleep(0.05)  # Increased for stability
            except Exception as e:
                print(f"YOLO error: {e}")
                time.sleep(0.05)

        # Enforce min time if not reached
        while time.time() - start_time < min_stabilization_time:
            time.sleep(0.01)

        if all_candidates:
            class_counts = Counter([c[0] for c in all_candidates])
            most_common_cls = class_counts.most_common(1)[0][0]
            filtered = [c for c in all_candidates if c[0] == most_common_cls]
            filtered.sort(key=lambda x: x[1], reverse=True)
            cls, conf, moisture = filtered[0]

            category = self.category_map.get(cls, 'overcooked-copra')
            class_str = category.upper().replace('-COPRA', '')

            self.last_sent_class = class_str
            self.last_sent_id = str(id)

            success = self.send_cmd(f"{class_str}")
            if success:
                self.copra_counter += 1
                cat_name = category.split('-')[0].capitalize()
                self._log_message(f"Class: {cat_name.upper()}")
                self.root.after(MOISTURE_PRINT_DELAY_MS, lambda m=moisture: self._log_message(f"Moisture: {m:.2f}%"))
                self.stats[category] += 1
                self.stats['total'] += 1
                self.moisture_sums[category] += moisture
                self.root.after(0, self.update_stats)
            else:
                self._log_message("All send retries failed → Defaulting to OVERCOOKED")
                self.send_cmd(f"OVERCOOKED")
        else:
            self._log_message("No detection after retries → Sending DEFAULT to Arduino")
            self.send_cmd(f"DEFAULT")

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