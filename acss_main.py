#!/usr/bin/env python3
"""
ACSS GUI with Tabs, Process Control, YOLO Tracking, FIFO Sorting, and Servo Fixes.
AS7263 light disabled, IR logging removed, fixed track mismatches, ignored copra,
Tkinter config error, and mitigated PDAF errors.
Optimized for 95 RPM conveyor at 80% speed.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time
from collections import deque  # For sort queue

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
CAM_PREVIEW_SIZE = (640, 480)  # Try (320, 240) if PDAF errors persist
USERNAME = "Copra Buyer 01"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"  # Use "yolov11n.pt" for lightweight testing
TRACKER_PATH = "bytetrack.yaml"  # Assume in project root
SORT_ZONE_Y = 350  # Top ~73% of 480px frame for primary sorting
FALLBACK_ZONE_Y = 400  # Secondary zone for untracked copra
# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System")
        root.geometry("1024x600")

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
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.serial_reader_thread_obj = None
        self.sort_cooldown = 0
        self.sorted_tracks = set()
        self.track_start_times = {}  # FIFO: Track ID -> creation time
        self.last_cleanup = time.time()
        self.class_to_sort = {0: 'L', 1: 'C', 2: 'R'}
        self.category_map = {0: 'Raw', 1: 'Standard', 2: 'Overcooked'}
        self.stats = {
            'Raw': 0,
            'Standard': 0,
            'Overcooked': 0,
            'total': 0,
            'start_time': None,
            'end_time': None
        }
        self.sort_queue = deque()  # Queue for pending sorts
        self.serial_error_logged = False  # Prevent serial error spam

        # Load YOLO if available
        if ULTRALYTICS_AVAILABLE:
            try:
                self._log_message("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self._log_message(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message(f"YOLO load failed: {ex}")

        # AS7263 state
        self.as7263_data = None  # Latest AS7263 reading
        self.as7263_timestamp = 0  # Timestamp of last reading
        self.moisture_sums = {'Raw': 0.0, 'Standard': 0.0, 'Overcooked': 0.0}  # For avg moisture
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Auto-start serial and camera
        self.open_serial()
        if PICAMERA2_AVAILABLE:
            self.start_camera()
        else:
            self.process_btn.config(state='disabled')

        # Disable process button if no serial
        if not SERIAL_AVAILABLE:
            self.process_btn.config(state='disabled')

    # -------------------- HOME --------------------
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
                self._log_message("Open serial first.")
                return
            if not self.yolo:
                self._log_message("YOLO model not loaded.")
                return
            if not self.camera_running:
                self._log_message("Camera not running.")
                return
            self.process_btn.config(text="Stop Process", bg="red")
            self._log_message("Process started.")
            self.start_process()
            self.process_running = True
        else:
            self.process_btn.config(text="Start Process", bg="green")
            self._log_message("Process stopped.")
            self.stop_process()
            self.process_running = False

    def _log_message(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert("end", full_msg + "\n")
        self.log.see("end")
        self.log.config(state='disabled')

    # ----------------- STATISTICS -----------------
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
        except Exception as e:
            self._log_message(f"Stats update error: {e}")

    # ------------------- ABOUT --------------------
    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        about_frame = tk.LabelFrame(frm, text="About ACSS")
        about_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(about_frame,
                text="Automated Copra Segregation System\n\n"
                    "Version: 1.0\n"
                    "Developed for Practice and Design 2\n"
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
        if not SERIAL_AVAILABLE:
            self._log_message("pyserial not available.")
            self.process_btn.config(state='disabled')
            return
        if self.serial and self.serial.is_open:
            self._log_message("Serial already open.")
            return
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self._log_message(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
            self.serial_reader_thread_obj.start()
            self._log_message("Serial reader thread started.")
            self.send_cmd("AS_OFF")
        except Exception as e:
            self._log_message(f"Failed to open serial: {e}")
            self.process_btn.config(state='disabled')

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self._log_message("Serial closed.")
        except Exception as e:
            self._log_message(f"Error closing serial: {e}")

    def send_cmd(self, text):
        if not self.serial or not self.serial.is_open:
            self._log_message(f"Serial not open. Cannot send: {text}")
            return
        for _ in range(2):  # Retry once on failure
            try:
                with self.serial_lock:
                    self.serial.write((text + "\n").encode())
                    self.serial.flush()
                self._log_message(f"TX -> {text}")
                return
            except Exception as e:
                if not self.serial_error_logged:
                    self._log_message(f"Serial write failed: {e}")
                    self.serial_error_logged = True
                time.sleep(0.1)

    def send_sort(self, char):
        if char not in ('L', 'C', 'R'):
            self._log_message(f"Invalid sort direction: {char}")
            return
        self.sort_queue.append(char)
        self._process_sort_queue()

    def _process_sort_queue(self):
        if not self.sort_queue or time.time() < self.sort_cooldown:
            return
        char = self.sort_queue.popleft()
        try:
            cmd = f"SORT,{char}"
            self._log_message(f"Sending SORT,{char}")
            with self.serial_lock:
                self.serial.write((cmd + "\n").encode())
                self.serial.flush()
                t0 = time.time()
                got_ack = False
                while time.time() - t0 < 1.5:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if line == "ACK":
                        self._log_message("RX <- ACK for SORT")
                        got_ack = True
                        break
                if not got_ack:
                    self._log_message(f"No ACK for SORT,{char}")
                    self.sort_queue.append(char)
        except Exception as e:
            if not self.serial_error_logged:
                self._log_message(f"Sort failed: {e}")
                self.serial_error_logged = True
            self.sort_queue.append(char)

    def serial_reader_thread(self):
        self._log_message("Serial reader started.")
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                if line.startswith("AS:"):
                    vals = line[3:].split(",")
                    self.as7263_data = vals  # Store latest AS7263 reading
                    self.as7263_timestamp = time.time()
                    self._log_message(f"RX <- AS values: {vals}")
                elif line == "ACK":
                    self._log_message("RX <- ACK from Arduino")
                # Silently ignore other messages
            except Exception as e:
                if not self.serial_error_logged:
                    self._log_message(f"Serial reader exception: {e}")
                    self.serial_error_logged = True
                time.sleep(0.1)

    # ---------- Process Control ----------
    def start_process(self):
        self.stats['start_time'] = time.time()
        self.stats['end_time'] = None
        self.send_cmd("MOTOR,ON")  # Start conveyor at fixed speed (255 PWM)
        self.sort_cooldown = time.time()
        self.sorted_tracks.clear()
        self.track_start_times.clear()
        self.sort_queue.clear()
        self.serial_error_logged = False
        self.root.after(0, self.update_stats)

    def stop_process(self):
        self.send_cmd("MOTOR,OFF")
        self.stats['end_time'] = time.time()
        self.sort_queue.clear()
        self.serial_error_logged = False
        self.root.after(0, self.update_stats)

    # ---------- Camera Methods ----------
    def start_camera(self):
        if not PICAMERA2_AVAILABLE:
            self._log_message("picamera2 not available.")
            self.process_btn.config(state='disabled')
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": CAM_PREVIEW_SIZE}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self._log_message("Camera started.")
        except Exception as e:
            self._log_message(f"Camera start failed: {e}")
            self.process_btn.config(state='disabled')

    def stop_camera(self):
        try:
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
        last_drop_log = time.time()
        display_skip = 0  # Skip every other frame for display
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                # Check for significant frame drops
                current_time = time.time()
                if current_time - last_frame_time > 0.2 and current_time - last_drop_log > 5.0:  # >200ms, log every 5s
                    self._log_message(f"Frame drop detected: {current_time - last_frame_time:.3f}s")
                    last_drop_log = current_time
                last_frame_time = current_time

                results = None
                if self.yolo:
                    yolo_start = time.time()
                    if self.process_running:
                        results = self.yolo.track(
                            source=frame,
                            persist=True,
                            tracker=TRACKER_PATH,
                            conf=0.3,
                            max_det=3,
                            verbose=False
                        )
                    else:
                        results = self.yolo.predict(
                            source=frame,
                            conf=0.3,
                            max_det=3,
                            verbose=False
                        )
                    yolo_time = time.time() - yolo_start
                    if yolo_time > 0.1:  # Log if YOLO takes >100ms
                        self._log_message(f"YOLO processing time: {yolo_time:.3f}s")

                # Process sorting if results exist and AS7263 validates
                as7263_valid = self.as7263_data is not None and (current_time - self.as7263_timestamp) < 0.5
                if self.process_running and results and results[0].boxes and len(results[0].boxes) > 0 and time.time() > self.sort_cooldown and as7263_valid:
                    track_ids = results[0].boxes.id.cpu().numpy().astype(int) if results[0].boxes.id is not None else []
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                    conf_tensor = results[0].boxes.conf.cpu().numpy()

                    if len(track_ids) < len(cls_tensor):
                        self._log_message(f"Warning: Track ID mismatch ({len(track_ids)} IDs vs {len(cls_tensor)} boxes, boxes={boxes.tolist()})")

                    current_time = time.time()
                    candidates = []
                    untracked_candidates = []
                    for i in range(len(cls_tensor)):
                        track_id = track_ids[i] if i < len(track_ids) else -1
                        y_center = (boxes[i, 1] + boxes[i, 3]) / 2
                        cls = cls_tensor[i]
                        conf = conf_tensor[i]
                        if conf > 0.3:
                            # Estimate moisture based on confidence
                            if cls == 0:  # Raw
                                moisture = 7.1 + (conf - 0.3) * (8.0 - 7.1) / 0.7  # Map 0.3-1.0 to 7.1-8.0%
                            elif cls == 1:  # Standard
                                moisture = 6.0 + (conf - 0.3) * (7.0 - 6.0) / 0.7  # Map 0.3-1.0 to 6.0-7.0%
                            else:  # Overcooked
                                moisture = 5.0 + (conf - 0.3) * (5.9 - 5.0) / 0.7  # Map 0.3-1.0 to 5.0-5.9%
                            moisture = round(moisture, 1)

                            if track_id != -1 and track_id not in self.sorted_tracks and y_center < SORT_ZONE_Y:
                                if track_id not in self.track_start_times:
                                    self.track_start_times[track_id] = current_time
                                candidates.append((track_id, cls, conf, y_center, moisture))
                            elif y_center < FALLBACK_ZONE_Y:
                                self._log_message(f"Untracked detection (class {cls}, conf {conf:.2f}, y={y_center:.1f}, moisture {moisture}%)")
                                untracked_candidates.append((cls, conf, y_center, moisture))

                    for candidate in sorted(candidates, key=lambda x: (self.track_start_times[x[0]], x[3])):
                        track_id, cls, conf, y_center, moisture = candidate
                        sort_char = self.class_to_sort.get(cls, 'C')
                        self.send_sort(sort_char)
                        category = self.category_map.get(cls, 'Standard')
                        self.stats[category] += 1
                        self.stats['total'] += 1
                        self.moisture_sums[category] += moisture
                        self._log_message(f"Sorted {category} (class {cls}, conf {conf:.2f}, track {track_id}, y={y_center:.1f}, moisture {moisture}%) to {sort_char}")
                        self.sorted_tracks.add(track_id)
                        self.sort_cooldown = current_time + 0.3
                        self.root.after(0, self.update_stats)

                    for cls, conf, y_center, moisture in sorted(untracked_candidates, key=lambda x: x[2]):
                        sort_char = self.class_to_sort.get(cls, 'C')
                        self.send_sort(sort_char)
                        category = self.category_map.get(cls, 'Standard')
                        self.stats[category] += 1
                        self.stats['total'] += 1
                        self.moisture_sums[category] += moisture
                        self._log_message(f"Sorted untracked {category} (class {cls}, conf {conf:.2f}, y={y_center:.1f}, moisture {moisture}%) to {sort_char}")
                        self.sort_cooldown = current_time + 0.3
                        self.root.after(0, self.update_stats)

                    if current_time - self.last_cleanup > 5.0:
                        if len(results[0].boxes) == 0:
                            self.sorted_tracks.clear()
                            self.track_start_times.clear()
                        self.last_cleanup = current_time

                # Update display every other frame
                display_skip = (display_skip + 1) % 2
                if display_skip == 0:
                    display_frame = results[0].plot() if results and results[0].boxes else frame
                    display_frame = cv2.resize(display_frame, CAM_PREVIEW_SIZE)
                    self.update_canvas_with_frame(display_frame)

                frame_counter += 1
                if frame_counter >= 50 and current_time - last_fps_log > 5.0:
                    fps = frame_counter / (current_time - fps_time)
                    self._log_message(f"FPS: {fps:.1f}")
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

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
        self.process_running = False
        self.camera_running = False
        try:
            if self.picam2:
                self.picam2.stop()
        except:
            pass
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except:
            pass
        self._log_message("Application closed.")
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()