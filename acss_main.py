#!/usr/bin/env python3
"""
ACSS GUI Skeleton with Tabs (Modern Design + Frames + Log + About + Exit)
Integrated with process control, auto-start camera and model, object tracking, and FIFO sorting.
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
CAM_PREVIEW_SIZE = (640, 480)
USERNAME = "Copra Buyer 01"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"  # Custom model; use "yolov11n.pt" for lightweight testing
TRACKER_PATH = "bytetrack.yaml"  # Assume available
SORT_ZONE_Y = 200  # Top third of 480px frame for sorting
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

        # Load YOLO if available
        if ULTRALYTICS_AVAILABLE:
            try:
                self._log_message("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self._log_message(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message(f"YOLO load failed: {ex}")

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
        frm.columnconfigure(0, weight=3)  # Left side wider for camera
        frm.columnconfigure(1, weight=2)  # Right side narrower for log

        # Left side: Camera + Button
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

        # Right side: Log (adjusted size, with word wrap)
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
        for cat in ["Standard", "Raw", "Overcooked"]:
            self.stat_pieces[cat].config(text=str(self.stats[cat]))
        self.total_pieces_label.config(text=str(self.stats['total']))
        start_str = time.strftime("%H:%M:%S", time.localtime(self.stats['start_time'])) if self.stats['start_time'] else "N/A"
        end_str = time.strftime("%H:%M:%S", time.localtime(self.stats['end_time'])) if self.stats['end_time'] else "N/A"
        self.start_time_label.config(text=start_str)
        self.end_time_label.config(text=end_str)

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
        except Exception as e:
            self._log_message(f"Failed to open serial: {e}")

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
        try:
            with self.serial_lock:
                self.serial.write((text + "\n").encode())
                self.serial.flush()  # Ensure immediate send
            self._log_message(f"TX -> {text}")
        except Exception as e:
            self._log_message(f"Serial write failed: {e}")

    def send_sort(self, char):
        if char not in ('L', 'C', 'R'):
            self._log_message(f"Invalid sort direction: {char}")
            return
        self._log_message(f"Sending SORT,{char}")
        self.send_cmd(f"SORT,{char}")

    def serial_reader_thread(self):
        self._log_message("Serial reader started.")
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                # Only log expected messages (AS: or ACK); ignore others (e.g., IR or noise)
                if line.startswith("AS:"):
                    vals = line[3:].split(",")
                    self._log_message(f"RX <- AS values: {vals}")
                elif line == "ACK":
                    self._log_message("RX <- ACK from Arduino")
                # Silently ignore unexpected messages to prevent log spam
            except Exception as e:
                self._log_message(f"Serial reader exception: {e}")
                time.sleep(0.1)

    # ---------- Process Control ----------
    def start_process(self):
        self.stats['start_time'] = time.time()
        self.stats['end_time'] = None
        self.send_cmd("MOTOR,ON")
        self.sort_cooldown = time.time()
        self.sorted_tracks.clear()
        self.track_start_times.clear()
        self.root.after(0, self.update_stats)

    def stop_process(self):
        self.send_cmd("MOTOR,OFF")
        self.stats['end_time'] = time.time()
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
                main={"format": 'XRGB8888', "size": (640, 480)}
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
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                results = None
                if self.yolo:
                    if self.process_running:
                        results = self.yolo.track(
                            source=frame,
                            persist=True,
                            tracker=TRACKER_PATH,
                            conf=0.25,
                            max_det=5,
                            verbose=False
                        )
                    else:
                        results = self.yolo.predict(
                            source=frame,
                            conf=0.25,
                            max_det=5,
                            verbose=False
                        )
                    if results and results[0].boxes is not None:
                        frame = results[0].plot()

                if self.process_running and results and results[0].boxes and len(results[0].boxes) > 0 and time.time() > self.sort_cooldown:
                    track_ids = results[0].boxes.id.cpu().numpy().astype(int) if results[0].boxes.id is not None else []
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                    conf_tensor = results[0].boxes.conf.cpu().numpy()

                    if len(track_ids) < len(cls_tensor):
                        self._log_message(f"Warning: Track ID mismatch ({len(track_ids)} IDs vs {len(cls_tensor)} boxes)")

                    current_time = time.time()
                    candidates = []
                    for i in range(len(cls_tensor)):
                        track_id = track_ids[i] if i < len(track_ids) else -1
                        if track_id not in self.sorted_tracks and track_id != -1:
                            y_center = (boxes[i, 1] + boxes[i, 3]) / 2
                            if y_center < SORT_ZONE_Y:  # Sort zone: top third
                                cls = cls_tensor[i]
                                conf = conf_tensor[i]
                                if conf > 0.25:
                                    if track_id not in self.track_start_times:
                                        self.track_start_times[track_id] = current_time
                                    candidates.append((track_id, cls, conf, y_center))

                    if candidates:
                        candidates.sort(key=lambda x: (self.track_start_times[x[0]], x[3]))
                        leading = candidates[0]
                        track_id, cls, conf, y_center = leading
                        sort_char = self.class_to_sort.get(cls, 'C')
                        self.send_sort(sort_char)
                        category = self.category_map.get(cls, 'Standard')
                        self.stats[category] += 1
                        self.stats['total'] += 1
                        self._log_message(f"Sorted {category} (class {cls}, conf {conf:.2f}, track {track_id}, y={y_center:.1f}) to {sort_char}")
                        self.sorted_tracks.add(track_id)
                        self.sort_cooldown = current_time + 1.5
                        self.root.after(0, self.update_stats)

                    if current_time - self.last_cleanup > 10.0:
                        if len(results[0].boxes) == 0:
                            self.sorted_tracks.clear()
                            self.track_start_times.clear()
                        self.last_cleanup = current_time

                frame_counter += 1
                if frame_counter >= 10:
                    fps = 10 / (time.time() - fps_time)
                    fps_time = time.time()
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                    frame_counter = 0

                display_frame = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display_frame)

                time.sleep(1/30)
            except Exception as e:
                self._log_message(f"Camera loop error: {e}")
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGRA2RGB)
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
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()