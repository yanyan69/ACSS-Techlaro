#!/usr/bin/env python3
"""
ACSS GUI with Tabs, Process Control, and Servo Fixes (Simplified).
- Optimized for 95 RPM conveyor at 80% speed.
- Reacts to Arduino's ACK,AT_CAM with YOLO classification, motor runs continuously.
- Suitable for controlled scenarios with one copra at a time.
- AS7263 used as light indicator, handled by Arduino.
- Suppressed spammy logs, full screen, reset stats.
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
CAM_PREVIEW_SIZE = (640, 480)  # Matches Arduino-timed conveyor
USERNAME = "Copra Buyer 01"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200  # Matches Arduino
YOLO_MODEL_PATH = "my_model/my_model.pt"  # Using yolov11n.pt

# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System (Simplified)")
        # Set full screen for 1024x600
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
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.serial_reader_thread_obj = None
        self.serial_error_logged = False  # Prevent serial error spam
        self.frame_drop_logged = False  # Prevent frame drop spam
        self.moisture_sums = {'Raw': 0.0, 'Standard': 0.0, 'Overcooked': 0.0}
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
        self.copra_counter = 0  # Counter for copra numbering

        # Load YOLO
        if ULTRALYTICS_AVAILABLE:
            try:
                print("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                print(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message(f"YOLO load failed: {ex}", console_only=True)

        # Auto-start serial and camera
        self.open_serial()
        if PICAMERA2_AVAILABLE:
            self.start_camera()
        else:
            self.process_btn.config(state='disabled')

        if not SERIAL_AVAILABLE:
            self.process_btn.config(state='disabled')

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

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
                self._log_message("Open serial first.", console_only=True)
                return
            if not self.yolo:
                self._log_message("YOLO model not loaded.", console_only=True)
                return
            if not self.camera_running:
                self._log_message("Camera not running.", console_only=True)
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
        print(msg)  # Always print to console for debugging
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

        # Reset button
        tk.Button(stats_frame, text="Reset Statistics", font=("Arial", 10), command=self._reset_stats).grid(row=row_offset+4, column=0, columnspan=3, pady=10)

    def _reset_stats(self):
        for cat in ["Raw", "Standard", "Overcooked"]:
            self.stats[cat] = 0
            self.moisture_sums[cat] = 0.0
        self.stats['total'] = 0
        self.stats['start_time'] = None
        self.stats['end_time'] = None
        self.copra_counter = 0
        self.update_stats()

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
                self._log_message("pyserial not available.", console_only=True)
                return
            if self.serial and self.serial.is_open:
                self._log_message("Serial already open.", console_only=True)
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self._log_message(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}", console_only=True)
            self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
            self.serial_reader_thread_obj.start()
            self._log_message("Serial reader thread started.", console_only=True)
        except Exception as e:
            self._log_message(f"Failed to open serial: {e}", console_only=True)

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                # Send RESET to clean up Arduino state
                self.send_cmd("RESET")
                self.serial.close()
                self._log_message("Serial closed.", console_only=True)
        except Exception as e:
            self._log_message(f"Error closing serial: {e}", console_only=True)

    def send_cmd(self, text):
        """Send any general command to Arduino with newline + flush"""
        try:
            if not self.serial or not self.serial.is_open:
                self._log_message(f"Serial not open — can't send: {text}", console_only=True)
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
        except Exception as e:
            self._log_message(f"Failed to send command '{text}': {e}", console_only=True)

    def serial_reader_thread(self):
        self._log_message("Serial reader started.", console_only=True)
        try:
            while self.serial and self.serial.is_open and self.running:
                try:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    print(f"ARDUINO: {line}")  # Print all serial lines to console for debugging
                    if line.startswith("ACK,AT_CAM"):
                        # Trigger classification on AT_CAM
                        self.perform_classification()
                    # Ignore other messages (e.g., AS:, TRIG:, ERR:, DBG:)
                except Exception as e:
                    if not self.serial_error_logged:
                        self._log_message(f"Serial reader exception: {e}", console_only=True)
                        self.serial_error_logged = True
                    time.sleep(0.2)
        except Exception as outer:
            self._log_message(f"Serial reader crashed: {outer}", console_only=True)

    def perform_classification(self):
        """Perform YOLO detection and send classification to Arduino."""
        try:
            with self.frame_lock:
                if self.latest_frame is None:
                    self._log_message("No frame available for classification.", console_only=True)
                    return
                frame = self.latest_frame.copy()

            results = self.yolo.predict(
                source=frame,
                conf=0.3,
                max_det=3,
                verbose=False
            )

            has_detection = results and results[0].boxes and len(results[0].boxes) > 0

            if has_detection:
                cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                conf_tensor = results[0].boxes.conf.cpu().numpy()
                candidates = []
                for i in range(len(cls_tensor)):
                    cls = cls_tensor[i]
                    conf = conf_tensor[i]
                    if conf > 0.3:
                        # Adjusted moisture ranges
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
                    # Pick the one with highest confidence
                    candidates.sort(key=lambda x: x[1], reverse=True)
                    leading = candidates[0]
                    cls, conf, moisture = leading
                    category = self.category_map.get(cls, 'Overcooked')
                    class_str = category.upper()
                    self.send_cmd(class_str)
                    self.copra_counter += 1
                    self._log_message(f"{category} Copra # {self.copra_counter:04d}")
                    self._log_message(f"Moisture: {moisture}%")
                    # Update stats immediately since classification sent
                    self.stats[category] += 1
                    self.stats['total'] += 1
                    self.moisture_sums[category] += moisture
                    self.root.after(0, self.update_stats)
        except Exception as e:
            self._log_message(f"Classification error: {e}", console_only=True)

    def start_process(self):
        try:
            if not (self.serial and self.serial.is_open):
                self._log_message("Open serial first.", console_only=True)
                return
            if not self.yolo:
                self._log_message("YOLO model not loaded.", console_only=True)
                return
            self.send_cmd("AUTO_ENABLE")
            self.stats['start_time'] = time.time()
            self.stats['end_time'] = None
            self.serial_error_logged = False
            self.frame_drop_logged = False
            self._log_message("Process started: Auto mode enabled.", console_only=True)
            self.root.after(0, self.update_stats)
        except Exception as e:
            self._log_message(f"Start process error: {e}", console_only=True)

    def stop_process(self):
        try:
            self.send_cmd("AUTO_DISABLE")
            self.stats['end_time'] = time.time()
            self.serial_error_logged = False
            self.frame_drop_logged = False
            self._log_message("Process stopped: Auto mode disabled.", console_only=True)
            self.root.after(0, self.update_stats)
        except Exception as e:
            self._log_message(f"Stop process error: {e}", console_only=True)

    def start_camera(self):
        if self.camera_running:
            self._log_message("Camera already running.", console_only=True)
            return
        if not PICAMERA2_AVAILABLE:
            self._log_message("picamera2 not available.", console_only=True)
            self.process_btn.config(state='disabled')
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": CAM_PREVIEW_SIZE}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)  # Warm-up as in demo code
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self._log_message("Camera started.", console_only=True)
        except Exception as e:
            self._log_message(f"Camera start failed: {e}", console_only=True)
            self.process_btn.config(state='disabled')

    def stop_camera(self):
        try:
            if not self.camera_running:
                self._log_message("Camera not running.", console_only=True)
                return
            self.camera_running = False
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self._log_message("Camera stopped.", console_only=True)
        except Exception as e:
            self._log_message(f"Camera stop error: {e}", console_only=True)

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
                # Suppress frame drop logs
                if current_time - last_frame_time > 0.2:
                    if not self.frame_drop_logged:
                        self._log_message(f"Frame drop detected: {current_time - last_frame_time:.3f}s", console_only=True)
                        self.frame_drop_logged = True
                last_frame_time = current_time

                with self.frame_lock:
                    self.latest_frame = frame

                # Update display every other frame
                display_skip = (display_skip + 1) % 2
                if display_skip == 0:
                    display_frame = frame.copy()  # No YOLO annotation needed
                    display_frame = cv2.resize(display_frame, CAM_PREVIEW_SIZE)
                    self.update_canvas_with_frame(display_frame)

                frame_counter += 1
                if frame_counter >= 50 and current_time - last_fps_log > 5.0:
                    fps = frame_counter / (current_time - fps_time)
                    self._log_message(f"FPS: {fps:.1f}", console_only=True)
                    fps_time = time.time()
                    frame_counter = 0

                time.sleep(0.01)
            except Exception as e:
                self._log_message(f"Camera loop error: {e}", console_only=True)
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
            self._log_message(f"Display frame error: {e}", console_only=True)

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
            self._log_message(f"Stats update error: {type(e).__name__}: {str(e)}", console_only=True)

    def on_close(self):
        self.running = False
        self.process_running = False
        self.camera_running = False
        self.stop_process()
        self.stop_camera()
        self.close_serial()
        self._log_message("Application closed.", console_only=True)
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()