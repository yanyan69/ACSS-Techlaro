#!/usr/bin/env python3
"""
ACSS GUI Skeleton with Tabs (Modern Design + Frames + Log + About + Exit)
Integrated with features from debug.py for component testing and live status.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time

# Optional imports (graceful degradation)
try:
    import serial
except:
    serial = None
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
USERNAME = "User001"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/train/weights/best.pt"
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
        self.ir_monitoring = False
        self.camera_thread = None
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.ir_detected = False
        self.ir_distance = None
        self.as_values = []
        self.motor_status = False  # Assumed based on last command
        self.servo_position = 'C'  # Assumed initial center
        self.serial_reader_thread_obj = None

        # Load YOLO if available
        if ULTRALYTICS_AVAILABLE:
            try:
                self._log_message("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self._log_message(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message("YOLO load failed: " + str(ex))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start settings update loop
        self.update_settings()

    # -------------------- HOME --------------------
    def _build_home_tab(self):
        frm = tk.Frame(self.home_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=3)  # Left side wider for camera
        frm.columnconfigure(1, weight=2)  # Right side narrower for log to fit

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

        self.start_btn = tk.Button(left_frame, text="Start Camera",
                                   font=("Arial", 14, "bold"), bg="green", fg="white",
                                   command=self._toggle_camera)
        self.start_btn.grid(row=1, column=0, sticky="ew", padx=4, pady=8)

        # Disable if picamera2 unavailable
        if not PICAMERA2_AVAILABLE:
            self.start_btn.config(state='disabled')

        # Right side: Log (adjusted size, with word wrap)
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, state='normal', wrap='word', height=20, width=40)  # Adjusted width and height to fit, wrap='word' for text wrapping
        self.log.pack(fill='both', expand=True)

    def _toggle_camera(self):
        if not self.camera_running:
            self.start_btn.config(text="Stop Camera", bg="red")
            self._log_message("Camera started.")
            self.start_camera()
            self.camera_running = True
        else:
            self.start_btn.config(text="Start Camera", bg="green")
            self._log_message("Camera stopped.")
            self.stop_camera()
            self.camera_running = False

    def _log_message(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.insert("end", full_msg + "\n")
        self.log.see("end")

    # ----------------- STATISTICS -----------------
    def _build_statistics_tab(self):
        frm = tk.Frame(self.statistics_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        stats_frame = tk.LabelFrame(frm, text="Processing Statistics")
        stats_frame.pack(fill="both", expand=True, padx=4, pady=4)

        # Add grid configuration for flexibility
        stats_frame.columnconfigure(0, weight=1)
        stats_frame.columnconfigure(1, weight=1)
        stats_frame.columnconfigure(2, weight=1)

        tk.Label(stats_frame, text=f"Username: {USERNAME}", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)

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

        tk.Label(stats_frame, text="Processing Start Time:", font=("Arial", 10, "bold")).grid(row=row_offset+2, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="N/A").grid(row=row_offset+2, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing End Time:", font=("Arial", 10, "bold")).grid(row=row_offset+3, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="N/A").grid(row=row_offset+3, column=1, padx=8, pady=4)

    # ------------------- SETTINGS -----------------
    def _build_settings_tab(self):
        frm = tk.Frame(self.settings_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)

        # Serial controls
        serial_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        serial_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(serial_frame, text=f"Port: {SERIAL_PORT}").pack(side="left", padx=5)
        tk.Button(serial_frame, text="Open Serial", command=self.open_serial).pack(side="left", padx=5)
        tk.Button(serial_frame, text="Close Serial", command=self.close_serial).pack(side="left", padx=5)

        # Component tests (kept for functionality, but can be removed if not needed)
        tests_frame = tk.LabelFrame(frm, text="Component Tests")
        tests_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        tk.Button(tests_frame, text="Servo: LEFT", command=lambda: self.send_sort('L')).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: CENTER", command=lambda: self.send_sort('C')).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: RIGHT", command=lambda: self.send_sort('R')).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor ON", command=lambda: self.send_cmd("MOTOR,ON")).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor OFF", command=lambda: self.send_cmd("MOTOR,OFF")).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Request AS7263", command=self.request_as).grid(row=2, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Start IR Monitor", command=self.start_ir_monitor).grid(row=2, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Stop IR Monitor", command=self.stop_ir_monitor).grid(row=2, column=2, padx=2, pady=2)

        # Live status dashboard (replaces previous contents)
        status_frame = tk.LabelFrame(frm, text="Live Component Status")
        status_frame.grid(row=2, column=0, sticky="nsew", padx=4, pady=4)
        self.serial_status_label = tk.Label(status_frame, text="Serial Connected: False", font=("Arial", 12))
        self.serial_status_label.pack(anchor="w", pady=2)
        self.camera_status_label = tk.Label(status_frame, text="Camera Running: False", font=("Arial", 12))
        self.camera_status_label.pack(anchor="w", pady=2)
        self.ir_status_label = tk.Label(status_frame, text="IR Proximity Sensor (distance): False ; N/A cm", font=("Arial", 12))
        self.ir_status_label.pack(anchor="w", pady=2)
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
        distance = f"{self.ir_distance} cm" if self.ir_distance is not None else "N/A"
        self.ir_status_label.config(text=f"IR Proximity Sensor (distance): {self.ir_detected} ; {distance}")
        self.motor_status_label.config(text=f"Motor: {'On' if self.motor_status else 'Off'}")
        servo_map = {'L': 'Left', 'C': 'Center', 'R': 'Right'}
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
                      "Version: 1.0 (GUI Prototype)\n"
                      "Developed for Technopreneurship Project\n"
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
            self._log_message("Failed to open serial: " + str(e))

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self._log_message("Serial closed.")
        except Exception as e:
            self._log_message("Error closing serial: " + str(e))

    def send_cmd(self, text):
        if not self.serial or not self.serial.is_open:
            self._log_message("Serial not open. Cannot send: " + text)
            return
        try:
            with self.serial_lock:
                self.serial.write((text + "\n").encode())
            self._log_message(f"TX -> {text}")
            if text == "MOTOR,ON":
                self.motor_status = True
            elif text == "MOTOR,OFF":
                self.motor_status = False
        except Exception as e:
            self._log_message("Serial write failed: " + str(e))

    def send_sort(self, char):
        if char not in ('L', 'C', 'R'):
            self._log_message(f"Invalid sort direction: {char}")
            return
        self._log_message(f"Sending SORT,{char} for servo test")
        self.send_cmd(f"SORT,{char}")
        self.servo_position = char

    def serial_reader_thread(self):
        self._log_message("Serial reader started.")
        last_ir_report_time = time.time()
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    if self.ir_monitoring and self.ir_detected and (time.time() - last_ir_report_time > 1.0):
                        self.ir_detected = False
                        self.ir_distance = None
                        self._log_message("IR: No object detected (timeout)")
                    continue

                self._log_message(f"RX <- {line}")

                if self.ir_monitoring and line.startswith("IR:"):
                    try:
                        distance_str = line[3:]
                        distance = float(distance_str)
                        last_ir_report_time = time.time()
                        if not self.ir_detected or self.ir_distance != distance:
                            self.ir_detected = True
                            self.ir_distance = distance
                            self._log_message(f"IR: Object detected at {distance}cm")
                    except ValueError:
                        self._log_message("Invalid IR distance format")
                elif line.startswith("AS:"):
                    payload = line[3:]
                    try:
                        self.as_values = [float(v) for v in payload.split(",")]
                        self._log_message(f"AS values: {self.as_values}")
                    except ValueError:
                        self._log_message("Invalid AS values format")
                elif line == "ACK":
                    self._log_message("ACK from Arduino (action completed)")
                else:
                    self._log_message("Serial raw: " + line)
            except Exception as e:
                self._log_message("Serial reader exception: " + str(e))
                time.sleep(0.1)

    # ---------- IR Monitor ----------
    def start_ir_monitor(self):
        if not (self.serial and self.serial.is_open):
            self._log_message("Open serial first.")
            return
        if self.ir_monitoring:
            self._log_message("IR monitor already running.")
            return
        self.ir_monitoring = True
        self._log_message("IR monitor started (listening for 'IR:distance' from Arduino).")

    def stop_ir_monitor(self):
        self.ir_monitoring = False
        self._log_message("IR monitor stopped.")

    # ---------- AS7263 Request ----------
    def request_as(self):
        if not (self.serial and self.serial.is_open):
            self._log_message("Open serial first.")
            return
        try:
            self._log_message("Sending REQ_AS and waiting for response...")
            with self.serial_lock:
                self.serial.reset_input_buffer()
                self.serial.write(b"REQ_AS\n")
                self._log_message("REQ_AS sent.")
                t0 = time.time()
                timeout = 2.0
                got = False
                while time.time() - t0 < timeout:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    self._log_message(f"RX during AS wait <- {line}")
                    if line.startswith("AS:"):
                        got = True
                        break
                if not got:
                    self._log_message("No AS response within timeout.")
                else:
                    self._log_message("AS response received successfully.")
        except Exception as e:
            self._log_message("REQ_AS failed: " + str(e))

    # ---------- Camera Methods ----------
    def start_camera(self):
        if not PICAMERA2_AVAILABLE:
            self._log_message("picamera2 not available.")
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self._log_message("Camera started with XRGB8888 @ 640x480.")
        except Exception as e:
            self._log_message("Camera start failed: " + str(e))

    def stop_camera(self):
        try:
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self._log_message("Camera stopped.")
        except Exception as e:
            self._log_message("Camera stop error: " + str(e))

    def camera_loop(self):
        frame_counter = 0
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                # YOLO inference (throttled)
                if self.yolo and frame_counter % 5 == 0:
                    results = self.yolo(frame, verbose=False, imgsz=640, conf=0.25, max_det=5)
                    detections = results[0].boxes
                    object_count = 0

                    for det in detections:
                        xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
                        xmin, ymin, xmax, ymax = xyxy
                        classidx = int(det.cls.item())
                        classname = self.yolo.names[classidx]
                        conf = det.conf.item()

                        if conf > 0.5:
                            color = (0, 255, 0)
                            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                            label = f"{classname}: {int(conf*100)}%"
                            cv2.putText(frame, label, (xmin, max(ymin - 10, 10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                            object_count += 1

                    cv2.putText(frame, f"Objects: {object_count}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

                frame_counter += 1

                # Update canvas
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                display_frame = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display_frame)

                time.sleep(1/30)
            except Exception as e:
                self._log_message("Camera loop error: " + str(e))
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            if PIL_AVAILABLE:
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                self.cam_canvas.imgtk = imgtk  # Keep reference
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
            else:
                # Fallback to OpenCV window
                cv2.imshow("Camera Preview", bgr_frame)
                cv2.waitKey(1)
        except Exception as e:
            self._log_message("Display frame error: " + str(e))

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
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