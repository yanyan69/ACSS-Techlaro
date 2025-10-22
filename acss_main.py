#!/usr/bin/env python3
"""
ACSS GUI for Raspberry Pi 5 with integrated hardware and tabbed interface.
- Uses Arduino as IO: expect Arduino serial protocol.
- Arduino -> Pi: Various ACK, TRIG, AS: messages.
- Pi -> Arduino: Classification strings (OVERCOOKED, RAW, STANDARD), other commands.
- Integrated with tabbed GUI (Home, Statistics, About, Exit).
- Auto-starts serial and camera, polls YOLO on AT_CAM until timeout.
- Failsafe (OVERCOOKED) only on flap detection without prior RPi input.
- Maintains live camera preview with bounding boxes.
"""

import sys, threading, time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
try:
    import serial
except Exception:
    serial = None
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False

# ---------- USER SETTINGS ----------
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
YOLO_MODEL_PATH = "my_model/my_model.pt"
CAM_PREVIEW_SIZE = (640, 480)
TRACKER_PATH = "bytetrack.yaml"
CLASSIFICATION_TIMEOUT_S = 2.0
USERNAME = "Copra Buyer 01"

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System (AUTO)")
        self.root.attributes('-fullscreen', True)
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.gui_ready = False
        self.camera_running = False
        self.stats = {'Raw': 0, 'Standard': 0, 'Overcooked': 0, 'total': 0, 'start_time': None, 'end_time': None}
        self.moisture_sums = {'Raw': 0.0, 'Standard': 0.0, 'Overcooked': 0.0}

        # ---------- GUI Layout ----------
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)
        self.home_tab = ttk.Frame(notebook)
        self.statistics_tab = ttk.Frame(notebook)
        self.about_tab = ttk.Frame(notebook)
        self.exit_tab = ttk.Frame(notebook)
        notebook.add(self.home_tab, text="Home")
        notebook.add(self.statistics_tab, text="Statistics")
        notebook.add(self.about_tab, text="About")
        notebook.add(self.exit_tab, text="Exit")

        self._build_home_tab()
        self._build_statistics_tab()
        self._build_about_tab()
        self._build_exit_tab()

        # ---------- Hardware Setup ----------
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.results_lock = threading.Lock()
        self.latest_results = None

        if ULTRALYTICS_AVAILABLE:
            try:
                self._log_message("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self._log_message(f"YOLO loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message(f"YOLO load failed: {ex}")

        try:
            self._log_message("Attempting to auto-open serial port...")
            self.open_serial()
        except Exception as e:
            self._log_message(f"Auto serial open failed: {e}")

        if PICAMERA2_AVAILABLE:
            try:
                self._log_message("Attempting to auto-start camera...")
                self.start_camera()
            except Exception as e:
                self._log_message(f"Auto camera start failed: {e}")

        if not PICAMERA2_AVAILABLE:
            for widget in self.home_tab.winfo_children()[0].winfo_children()[0].winfo_children():
                if isinstance(widget, tk.Button) and widget['text'] in ("Start Camera Preview", "Stop Camera"):
                    widget.config(state='disabled')

        self.stats['start_time'] = time.time()
        self.update_stats()
        self.gui_ready = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI Build ----------
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
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(padx=4, pady=4)
        tk.Button(cam_frame, text="Start Camera Preview", command=self.start_camera).pack(side='left', padx=4, pady=4)
        tk.Button(cam_frame, text="Stop Camera", command=self.stop_camera).pack(side='left', padx=4, pady=4)

        controls_frame = tk.LabelFrame(left_frame, text="Controls")
        controls_frame.grid(row=1, column=0, sticky="ew", padx=4, pady=8)
        tk.Button(controls_frame, text="Servo: LEFT", command=lambda: self.send_cmd("SORT,L")).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Servo: CENTER", command=lambda: self.send_cmd("SORT,C")).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Servo: RIGHT", command=lambda: self.send_cmd("SORT,R")).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Motor ON", command=lambda: self.send_cmd("MOTOR,ON")).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Motor OFF", command=lambda: self.send_cmd("MOTOR,OFF")).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Request AS7263", command=self.request_as).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Auto Mode", command=lambda: self.send_cmd("MODE,AUTO")).pack(side='left', padx=2, pady=2)
        tk.Button(controls_frame, text="Manual Mode", command=lambda: self.send_cmd("MODE,MANUAL")).pack(side='left', padx=2, pady=2)

        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, state='disabled', wrap='word', height=20, width=40)
        self.log.pack(fill='both', expand=True)

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

    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        about_frame = tk.LabelFrame(frm, text="About ACSS")
        about_frame.pack(fill="both", expand=True, padx=4, pady=4)
        tk.Label(about_frame, text="Automated Copra Segregation System\n\nVersion: 1.0\nDeveloped for Practice and Design 2\nMarinduque State University, 2025", font=("Arial", 12), justify="center").pack(pady=40)

    def _build_exit_tab(self):
        frm = tk.Frame(self.exit_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        exit_frame = tk.LabelFrame(frm, text="Exit Application")
        exit_frame.pack(fill="both", expand=True, padx=4, pady=4)
        tk.Label(exit_frame, text="Click the button below to close the program.", font=("Arial", 12)).pack(pady=20)
        tk.Button(exit_frame, text="Exit Program", fg="white", bg="red", font=("Arial", 14, "bold"), width=20, command=self._exit_program).pack(pady=30)

    def _exit_program(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit?"):
            self.running = False
            self.on_close()

    # ---------- Logging ----------
    def _log_message(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        def _do_log():
            if not hasattr(self, 'log') or not self.log.winfo_exists() or not self.gui_ready:
                return
            try:
                self.log.config(state='normal')
                self.log.insert("end", full_msg + "\n")
                self.log.see("end")
                self.log.config(state='disabled')
            except tk.TclError:
                pass
        if self.gui_ready:
            self.root.after(0, _do_log)
        else:
            print(full_msg)

    # ---------- Serial ----------
    def open_serial(self):
        try:
            if serial is None:
                self._log_message("pyserial not available.")
                return
            if self.serial and self.serial.is_open:
                self._log_message("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self._log_message(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            self.root.after(100, lambda: threading.Thread(target=self.serial_reader_thread, daemon=True).start())
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
        try:
            if not self.serial or not self.serial.is_open:
                self._log_message(f"[WARN] Serial not open — can't send: {text}")
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            self._log_message(f"[TX] {text.strip().upper()}")
        except Exception as e:
            self._log_message(f"[ERR] Failed to send command '{text}': {e}")

    # ---------- Serial Reader ----------
    def serial_reader_thread(self):
        if not self.gui_ready:
            time.sleep(0.5)
        self._log_message("Serial reader started.")
        try:
            while self.serial and self.serial.is_open and self.running:
                try:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    self._log_message("[RX] " + line)
                    if line.startswith("ACK,AT_CAM"):
                        self._log_message("Received AT_CAM, classifying...")
                        self.classify_and_send()
                    elif line.startswith("ACK,CLASS,"):
                        cls = line.split(",")[-1]
                        if cls in ["OVERCOOKED", "RAW", "STANDARD"]:
                            self.stats[cls] += 1
                            self.stats['total'] += 1
                            self.update_stats()
                except Exception as e:
                    self._log_message(f"Serial reader exception: {e}")
                    time.sleep(0.2)
        except Exception as outer:
            self._log_message(f"Serial reader crashed: {outer}")

    # ---------- Classification ----------
    def classify_and_send(self):
        try:
            start_time = time.time()
            while time.time() - start_time < CLASSIFICATION_TIMEOUT_S:
                with self.results_lock:
                    if self.latest_results and self.latest_results.boxes:
                        boxes = self.latest_results.boxes
                        confs = boxes.conf.cpu().numpy()
                        if len(confs) > 0:
                            max_idx = np.argmax(confs)
                            cls = int(boxes.cls[max_idx])
                            conf = confs[max_idx]
                            if conf > 0.5:
                                class_str = {0: "OVERCOOKED", 1: "RAW", 2: "STANDARD"}.get(cls, "OVERCOOKED")
                                self.send_cmd(class_str)
                                self._log_message(f"Classified and sent: {class_str} (conf {conf:.2f})")
                                return
                time.sleep(0.1)
            self._log_message("No reliable detection within timeout, skipping send (Arduino failsafe to OVERCOOKED)")
        except Exception as e:
            self._log_message(f"Classification error: {e}")

    # ---------- AS Request ----------
    def request_as(self):
        try:
            if not (self.serial and self.serial.is_open):
                self._log_message("Open serial first.")
                return
            self._log_message("Sending REQ_AS and waiting for response...")
            with self.serial_lock:
                self.serial.reset_input_buffer()
                self.serial.write(b"REQ_AS\n")
                self.serial.flush()
            t0 = time.time()
            got = False
            while time.time() - t0 < 2:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                if line.startswith("AS:"):
                    self._log_message("AS response received: " + line)
                    got = True
                    # Parse AS7263 readings (example: simulate moisture)
                    values = [float(x) for x in line.split(":")[1].split(",")]
                    avg_moisture = sum(values) / len(values)  # Simplified
                    self.moisture_sums["Standard" if self.stats["total"] == 0 else max(self.stats, key=self.stats.get)] += avg_moisture
                    self.update_stats()
                    break
            if not got:
                self._log_message("No AS response within timeout.")
        except Exception as e:
            self._log_message(f"REQ_AS failed: {e}")

    # ---------- Camera ----------
    def start_camera(self):
        if self.camera_running:
            self._log_message("Camera already running.")
            return
        if not PICAMERA2_AVAILABLE:
            self._log_message("picamera2 not available.")
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)
            self.camera_running = True
            threading.Thread(target=self.camera_loop, daemon=True).start()
            self._log_message("Camera started with optimized preview config (640x480).")
        except Exception as e:
            self._log_message(f"Camera start failed: {e}")

    def stop_camera(self):
        try:
            if not self.camera_running:
                self._log_message("Camera not running.")
                return
            self.camera_running = False
            time.sleep(0.2)
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
                    results = self.yolo.track(source=frame, persist=True, tracker=TRACKER_PATH, conf=0.25, max_det=5, verbose=False)
                    frame = results[0].plot()
                    with self.results_lock:
                        self.latest_results = results[0]
                frame_counter += 1
                if frame_counter >= 10:
                    fps = 10 / (time.time() - fps_time)
                    fps_time = time.time()
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    frame_counter = 0
                display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display)
                time.sleep(0.001)
            except Exception as e:
                self._log_message(f"Camera loop error: {e}")

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

    # ---------- Stats ----------
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

    def _reset_stats(self):
        for cat in ["Raw", "Standard", "Overcooked"]:
            self.stats[cat] = 0
            self.moisture_sums[cat] = 0.0
        self.stats['total'] = 0
        self.stats['start_time'] = time.time()
        self.stats['end_time'] = None
        self.update_stats()

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
        self.camera_running = False
        try:
            if self.picam2:
                self.picam2.stop()
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.stats['end_time'] = time.time()
            self.update_stats()
            self._log_message("Application closed safely.")
            self.root.destroy()
        except Exception as e:
            self._log_message(f"Shutdown error: {e}")

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = ACSSGui(root)
        root.mainloop()
    except Exception as e:
        print(f"Fatal error: {e}")