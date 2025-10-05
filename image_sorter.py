#!/usr/bin/env python3
"""
ACSS YOLO + Servo Test GUI for Raspberry Pi 5.
- Uses Arduino for servo control via serial: sends "SORT,L\n" or "SORT,C\n" or "SORT,R\n" based on YOLO detection.
- Detects 'raw-copra', 'standard-copra', 'overcooked-copra' and maps:
  - raw-copra: SORT,R (right)
  - standard-copra: SORT,C (center)
  - overcooked-copra: SORT,L (left)
- Assumes YOLO classes: e.g., 0='raw-copra', 1='standard-copra', 2='overcooked-copra'
"""

import sys, threading, time

# Optional imports (graceful degradation)
try:
    import tkinter as tk
    from tkinter import scrolledtext, messagebox
except Exception as e:
    print("tkinter missing - GUI won't run.", e)
    sys.exit(1)

try:
    import serial
    print("serial is available")
except Exception:
    serial = None
    print("serial is not available")

try:
    import cv2
    import numpy as np
    print("cv2 and numpy are available")
except Exception:
    cv2 = None
    np = None
    print("cv2 and numpy are not available")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
    print("picamera is available")
except Exception:
    PICAMERA2_AVAILABLE = False
    print("picamera is not available")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
    print("ultralytics is available")
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("ultralytics is not available")

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
    print("PIL is available")
except Exception:
    PIL_AVAILABLE = False
    print("PIL is not available")

# ------------------ USER SETTINGS ------------------
SERIAL_PORT = "/dev/ttyUSB0"   # change if needed
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"
CAM_PREVIEW_SIZE = (480, 360)
# ----------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS YOLO + Servo Tester")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.detection_running = False

        # UI setup
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        # Serial / Arduino controls
        conn_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        conn_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).grid(row=0, column=1)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).grid(row=0, column=2)

        # Servo test buttons
        tests_frame = tk.LabelFrame(frm, text="Tests")
        tests_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        tk.Button(tests_frame, text="Servo: LEFT", command=lambda: self.send_sort('L')).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: CENTER", command=lambda: self.send_sort('C')).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: RIGHT", command=lambda: self.send_sort('R')).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(tests_frame, text="Start YOLO Detection", command=self.start_detection).grid(row=1, column=0, columnspan=2, padx=2, pady=2, sticky="ew")
        tk.Button(tests_frame, text="Stop YOLO Detection", command=self.stop_detection).grid(row=1, column=2, padx=2, pady=2)

        # Camera controls
        cam_frame = tk.LabelFrame(frm, text="Camera / YOLO")
        cam_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack()
        self.camera_running = False
        tk.Button(cam_frame, text="Start Camera Preview", command=self.start_camera).pack(side='left', padx=4, pady=4)
        tk.Button(cam_frame, text="Stop Camera", command=self.stop_camera).pack(side='left', padx=4, pady=4)

        # Log area
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, height=6, width=80, state='disabled', wrap='none')
        self.log.pack(fill='both', expand=True)

        # Internal vars
        self.camera_thread = None
        self.detection_thread = None
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None

        # Load YOLO model
        if ULTRALYTICS_AVAILABLE:
            try:
                self.logmsg("Loading YOLO model (may take a while)...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self.logmsg("YOLO load failed: " + str(ex))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        if not PICAMERA2_AVAILABLE:
            for widget in cam_frame.winfo_children():
                if isinstance(widget, tk.Button) and widget['text'] in ("Start Camera Preview", "Stop Camera"):
                    widget.config(state='disabled')

    # ---------- Utility ----------
    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert('end', full_msg + "\n")
        self.log.see('end')
        self.log.config(state='disabled')
        print(full_msg)

    # ---------- Serial ----------
    def open_serial(self):
        if serial is None:
            self.logmsg("pyserial not available.")
            return
        if self.serial and self.serial.is_open:
            self.logmsg("Serial already open.")
            return
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            t = threading.Thread(target=self.serial_reader_thread, daemon=True)
            t.start()
        except Exception as e:
            self.logmsg("Failed to open serial: " + str(e))

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("Serial closed.")
        except Exception as e:
            self.logmsg("Error closing serial: " + str(e))

    def send_cmd(self, text):
        if not self.serial or not self.serial.is_open:
            self.logmsg("Serial not open. Cannot send: " + text)
            return
        try:
            with self.serial_lock:
                self.serial.write((text + "\n").encode())
            self.logmsg(f"TX -> {text}")
        except Exception as e:
            self.logmsg("Serial write failed: " + str(e))

    def send_sort(self, char):
        if char not in ('L', 'C', 'R'):
            self.logmsg(f"Invalid sort direction: {char}")
            return
        self.logmsg(f"Sending SORT,{char}")
        self.send_cmd(f"SORT,{char}")

    def serial_reader_thread(self):
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if line:
                    self.logmsg(f"RX <- {line}")
            except Exception as e:
                self.logmsg("Serial reader exception: " + str(e))
                time.sleep(0.1)

    # ---------- Camera ----------
    def start_camera(self):
        if self.camera_running:
            self.logmsg("Camera already running.")
            return
        if not PICAMERA2_AVAILABLE:
            self.logmsg("picamera2 not available.")
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
            self.picam2.configure(config)
            self.picam2.start()
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self.logmsg("Camera started.")
        except Exception as e:
            self.logmsg("Camera start failed: " + str(e))

    def stop_camera(self):
        if not self.camera_running:
            self.logmsg("Camera not running.")
            return
        try:
            self.camera_running = False
            time.sleep(0.2)
            if self.picam2:
                self.picam2.stop()
            self.picam2 = None
            self.logmsg("Camera stopped.")
        except Exception as e:
            self.logmsg("Camera stop error: " + str(e))

    def camera_loop(self):
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display)
                time.sleep(1/30)
            except Exception as e:
                self.logmsg("Camera loop error: " + str(e))
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            if PIL_AVAILABLE:
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                self.cam_canvas.imgtk = imgtk
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
        except Exception as e:
            self.logmsg("Display frame error: " + str(e))

    # ---------- YOLO Detection ----------
    def start_detection(self):
        if self.detection_running:
            self.logmsg("Detection already running.")
            return
        if not self.camera_running:
            self.start_camera()
        if self.yolo is None:
            self.logmsg("YOLO model not loaded.")
            return
        if not (self.serial and self.serial.is_open):
            self.logmsg("Open serial first for servo control.")
            return
        self.detection_running = True
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()
        self.logmsg("YOLO detection started.")

    def stop_detection(self):
        self.detection_running = False
        self.logmsg("YOLO detection stopped.")

    def detection_loop(self):
        frame_counter = 0
        color_map = {
            'raw-copra': (0, 0, 255),          # Red for raw
            'standard-copra': (0, 255, 0),     # Green for standard
            'overcooked-copra': (0, 255, 255)  # Yellow for overcooked
        }

        while self.detection_running:
            try:
                frame = None
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame = self.latest_frame.copy()
                if frame is None:
                    time.sleep(0.1)
                    continue

                if frame_counter % 5 == 0:
                    results = self.yolo(frame, verbose=False, imgsz=640, conf=0.25, max_det=1)
                    detections = results[0].boxes

                    if detections:
                        det = detections[0]
                        classidx = int(det.cls.item())
                        classname = self.yolo.names.get(classidx, 'unknown')
                        conf = det.conf.item()

                        if conf > 0.5:
                            if classname == 'raw-copra':
                                sort_cmd = 'R'
                            elif classname == 'standard-copra':
                                sort_cmd = 'C'
                            elif classname == 'overcooked-copra':
                                sort_cmd = 'L'
                            else:
                                sort_cmd = 'C'

                            self.send_sort(sort_cmd)
                            self.logmsg(f"Detected {classname} ({conf:.2f}) -> SORT,{sort_cmd}")

                            xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
                            color = color_map.get(classname, (255, 255, 255))
                            cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                            cv2.putText(frame, f"{classname}: {int(conf*100)}%", 
                                        (xyxy[0], xyxy[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                            display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                            self.update_canvas_with_frame(display)

                frame_counter += 1
                time.sleep(1/30)

            except Exception as e:
                self.logmsg("Detection loop error: " + str(e))
                time.sleep(0.2)

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
        self.camera_running = False
        self.detection_running = False
        try:
            if self.picam2:
                self.picam2.stop()
        except Exception:
            pass
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)

    def auto_start():
        try:
            app.start_camera()
            app.logmsg("Camera auto-started on launch.")
            root.after(2000, lambda: app.start_detection())  # wait 2s before YOLO start
        except Exception as e:
            app.logmsg("Auto-start failed: " + str(e))

    # Delay auto-start slightly so UI fully loads
    root.after(1000, auto_start)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    except Exception as e:
        print("Fatal error:", e)
        app.on_close()
