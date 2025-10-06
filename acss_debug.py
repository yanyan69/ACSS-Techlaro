#!/usr/bin/env python3
"""
ACSS component test GUI for Raspberry Pi 5.
- Uses Arduino as IO: expect Arduino serial protocol:
  Arduino -> Pi: "IR\n" when IR triggered, "AS:val1,val2,...\n" when responding to REQ_AS, "ACK\n" for actions ack
  Pi -> Arduino: "SORT,L\n" or "SORT,C\n" or "SORT,R\n" ; "REQ_AS\n" ; "MOTOR,ON\n" / "MOTOR,OFF\n"
Notes:
- Automatically opens serial port and starts camera if available.
- All actions wrapped in try/except with debug printouts.
"""

import sys, threading, time

# ---------- Optional imports ----------
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
    from picamera2 import Picamera2, Preview
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

# ---------- USER SETTINGS ----------
SERIAL_PORT = "/dev/ttyUSB0"       # or  for Raspberry Pi
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"
CAM_PREVIEW_SIZE = (480, 360)
# ----------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Component Tester")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True

        # ---------- GUI Layout ----------
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        conn_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        conn_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).grid(row=0, column=1)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).grid(row=0, column=2)

        tests_frame = tk.LabelFrame(frm, text="Tests")
        tests_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        tk.Button(tests_frame, text="Servo: LEFT", command=lambda: self.send_sort('L')).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: CENTER", command=lambda: self.send_sort('C')).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: RIGHT", command=lambda: self.send_sort('R')).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor ON", command=lambda: self.send_cmd("MOTOR,ON")).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor OFF", command=lambda: self.send_cmd("MOTOR,OFF")).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Request AS7263", command=self.request_as).grid(row=2, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Start IR Monitor", command=self.start_ir_monitor).grid(row=2, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Stop IR Monitor", command=self.stop_ir_monitor).grid(row=2, column=2, padx=2, pady=2)
        tk.Button(tests_frame, text="Start Process", command=self.start_process).grid(row=3, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Stop Process", command=self.stop_process).grid(row=3, column=1, padx=2, pady=2)

        cam_frame = tk.LabelFrame(frm, text="Camera / YOLO")
        cam_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack()
        self.camera_running = False
        tk.Button(cam_frame, text="Start Camera Preview", command=self.start_camera).pack(side='left', padx=4, pady=4)
        tk.Button(cam_frame, text="Stop Camera", command=self.stop_camera).pack(side='left', padx=4, pady=4)

        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, height=6, width=80, state='disabled', wrap='none')
        self.log.pack(fill='both', expand=True)

        self.ir_monitoring = False
        self.ir_thread = None
        self.camera_thread = None
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.process_running = False
        self.sort_cooldown = 0
        self.class_to_sort = {0: 'L', 1: 'C', 2: 'R'}

        # ---------- YOLO Model Load ----------
        if ULTRALYTICS_AVAILABLE:
            try:
                self.logmsg("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg(f"YOLO loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self.logmsg("YOLO load failed: " + str(ex))

        # ---------- Auto-start serial & camera ----------
        try:
            self.logmsg("Attempting to auto-open serial port...")
            self.open_serial()
        except Exception as e:
            self.logmsg(f"Auto serial open failed: {e}")

        if PICAMERA2_AVAILABLE:
            try:
                self.logmsg("Attempting to auto-start camera...")
                self.start_camera()
            except Exception as e:
                self.logmsg(f"Auto camera start failed: {e}")

        if not PICAMERA2_AVAILABLE:
            for widget in cam_frame.winfo_children():
                if isinstance(widget, tk.Button) and widget['text'] in ("Start Camera Preview", "Stop Camera"):
                    widget.config(state='disabled')

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Logging ----------
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
        try:
            if serial is None:
                self.logmsg("pyserial not available.")
                return
            if self.serial and self.serial.is_open:
                self.logmsg("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            threading.Thread(target=self.serial_reader_thread, daemon=True).start()
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
        """Send any general command to Arduino with newline + flush"""
        try:
            if not self.serial or not self.serial.is_open:
                self.logmsg(f"[WARN] Serial not open — can't send: {text}")
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            self.logmsg(f"[TX] {text.strip().upper()}")
        except Exception as e:
            self.logmsg(f"[ERR] Failed to send command '{text}': {e}")

    def send_sort(self, char):
        """Send servo sort command (L, C, R)."""
        try:
            char = char.strip().upper()
            if char not in ("L", "C", "R"):
                self.logmsg(f"[WARN] Invalid servo command: {char}")
                return
            cmd = f"SORT,{char}"
            self.logmsg(f"[SERVO] Sending: {cmd}")
            self.send_cmd(cmd)
        except Exception as e:
            self.logmsg(f"[ERR] Servo send_sort failed: {e}")

    # ---------- Process Control ----------
    def start_process(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                return
            if not self.yolo:
                self.logmsg("YOLO model not loaded.")
                return
            if self.process_running:
                self.logmsg("Process already running.")
                return
            self.send_cmd("MOTOR,ON")
            self.process_running = True
            self.sort_cooldown = time.time()
            self.logmsg("Process started: Motor ON, detection active.")
        except Exception as e:
            self.logmsg(f"Start process error: {e}")

    def stop_process(self):
        try:
            if not self.process_running:
                self.logmsg("Process not running.")
                return
            self.send_cmd("MOTOR,OFF")
            self.process_running = False
            self.logmsg("Process stopped: Motor OFF, detection inactive.")
        except Exception as e:
            self.logmsg(f"Stop process error: {e}")

    # ---------- Serial Reader ----------
    def serial_reader_thread(self):
        self.logmsg("Serial reader started.")
        try:
            ir_state = False
            last_ir_report_time = time.time()
            while self.serial and self.serial.is_open and self.running:
                try:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line:
                        if self.ir_monitoring and ir_state and (time.time() - last_ir_report_time > 1.0):
                            ir_state = False
                            self.logmsg("IR: No object detected (timeout)")
                        continue

                    self.logmsg(f"RX <- {line}")

                    if self.ir_monitoring and line == "IR":
                        last_ir_report_time = time.time()
                        if not ir_state:
                            ir_state = True
                            self.logmsg("IR: Object detected")
                    elif line.startswith("AS:"):
                        vals = line[3:].split(",")
                        self.logmsg(f"AS values: {vals}")
                    elif line == "ACK":
                        self.logmsg("ACK from Arduino")
                    else:
                        self.logmsg(f"Serial raw: {line}")
                except Exception as e:
                    self.logmsg(f"Serial reader exception: {e}")
                    time.sleep(0.2)
        except Exception as outer:
            self.logmsg(f"Serial reader crashed: {outer}")

    # ---------- AS Request ----------
    def request_as(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                return
            self.logmsg("Sending REQ_AS and waiting for response...")
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
                    self.logmsg("AS response received: " + line)
                    got = True
                    break
            if not got:
                self.logmsg("No AS response within timeout.")
        except Exception as e:
            self.logmsg(f"REQ_AS failed: {e}")

    # ---------- IR Monitor ----------
    def start_ir_monitor(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                return
            if self.ir_monitoring:
                self.logmsg("IR monitor already running.")
                return
            self.ir_monitoring = True
            self.logmsg("IR monitor started.")
        except Exception as e:
            self.logmsg(f"IR monitor start error: {e}")

    def stop_ir_monitor(self):
        try:
            self.ir_monitoring = False
            self.logmsg("IR monitor stopped.")
        except Exception as e:
            self.logmsg(f"IR monitor stop error: {e}")

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
            # === Same settings as your YOLO test ===
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)  # let camera warm up a bit
            self.camera_running = True
            threading.Thread(target=self.camera_loop, daemon=True).start()
            self.logmsg("Camera started with optimized preview config (640x480).")
        except Exception as e:
            self.logmsg(f"Camera start failed: {e}")

    def stop_camera(self):
        try:
            if not self.camera_running:
                self.logmsg("Camera not running.")
                return
            self.camera_running = False
            time.sleep(0.2)
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self.logmsg("Camera stopped.")
        except Exception as e:
            self.logmsg(f"Camera stop error: {e}")

    def camera_loop(self):
        frame_counter = 0
        fps_time = time.time()
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()  # capture raw frame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                # --- YOLO live detection ---
                results = None
                if self.yolo:
                    results = self.yolo.predict(
                        source=frame,
                        imgsz=640,
                        conf=0.25,
                        max_det=5,
                        verbose=False
                    )
                    frame = results[0].plot()

                    # --- Integrated Process Detection ---
                    current_time = time.time()
                    if self.process_running and results[0].boxes is not None and len(results[0].boxes) > 0 and current_time > self.sort_cooldown:
                        cls_tensor = results[0].boxes.cls
                        conf_tensor = results[0].boxes.conf
                        max_idx = conf_tensor.argmax().item()
                        cls = int(cls_tensor[max_idx].item())
                        conf = conf_tensor[max_idx].item()
                        sort_char = self.class_to_sort.get(cls, 'C')
                        self.send_sort(sort_char)
                        self.logmsg(f"Detected class {cls} conf {conf:.2f}, sorting {sort_char}")
                        self.sort_cooldown = current_time + 2.0  # 2-second cooldown to avoid multiple sorts per object

                # --- FPS overlay ---
                frame_counter += 1
                if frame_counter >= 10:
                    fps = 10 / (time.time() - fps_time)
                    fps_time = time.time()
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    frame_counter = 0

                # --- Display on canvas ---
                display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display)
                time.sleep(0.001)  # smooth refresh, minimal delay
            except Exception as e:
                self.logmsg(f"Camera loop error: {e}")
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
            self.logmsg(f"Display frame error: {e}")

    # ---------- Shutdown ----------
    def on_close(self):
        try:
            self.running = False
            self.process_running = False
            self.camera_running = False
            if self.picam2:
                self.picam2.stop()
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.root.destroy()
            self.logmsg("Application closed safely.")
        except Exception as e:
            print("Shutdown error:", e)
            self.root.destroy()


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = ACSSGui(root)
        root.mainloop()
    except Exception as e:
        print("Fatal error:", e)