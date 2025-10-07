#!/usr/bin/env python3
"""
ACSS component test GUI for Raspberry Pi 5.
- Uses Arduino as IO: expect Arduino serial protocol:
- Arduino -> Pi: "IR\n" when IR triggered, "AS:val1,val2,...\n" when responding to REQ_AS, "ACK\n" for actions ack
- Pi -> Arduino: "SORT,L\n" or "SORT,C\n" or "SORT,R\n" ; "REQ_AS\n" ; "MOTOR,ON\n" / "MOTOR,OFF\n"
Notes:
- Automatically opens serial port and starts camera if available.
- All actions wrapped in try/except with debug printouts.
"""

import sys, threading, time
import numpy as np  # Ensure numpy is imported for tracking logic
from resources.reze_console import RezeConsole
console = RezeConsole(use_llm=True, model="gemma2:2b")
# ---------- Optional imports ----------
try:
    import tkinter as tk
    from tkinter import scrolledtext, messagebox
except Exception as e:
    console.comment(f"tkinter not found - GUI will not run. {e}", rephrase=True)
    sys.exit(1)

try:
    import serial
    console.comment("serial module available", rephrase=False)
except Exception:
    serial = None
    console.comment("serial module not available", rephrase=True)

try:
    import cv2
    import numpy as np
    console.comment("OpenCV and NumPy available", rephrase=False)
except Exception:
    cv2 = None
    np = None
    console.comment("OpenCV and NumPy not available", rephrase=True)

try:
    from picamera2 import Picamera2, Preview
    PICAMERA2_AVAILABLE = True
    console.comment("Picamera2 available", rephrase=False)
except Exception:
    PICAMERA2_AVAILABLE = False
    console.comment("Picamera2 not available", rephrase=True)

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
    console.comment("Ultralytics available", rephrase=False)
except Exception:
    ULTRALYTICS_AVAILABLE = False
    console.comment("Ultralytics not available", rephrase=True)

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
    console.comment("Pillow (PIL) available", rephrase=False)
except Exception:
    PIL_AVAILABLE = False
    console.comment("Pillow (PIL) not available", rephrase=True)

# ---------- USER SETTINGS ----------
SERIAL_PORT = "/dev/ttyUSB0"       # /dev/ttyUSB0
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"  # Fixed typo (.py → .pt); replace with your custom path if needed
CAM_PREVIEW_SIZE = (480, 360)
TRACKER_PATH = "bytetrack.yaml"  # Optional: Path to tracker config (download from Ultralytics GitHub if needed)
# ----------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Component Tester")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.gui_ready = False  # Flag to ensure GUI is ready before threading logs

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
        self.sorted_tracks = set()  # Track sorted IDs to avoid re-sorts
        self.last_cleanup = time.time()  # For periodic cleanup

        # ---------- YOLO Model Load ----------
        if ULTRALYTICS_AVAILABLE:
            try:
                self.logmsg("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)  # Updated to support YOLOv11
                self.logmsg(f"YOLO loaded: {YOLO_MODEL_PATH}")
                console.comment(f"YOLO model loaded: {YOLO_MODEL_PATH}", rephrase=False)
            except Exception as ex:
                self.logmsg("YOLO load failed: " + str(ex))
                console.comment("YOLO model load failed", rephrase=True)

        # ---------- Auto-start serial & camera ----------
        try:
            self.logmsg("Attempting to auto-open serial port...")
            self.open_serial()
        except Exception as e:
            self.logmsg(f"Auto serial open failed: {e}")
            console.comment(f"Auto serial open failed: {e}", rephrase=True)

        if PICAMERA2_AVAILABLE:
            try:
                self.logmsg("Attempting to auto-start camera...")
                self.start_camera()
            except Exception as e:
                self.logmsg(f"Auto camera start failed: {e}")
                console.comment(f"Auto camera start failed: {e}", rephrase=True)

        if not PICAMERA2_AVAILABLE:
            for widget in cam_frame.winfo_children():
                if isinstance(widget, tk.Button) and widget['text'] in ("Start Camera Preview", "Stop Camera"):
                    widget.config(state='disabled')

        # Set GUI ready after layout
        self.gui_ready = True

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Logging ----------
    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"

        def _do_log():
            if not hasattr(self, 'log') or not self.log.winfo_exists() or not self.gui_ready:
                return
            try:
                self.log.config(state='normal')
                self.log.insert('end', full_msg + "\n")
                self.log.see('end')
                self.log.config(state='disabled')
            except tk.TclError:
                pass  # Widget destroyed

        if self.gui_ready:
            self.root.after(0, _do_log)
        else:
            print(full_msg)  # Fallback for early logs

    # ---------- Serial ----------
    def open_serial(self):
        try:
            if serial is None:
                self.logmsg("pyserial not available.")
                console.comment("pyserial not available", rephrase=True)
                return
            if self.serial and self.serial.is_open:
                self.logmsg("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            console.comment(f"Serial connected on {SERIAL_PORT}", rephrase=False)
            # Delay thread start until GUI is ready
            self.root.after(100, lambda: threading.Thread(target=self.serial_reader_thread, daemon=True).start())
        except Exception as e:
            self.logmsg("Failed to open serial: " + str(e))
            console.comment("Failed to open serial port", rephrase=True)

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("Serial closed.")
                console.comment("Serial port closed", rephrase=False)
        except Exception as e:
            self.logmsg("Error closing serial: " + str(e))
            console.comment("Error closing serial port", rephrase=True)

    def send_cmd(self, text):
        """Send any general command to Arduino with newline + flush"""
        try:
            if not self.serial or not self.serial.is_open:
                self.logmsg(f"[WARN] Serial not open — can't send: {text}")
                console.comment(f"Cannot send command {text} because serial not open", rephrase=True)
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            self.logmsg(f"[TX] {text.strip().upper()}")
            console.comment(f"Sent command {text} to Arduino", rephrase=False)
        except Exception as e:
            self.logmsg(f"[ERR] Failed to send command '{text}': {e}")
            console.comment(f"Failed to send command {text}", rephrase=True)

    def send_sort(self, char):
        """Send servo sort command (L, C, R)."""
        try:
            char = char.strip().upper()
            if char not in ("L", "C", "R"):
                self.logmsg(f"[WARN] Invalid servo command: {char}")
                console.comment(f"Invalid servo command: {char}", rephrase=True)
                return
            cmd = f"SORT,{char}"
            self.logmsg(f"[SERVO] Sending: {cmd}")
            console.comment(f"Sending servo command {char}", rephrase=False)
            self.send_cmd(cmd)
        except Exception as e:
            self.logmsg(f"[ERR] Servo send_sort failed: {e}")
            console.comment("Failed to send servo command", rephrase=True)

    # ---------- Process Control ----------
    def start_process(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                console.comment("Cannot start process because serial not open", rephrase=True)
                return
            if not self.yolo:
                self.logmsg("YOLO model not loaded.")
                console.comment("Cannot start process because YOLO not loaded", rephrase=True)
                return
            if self.process_running:
                self.logmsg("Process already running.")
                console.comment("Process already running", rephrase=False)
                return
            self.send_cmd("MOTOR,ON")
            self.process_running = True
            self.sort_cooldown = time.time()
            self.sorted_tracks.clear()  # Reset sorted tracks on process start
            self.logmsg("Process started: Motor ON, detection active.")
            console.comment("Process started: Motor ON, detection active", rephrase=False)
        except Exception as e:
            self.logmsg(f"Start process error: {e}")
            console.comment("Error starting process", rephrase=True)

    def stop_process(self):
        try:
            if not self.process_running:
                self.logmsg("Process not running.")
                console.comment("Process not running", rephrase=False)
                return
            self.send_cmd("MOTOR,OFF")
            self.process_running = False
            self.logmsg("Process stopped: Motor OFF, detection inactive.")
            console.comment("Process stopped: Motor OFF, detection inactive", rephrase=False)
        except Exception as e:
            self.logmsg(f"Stop process error: {e}")
            console.comment("Error stopping process", rephrase=True)

    # ---------- Serial Reader ----------
    def serial_reader_thread(self):
        if not self.gui_ready:
            time.sleep(0.5)  # Extra safety delay
        self.logmsg("Serial reader started.")
        console.comment("Serial reader started", rephrase=False)
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
                            console.comment("IR no object detected", rephrase=False)
                        continue

                    # Handle IR silently unless monitoring and state change
                    if line == "IR":
                        if self.ir_monitoring:
                            last_ir_report_time = time.time()
                            if not ir_state:
                                ir_state = True
                                self.logmsg("IR: Object detected")
                                console.comment("IR object detected", rephrase=False)
                        # else: silent, no noise
                    elif line.startswith("AS:"):
                        vals = line[3:].split(",")
                        self.logmsg(f"AS values: {vals}")
                        console.comment(f"Received AS values: {vals}", rephrase=False)
                    elif line == "ACK":
                        self.logmsg("ACK from Arduino")
                        console.comment("Received ACK from Arduino", rephrase=False)
                    # No else: silent for unknowns, keeps it clean
                except Exception as e:
                    self.logmsg(f"Serial reader exception: {e}")
                    console.comment("Serial reader exception", rephrase=True)
                    time.sleep(0.2)
        except Exception as outer:
            self.logmsg(f"Serial reader crashed: {outer}")
            console.comment("Serial reader crashed", rephrase=True)

    # ---------- AS Request ----------
    def request_as(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                console.comment("Cannot request AS because serial not open", rephrase=True)
                return
            self.logmsg("Sending REQ_AS and waiting for response...")
            console.comment("Requesting AS7263 data", rephrase=False)
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
                    console.comment(f"AS response received: {line[3:]}", rephrase=False)
                    got = True
                    break
            if not got:
                self.logmsg("No AS response within timeout.")
                console.comment("No AS response within timeout", rephrase=True)
        except Exception as e:
            self.logmsg(f"REQ_AS failed: {e}")
            console.comment("Failed to request AS", rephrase=True)

    # ---------- IR Monitor ----------
    def start_ir_monitor(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                console.comment("Cannot start IR monitor because serial not open", rephrase=True)
                return
            if self.ir_monitoring:
                self.logmsg("IR monitor already running.")
                console.comment("IR monitor already running", rephrase=False)
                return
            self.ir_monitoring = True
            self.logmsg("IR monitor started.")
            console.comment("IR monitor started", rephrase=False)
        except Exception as e:
            self.logmsg(f"IR monitor start error: {e}")
            console.comment("Error starting IR monitor", rephrase=True)

    def stop_ir_monitor(self):
        try:
            self.ir_monitoring = False
            self.logmsg("IR monitor stopped.")
            console.comment("IR monitor stopped", rephrase=False)
        except Exception as e:
            self.logmsg(f"IR monitor stop error: {e}")
            console.comment("Error stopping IR monitor", rephrase=True)

    # ---------- Camera ----------
    def start_camera(self):
        if self.camera_running:
            self.logmsg("Camera already running.")
            console.comment("Camera already running", rephrase=False)
            return
        if not PICAMERA2_AVAILABLE:
            self.logmsg("picamera2 not available.")
            console.comment("picamera2 not available", rephrase=True)
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
            console.comment("Camera started", rephrase=False)
        except Exception as e:
            self.logmsg(f"Camera start failed: {e}")
            console.comment("Failed to start camera", rephrase=True)

    def stop_camera(self):
        try:
            if not self.camera_running:
                self.logmsg("Camera not running.")
                console.comment("Camera not running", rephrase=False)
                return
            self.camera_running = False
            time.sleep(0.2)
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self.logmsg("Camera stopped.")
            console.comment("Camera stopped", rephrase=False)
        except Exception as e:
            self.logmsg(f"Camera stop error: {e}")
            console.comment("Error stopping camera", rephrase=True)

    def camera_loop(self):
        frame_counter = 0
        fps_time = time.time()
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()  # capture raw frame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                # --- YOLO live tracking (updated for object tracking) ---
                results = None
                if self.yolo:
                    results = self.yolo.track(  # Changed from predict to track
                        source=frame,
                        persist=True,  # Maintain track IDs across frames
                        tracker=TRACKER_PATH,  # Optional: ByteTrack for speed
                        conf=0.25,
                        max_det=5,
                        verbose=False
                    )
                    frame = results[0].plot()  # Plots boxes + track IDs!

                    # --- Integrated Process Detection with Tracking ---
                    current_time = time.time()
                    if self.process_running and results[0].boxes is not None and len(results[0].boxes) > 0 and current_time > self.sort_cooldown:
                        # Get track IDs
                        track_ids = results[0].boxes.id.cpu().numpy() if results[0].boxes.id is not None else np.array([])
                        
                        # For each detection, check if it's a new track (sort only once per ID)
                        boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding boxes
                        cls_tensor = results[0].boxes.cls.cpu().numpy()
                        conf_tensor = results[0].boxes.conf.cpu().numpy()
                        
                        for i in range(len(cls_tensor)):  # Loop over detections
                            track_id = int(track_ids[i]) if len(track_ids) > i else -1
                            if track_id in self.sorted_tracks:  # Skip if already sorted
                                continue
                            
                            cls = int(cls_tensor[i])
                            conf = conf_tensor[i]
                            if conf > 0.25:  # Extra conf check per track
                                sort_char = self.class_to_sort.get(cls, 'C')
                                self.send_sort(sort_char)
                                self.logmsg(f"Tracked ID {track_id}: class {cls} conf {conf:.2f}, sorting {sort_char}")
                                console.comment(f"Detected track {track_id} class {cls} conf {conf:.2f} sorting {sort_char}", rephrase=False)
                                self.sorted_tracks.add(track_id)  # Mark as sorted
                                self.sort_cooldown = current_time + 2.0  # Global debounce
                                break  # Sort only the first valid track per frame (leading copra)

                        # Periodic cleanup of old tracks
                        if current_time - self.last_cleanup > 10.0:  # Every 10s
                            if len(results[0].boxes) == 0:
                                self.sorted_tracks.clear()  # Reset if empty
                            self.last_cleanup = current_time

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
                console.comment(f"Camera loop error: {e}", rephrase=True)

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
            console.comment(f"Display frame error: {e}", rephrase=True)

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
            self.logmsg("Application closed safely.")
            console.comment("Application closed", rephrase=False)
            self.root.destroy()
        except Exception as e:
            console.comment(f"Shutdown error: {e}", rephrase=True)


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = ACSSGui(root)
        root.mainloop()
    except Exception as e:
        console.comment(f"Fatal error: {e}", rephrase=True)