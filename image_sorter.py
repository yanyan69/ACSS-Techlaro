#!/usr/bin/env python3
"""
ACSS YOLO + Servo Test GUI for Raspberry Pi 5.
- Uses Arduino for servo control via serial: sends "SORT,L\n" or "SORT,C\n" or "SORT,R\n" based on YOLO detection.
- Detects 'raw-copra', 'standard-copra', 'overcooked-copra' and maps:
  - raw-copra / raw: SORT,R (right)
  - standard-copra / standard: SORT,C (center)
  - overcooked-copra / overcooked / rejected: SORT,L (left)
- Detection runs even if serial is not opened. UI updates are scheduled on the main thread to avoid segfaults.
"""

import sys
import threading
import time

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

# Bounding box colors requested (BGR)
BOX_COLORS = {
    'raw': (255, 0, 123),            # Blue-ish (user-specified)
    'raw-copra': (255, 0, 123),
    'standard': (40, 167, 69),       # Green
    'standard-copra': (40, 167, 69),
    'rejected': (220, 53, 69),       # Red
    'overcooked': (220, 53, 69),
    'overcooked-copra': (220, 53, 69),
}

# A short cooldown (seconds) to reduce repeated logs/commands for same detection
DETECTION_COOLDOWN = 0.6

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
        self._tk_image_ref = None  # keep reference to PhotoImage
        self._last_detection_time = 0.0

        # Load YOLO model (non-blocking try — if fails, detection won't start)
        if ULTRALYTICS_AVAILABLE:
            try:
                self.logmsg("Loading YOLO model (may take a while)...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self.logmsg("YOLO load failed: " + str(ex))
                self.yolo = None

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
        if self.serial and getattr(self.serial, "is_open", False):
            self.logmsg("Serial already open.")
            return
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            t = threading.Thread(target=self.serial_reader_thread, daemon=True)
            t.start()
        except Exception as e:
            self.logmsg("Failed to open serial: " + str(e))
            self.serial = None

    def close_serial(self):
        try:
            if self.serial and getattr(self.serial, "is_open", False):
                self.serial.close()
                self.logmsg("Serial closed.")
        except Exception as e:
            self.logmsg("Error closing serial: " + str(e))
        finally:
            self.serial = None

    def send_cmd(self, text):
        if not self.serial or not getattr(self.serial, "is_open", False):
            # don't spam log for every detection when serial missing
            return False
        try:
            with self.serial_lock:
                self.serial.write((text + "\n").encode())
            # short log only
            self.logmsg(f"TX -> {text}")
            return True
        except Exception as e:
            self.logmsg("Serial write failed: " + str(e))
            return False

    def send_sort(self, char):
        if char not in ('L', 'C', 'R'):
            self.logmsg(f"Invalid sort direction: {char}")
            return
        self.logmsg(f"Sending SORT,{char}")
        self.send_cmd(f"SORT,{char}")

    def serial_reader_thread(self):
        # Filters noisy serial messages (e.g., IR or frequent telemetry)
        noisy_prefixes = ('IR', 'AS:', 'STATUS', 'SENSOR', 'DBG')
        while self.serial and getattr(self.serial, "is_open", False) and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                # ignore noisy lines
                if any(line.startswith(p) for p in noisy_prefixes) or line in ("IR", "ACK"):
                    # optionally log ACK minimally
                    if line == "ACK":
                        self.logmsg("ACK from Arduino")
                    # otherwise skip
                    continue
                self.logmsg(f"Serial <- {line}")
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
            self.picam2 = None
            self.camera_running = False

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
                # convert from BGRA -> BGR safely
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                # schedule UI update on main thread
                self.root.after(0, self._display_frame_on_canvas, display)
                time.sleep(1/30)
            except Exception as e:
                self.logmsg("Camera loop error: " + str(e))
                time.sleep(0.2)

    def _display_frame_on_canvas(self, bgr_frame):
        try:
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            if PIL_AVAILABLE:
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                self._tk_image_ref = imgtk
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
            else:
                # fallback: show in OpenCV window (not ideal for Tkinter-only setups)
                cv2.imshow("Camera Preview", bgr_frame)
                cv2.waitKey(1)
        except Exception as e:
            # don't spam log if UI update occasionally fails
            self.logmsg("Display frame error: " + str(e))

    # ---------- YOLO Detection ----------
    def start_detection(self):
        if self.detection_running:
            self.logmsg("Detection already running.")
            return
        if not self.camera_running:
            self.start_camera()
        if self.yolo is None:
            self.logmsg("YOLO model not loaded. Detection won't start.")
            return
        # allow detection even if serial isn't open
        if not (self.serial and getattr(self.serial, "is_open", False)):
            self.logmsg("Serial not open — detection will run without servo control.")
        self.detection_running = True
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()
        self.logmsg("YOLO detection started.")

    def stop_detection(self):
        self.detection_running = False
        self.logmsg("YOLO detection stopped.")

    def _classname_to_sort(self, classname):
        """Normalize classname and map to servo char."""
        name = classname.lower()
        if 'raw' in name:
            return 'R'
        if 'standard' in name:
            return 'C'
        # treat overcooked / rejected as rejected -> left
        if 'overcook' in name or 'reject' in name or 'rejected' in name:
            return 'L'
        return 'C'

    def detection_loop(self):
        frame_counter = 0
        while self.detection_running:
            try:
                frame = None
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame = self.latest_frame.copy()
                if frame is None:
                    time.sleep(0.1)
                    continue

                # throttle inference to reduce CPU load
                if frame_counter % 5 == 0:
                    # call model
                    results = self.yolo(frame, verbose=False, imgsz=640, conf=0.25, max_det=1)
                    detections = results[0].boxes

                    if detections:
                        det = detections[0]
                        try:
                            classidx = int(det.cls.item())
                            classname = self.yolo.names.get(classidx, 'unknown')
                        except Exception:
                            classname = getattr(det, 'cls', 'unknown')
                        conf = float(det.conf.item() if hasattr(det, 'conf') else 0.0)

                        now = time.time()
                        if conf > 0.5 and (now - self._last_detection_time) >= DETECTION_COOLDOWN:
                            self._last_detection_time = now

                            sort_cmd = self._classname_to_sort(classname)

                            # send to Arduino only if open
                            if self.serial and getattr(self.serial, "is_open", False):
                                sent = self.send_cmd(f"SORT,{sort_cmd}")
                                if sent:
                                    self.logmsg(f"Detected {classname} ({conf:.2f}) -> SORT,{sort_cmd}")
                                else:
                                    self.logmsg(f"Detected {classname} ({conf:.2f}) -> failed to send SORT,{sort_cmd}")
                            else:
                                # Don't spam logs too much; minimal log for detection without serial
                                self.logmsg(f"Detected {classname} ({conf:.2f}) - serial closed, skipping SORT")

                            # annotate frame safely and schedule display
                            xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
                            color = BOX_COLORS.get(classname.lower(), (0, 255, 0))
                            cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                            cv2.putText(frame, f"{classname}: {int(conf*100)}%", (xyxy[0], xyxy[1]-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                            display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                            # schedule on main thread
                            self.root.after(0, self._display_frame_on_canvas, display)

                frame_counter += 1
                time.sleep(1/30)

            except Exception as e:
                # keep detection alive on errors; log once per error occurrence
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
            if self.serial and getattr(self.serial, "is_open", False):
                self.serial.close()
        except Exception:
            pass
        # Close any OpenCV windows if used
        try:
            cv2.destroyAllWindows()
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
            # delay YOLO start to allow camera warm-up
            root.after(2000, lambda: app.start_detection())
        except Exception as e:
            app.logmsg("Auto-start failed: " + str(e))

    # delay auto-start slightly so UI fully loads
    root.after(1000, auto_start)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    except Exception as e:
        print("Fatal error:", e)
        app.on_close()