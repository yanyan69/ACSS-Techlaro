#!/usr/bin/env python3
"""
ACSS YOLO + Servo Test GUI for Raspberry Pi 5.
- Uses Arduino for servo control via serial: sends "SORT,L\n" or "SORT,C\n" or "SORT,R\n" based on YOLO detection.
- Detects 'raw-copra', 'standard-copra', 'overcooked-copra' and maps:
  - raw-copra -> SORT,R (right)
  - standard-copra -> SORT,C (center)
  - overcooked-copra -> SORT,L (left)
- Bounding box stays visible until replaced by a new detection.
"""

import sys
import threading
import time

# --- Optional Imports (for Raspberry Pi) ---
try:
    import tkinter as tk
    from tkinter import scrolledtext
except Exception as e:
    print("tkinter missing:", e)
    sys.exit(1)

try:
    import serial
except Exception:
    serial = None
    print("pyserial missing")

try:
    import cv2, numpy as np
except Exception:
    cv2 = None
    np = None
    print("cv2/numpy missing")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
    print("picamera2 missing")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("ultralytics missing")

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False
    print("PIL missing")

# ------------------ SETTINGS ------------------
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"
CAM_PREVIEW_SIZE = (480, 360)
DETECTION_COOLDOWN = 0.6

# --- Bounding Box Colors (BGR) ---
BOX_COLORS = {
    'raw-copra': (255, 0, 123),       # Blue
    'standard-copra': (40, 167, 69),  # Green
    'overcooked-copra': (220, 53, 69) # Red
}
# -------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS YOLO + Servo Tester")

        self.serial = None
        self.running = True
        self.camera_running = False
        self.detection_running = False
        self.latest_frame = None
        self.last_detected_frame = None
        self.last_detection_time = 0.0

        # ---- GUI SETUP ----
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure((0, 1), weight=1)

        conn = tk.LabelFrame(frm, text="Serial / Arduino")
        conn.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn, text="Open Serial", command=self.open_serial).grid(row=0, column=1)
        tk.Button(conn, text="Close Serial", command=self.close_serial).grid(row=0, column=2)

        tests = tk.LabelFrame(frm, text="Servo Tests")
        tests.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        tk.Button(tests, text="LEFT", command=lambda: self.send_sort('L')).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests, text="CENTER", command=lambda: self.send_sort('C')).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests, text="RIGHT", command=lambda: self.send_sort('R')).grid(row=0, column=2, padx=2, pady=2)
        tk.Button(tests, text="Start Detection", command=self.start_detection).grid(row=1, column=0, columnspan=2, sticky="ew", pady=2)
        tk.Button(tests, text="Stop Detection", command=self.stop_detection).grid(row=1, column=2, pady=2)

        cam = tk.LabelFrame(frm, text="Camera / YOLO")
        cam.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.canvas = tk.Canvas(cam, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.canvas.pack()
        tk.Button(cam, text="Start Camera", command=self.start_camera).pack(side='left', padx=4, pady=4)
        tk.Button(cam, text="Stop Camera", command=self.stop_camera).pack(side='left', padx=4, pady=4)

        logf = tk.LabelFrame(frm, text="Log")
        logf.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(logf, height=6, width=80, state='disabled')
        self.log.pack(fill='both', expand=True)

        self.frame_lock = threading.Lock()
        self._tk_img = None

        # YOLO Load
        self.yolo = None
        if ULTRALYTICS_AVAILABLE:
            try:
                self.logmsg("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg("YOLO model loaded.")
            except Exception as e:
                self.logmsg(f"YOLO load failed: {e}")

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Logging ----------
    def logmsg(self, text):
        ts = time.strftime("%H:%M:%S")
        msg = f"[{ts}] {text}"
        print(msg)
        self.log.config(state='normal')
        self.log.insert('end', msg + "\n")
        self.log.see('end')
        self.log.config(state='disabled')

    # ---------- Serial ----------
    def open_serial(self):
        if not serial:
            self.logmsg("pyserial not available.")
            return
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"Serial opened on {SERIAL_PORT}")
        except Exception as e:
            self.logmsg(f"Serial open failed: {e}")

    def close_serial(self):
        if self.serial:
            self.serial.close()
            self.serial = None
            self.logmsg("Serial closed.")

    def send_sort(self, char):
        if self.serial and self.serial.is_open:
            self.serial.write(f"SORT,{char}\n".encode())
            self.logmsg(f"SORT command sent: {char}")
        else:
            self.logmsg(f"(Offline) SORT,{char}")

    # ---------- Camera ----------
    def start_camera(self):
        if not PICAMERA2_AVAILABLE:
            self.logmsg("Picamera2 not available.")
            return
        if self.camera_running:
            self.logmsg("Camera already running.")
            return
        try:
            self.picam2 = Picamera2()
            cfg = self.picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
            self.picam2.configure(cfg)
            self.picam2.start()
            self.camera_running = True
            threading.Thread(target=self.camera_loop, daemon=True).start()
            self.logmsg("Camera started.")
        except Exception as e:
            self.logmsg(f"Camera start failed: {e}")

    def stop_camera(self):
        self.camera_running = False
        try:
            if self.picam2:
                self.picam2.stop()
            self.logmsg("Camera stopped.")
        except Exception as e:
            self.logmsg(f"Camera stop failed: {e}")

    def camera_loop(self):
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                # Only update if no detection frame currently held
                if self.last_detected_frame is None:
                    display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                    self.root.after(0, self._show_frame, display)
                time.sleep(1/30)
            except Exception as e:
                self.logmsg(f"Camera loop error: {e}")
                time.sleep(0.1)

    def _show_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        self._tk_img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0, 0, anchor='nw', image=self._tk_img)

    # ---------- Detection ----------
    def start_detection(self):
        if not self.yolo:
            self.logmsg("YOLO model missing.")
            return
        if self.detection_running:
            self.logmsg("Detection already running.")
            return
        self.detection_running = True
        threading.Thread(target=self.detection_loop, daemon=True).start()
        self.logmsg("Detection started.")

    def stop_detection(self):
        self.detection_running = False
        self.last_detected_frame = None
        self.logmsg("Detection stopped.")

    def _classname_to_sort(self, name):
        name = name.lower()
        if "raw" in name:
            return "R"
        elif "standard" in name:
            return "C"
        elif "overcook" in name:
            return "L"
        return "C"

    def detection_loop(self):
        frame_count = 0
        while self.detection_running:
            try:
                with self.frame_lock:
                    if self.latest_frame is None:
                        continue
                    frame = self.latest_frame.copy()

                if frame_count % 5 == 0:
                    results = self.yolo(frame, verbose=False, imgsz=640, conf=0.25, max_det=1)
                    dets = results[0].boxes
                    if dets:
                        det = dets[0]
                        class_id = int(det.cls.item())
                        name = self.yolo.names[class_id].lower()
                        conf = float(det.conf.item())

                        if conf > 0.5:
                            xyxy = det.xyxy.cpu().numpy().astype(int).squeeze()
                            color = BOX_COLORS.get(name, (40, 167, 69))  # default green
                            cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                            cv2.putText(frame, f"{name} {int(conf*100)}%", (xyxy[0], xyxy[1]-8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                            # show this annotated frame persistently
                            self.last_detected_frame = frame.copy()
                            display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                            self.root.after(0, self._show_frame, display)

                            sort_cmd = self._classname_to_sort(name)
                            self.send_sort(sort_cmd)
                            self.logmsg(f"Detected {name} ({conf:.2f}) -> SORT,{sort_cmd}")

                frame_count += 1
                time.sleep(1/30)

            except Exception as e:
                self.logmsg(f"Detection error: {e}")
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
            app.logmsg("Camera auto-started.")
            root.after(2000, app.start_detection)
        except Exception as e:
            app.logmsg(f"Auto-start failed: {e}")

    root.after(1000, auto_start)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()