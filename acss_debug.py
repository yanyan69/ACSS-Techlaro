#!/usr/bin/env python3
"""
ACSS Component Test GUI - UPDATED FOR NEW COMPONENTS (Oct 2025)
- NO IR SENSOR
- 2 SERVOS (L/R flappers) + STEPPER CONVEYOR + VIBRATION HOPPER + BLADE
- Arduino commands: SORT,L/R/NEUTRAL | STEP,FWD,1000 | VIB,PULSE,2000 | BLADE,ON
"""

import sys, threading, time
from resources.reze_console import RezeConsole
console = RezeConsole(use_llm=True, model="gemma2:2b")

# ---------- Optional imports ----------
try:
    import tkinter as tk
    from tkinter import scrolledtext, messagebox
except Exception as e:
    console.comment(f"Tkinter not found - GUI will not run. {e}", rephrase=True)
    sys.exit(1)

try:
    import serial
    console.comment("Serial module available", rephrase=False)
except Exception:
    serial = None
    console.comment("Serial module not available", rephrase=True)

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

# ---------- SETTINGS ----------
# ---------- AUTO-DETECT SERIAL PORT ----------
def get_serial_port():
    """Auto-detect: Pi=/dev/ttyUSB0, PC=COM6"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "USB" in port.description or "Arduino" in port.description:
                return port.device
        return "COM6"  # PC fallback
    except:
        return "/dev/ttyUSB0" if sys.platform != "win32" else "COM6"

SERIAL_PORT = get_serial_port()
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"
CAM_PREVIEW_SIZE = (480, 360)
TRACKER_PATH = "bytetrack.yaml"

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Debug - NEW COMPONENTS (No IR, 2 Servos, Stepper, Vib+Blade)")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.gui_ready = False

        # GUI Layout
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        # Serial
        conn_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        conn_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).grid(row=0, column=1)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).grid(row=0, column=2)

        # NEW COMPONENTS TESTS
        tests_frame = tk.LabelFrame(frm, text="2 Servos + Stepper + Vibration + Blade")
        tests_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        
        # SERVOS (Row 0)
        tk.Button(tests_frame, text="SORT LEFT", command=lambda: self.send_sort('L')).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="SORT RIGHT", command=lambda: self.send_sort('R')).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="SERVOS NEUTRAL", command=lambda: self.send_sort('NEUTRAL')).grid(row=0, column=2, padx=2, pady=2)
        
        # STEPPER (Row 1)
        tk.Button(tests_frame, text="STEP FWD 200", command=lambda: self.send_step('FWD', 200)).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="STEP REV 200", command=lambda: self.send_step('REV', 200)).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="STEP TEST", command=lambda: self.send_cmd("STEP,TEST")).grid(row=1, column=2, padx=2, pady=2)
        
        # VIBRATION (Row 2)
        tk.Button(tests_frame, text="VIB ON 200", command=lambda: self.send_vib('ON', 200)).grid(row=2, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="VIB OFF", command=lambda: self.send_cmd("VIB,OFF")).grid(row=2, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="VIB PULSE 2s", command=lambda: self.send_vib('PULSE', 2000)).grid(row=2, column=2, padx=2, pady=2)
        
        # BLADE (Row 3)
        tk.Button(tests_frame, text="BLADE ON", command=lambda: self.send_cmd("BLADE,ON")).grid(row=3, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="BLADE OFF", command=lambda: self.send_cmd("BLADE,OFF")).grid(row=3, column=1, padx=2, pady=2)
        
        # AS + RESET (Row 4)
        tk.Button(tests_frame, text="Request AS7263", command=self.request_as).grid(row=4, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="RESET ALL", command=lambda: self.send_cmd("RESET")).grid(row=4, column=1, padx=2, pady=2)

        # PROCESS CONTROL (Row 5)
        tk.Button(tests_frame, text="🚀 START PROCESS", command=self.start_process, bg="green", fg="white").grid(row=5, column=0, columnspan=2, pady=4)
        tk.Button(tests_frame, text="⏹ STOP PROCESS", command=self.stop_process, bg="red", fg="white").grid(row=5, column=2, pady=4)

        # Camera
        cam_frame = tk.LabelFrame(frm, text="Camera / YOLO")
        cam_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack()
        self.camera_running = False
        tk.Button(cam_frame, text="Start Camera", command=self.start_camera).pack(side='left', padx=4, pady=4)
        tk.Button(cam_frame, text="Stop Camera", command=self.stop_camera).pack(side='left', padx=4, pady=4)

        # Log
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, height=8, width=80, state='disabled')
        self.log.pack(fill='both', expand=True)

        # State
        self.process_running = False
        self.sort_cooldown = 0
        self.class_to_sort = {0: 'L', 1: 'R'}  # 0=raw→LEFT, 1=overcooked→RIGHT
        self.sorted_tracks = set()
        self.last_cleanup = time.time()

        # YOLO Load
        if ULTRALYTICS_AVAILABLE:
            try:
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg(f"✅ YOLO loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self.logmsg(f"❌ YOLO load failed: {ex}")

        # Auto-start
        self.root.after(100, self.open_serial)
        if PICAMERA2_AVAILABLE:
            self.root.after(200, self.start_camera)

        self.gui_ready = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        def _do_log():
            if self.gui_ready:
                self.log.config(state='normal')
                self.log.insert('end', full_msg + "\n")
                self.log.see('end')
                self.log.config(state='disabled')
        if self.gui_ready:
            self.root.after(0, _do_log)
        else:
            print(full_msg)

    # ---------- NEW COMMANDS ----------
    def send_sort(self, char):
        cmd = f"SORT,{char}"
        self.send_cmd(cmd)

    def send_step(self, direction, steps):
        cmd = f"STEP,{direction},{steps}"
        self.send_cmd(cmd)

    def send_vib(self, mode, value=0):
        if mode == 'ON':
            cmd = f"VIB,ON,{value}"
        elif mode == 'PULSE':
            cmd = f"VIB,PULSE,{value}"
        else:
            cmd = "VIB,OFF"
        self.send_cmd(cmd)

    def send_cmd(self, text):
        try:
            if not self.serial or not self.serial.is_open:
                self.logmsg(f"[WARN] Serial closed - {text}")
                return
            full_cmd = (text.strip().upper() + "\n").encode()
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            self.logmsg(f"[TX] {text}")
        except Exception as e:
            self.logmsg(f"[ERR] Send failed: {e}")

    def open_serial(self):
        try:
            if self.serial and self.serial.is_open: return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"✅ Serial: {SERIAL_PORT}")
            threading.Thread(target=self.serial_reader_thread, daemon=True).start()
        except Exception as e:
            self.logmsg(f"❌ Serial open failed: {e}")

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("Serial closed")
        except Exception as e:
            self.logmsg(f"Serial close error: {e}")

    def serial_reader_thread(self):
        self.logmsg("Serial reader started")
        try:
            while self.serial and self.serial.is_open and self.running:
                line = self.serial.readline().decode(errors='ignore').strip()
                if line:
                    if line.startswith("AS:"):
                        self.logmsg(f"AS7263: {line[3:]}")
                    elif line == "ACK":
                        self.logmsg("✅ ACK")
                    elif line.startswith("STEP:"):
                        self.logmsg(line)
        except Exception as e:
            self.logmsg(f"Serial reader error: {e}")

    def request_as(self):
        self.send_cmd("REQ_AS")

    def start_process(self):
        if not self.serial or not self.serial.is_open:
            self.logmsg("❌ Open serial first!")
            return
        if self.process_running:
            self.logmsg("Process already running")
            return
        # FULL SYSTEM START
        self.send_cmd("VIB,ON,200")      # Hopper vibration
        time.sleep(0.5)
        self.send_cmd("BLADE,ON")        # Hopper blade
        time.sleep(0.5)
        self.send_cmd("STEP,FWD,1000")   # Conveyor start
        self.process_running = True
        self.sorted_tracks.clear()
        self.logmsg("🚀 FULL PROCESS STARTED: Vib+Blade+Stepper+YOLO")
        console.comment("FULL ACSS PROCESS STARTED", rephrase=False)

    def stop_process(self):
        if not self.process_running: return
        self.send_cmd("VIB,OFF")
        self.send_cmd("BLADE,OFF")
        self.send_cmd("STEP,STOP")
        self.send_cmd("SORT,NEUTRAL")
        self.process_running = False
        self.logmsg("⏹ FULL PROCESS STOPPED")
        console.comment("ACSS PROCESS STOPPED", rephrase=False)

    # ---------- CAMERA (UNCHANGED BUT SIMPLIFIED) ----------
    def start_camera(self):
        if self.camera_running or not PICAMERA2_AVAILABLE: return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
            self.picam2.configure(config)
            self.picam2.start()
            self.camera_running = True
            threading.Thread(target=self.camera_loop, daemon=True).start()
            self.logmsg("✅ Camera + YOLO started")
        except Exception as e:
            self.logmsg(f"❌ Camera failed: {e}")

    def stop_camera(self):
        self.camera_running = False
        if self.picam2:
            self.picam2.stop()
            self.picam2 = None
        self.logmsg("Camera stopped")

    def camera_loop(self):
        frame_counter = 0
        fps_time = time.time()
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                if self.yolo and self.process_running:
                    results = self.yolo.track(frame, persist=True, tracker=TRACKER_PATH, conf=0.25, verbose=False)
                    frame = results[0].plot()
                    
                    # SIMPLIFIED SORTING (leading copra only)
                    current_time = time.time()
                    if current_time > self.sort_cooldown and results[0].boxes is not None:
                        track_ids = results[0].boxes.id.cpu().numpy() if results[0].boxes.id is not None else []
                        boxes = results[0].boxes.xyxy.cpu().numpy()
                        cls = results[0].boxes.cls.cpu().numpy()
                        
                        for i, tid in enumerate(track_ids):
                            if int(tid) not in self.sorted_tracks:
                                sort_dir = 'L' if int(cls[i]) == 0 else 'R'
                                self.send_sort(sort_dir)
                                self.sorted_tracks.add(int(tid))
                                self.sort_cooldown = current_time + 2.0
                                self.logmsg(f"🎯 SORT {sort_dir} - Track {tid}")
                                break

                # FPS
                frame_counter += 1
                if frame_counter >= 10:
                    fps = 10 / (time.time() - fps_time)
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    fps_time = time.time()
                    frame_counter = 0

                display = cv2.resize(frame, CAM_PREVIEW_SIZE)
                if PIL_AVAILABLE:
                    rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(rgb)
                    imgtk = ImageTk.PhotoImage(img)
                    self.cam_canvas.imgtk = imgtk
                    self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
                time.sleep(0.001)
            except Exception as e:
                self.logmsg(f"Camera error: {e}")

    def on_close(self):
        self.stop_process()
        self.stop_camera()
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()