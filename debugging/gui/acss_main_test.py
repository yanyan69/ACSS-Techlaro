#!/usr/bin/env python3
"""
ACSS GUI Skeleton with Tabs (Modern Design + Frames + Log + About + Exit)
Integrated with features from debug.py for component testing and live status.
Added Run Whole System functionality based on flowchart.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time
import csv

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
#LOG_FILE = "ac This paper uses PLS regression on full spectrum, no specific wavelengths.

#Instrument S400, probably 900-1700nm or standard NIR range.

#Not specific bands for AS7263.

#Another  for coco-peat (coconut coir), moisture.

#URL https://www.sciencedirect.com/science/article/am/pii/S1350449520305302

#Browse it.

#Instructions: "Extract the method for moisture content, wavelengths, model"

#ut it's for coco-peat, not copra, but similar coconut product.

#Call.```python
#!/usr/bin/env python3
"""
ACSS component test GUI for Raspberry Pi 5.
- Uses Arduino as IO: expect Arduino serial protocol:
  Arduino -> Pi: "IR\n" when IR triggered, "AS:val1,val2,...\n" when responding to REQ_AS, "ACK\n" for actions ack
  Pi -> Arduino: "SORT,L\n" or "SORT,C\n" or "SORT,R\n" ; "REQ_AS\n" ; "MOTOR,ON\n" / "MOTOR,OFF\n"
Notes:
- Update SERIAL_PORT if Arduino is on another device.
- This GUI is defensive: each test wrapped in try/except to allow one-by-one testing.
"""

import sys, threading, time # CHANGED: Removed unused imports (collections, traceback)

# Optional imports (we'll try them and gracefully degrade)
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
    print("serial is not available") # CHANGED: Fixed print statement to "not available"

try:
    import cv2
    import numpy as np
    print("cv2 and numpy are available") # CHANGED: Fixed grammar ("is" to "are")
except Exception:
    cv2 = None
    np = None
    print("cv2 and numpy are not available") # CHANGED: Fixed grammar and "not"

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
    print("PIL is not available") # CHANGED: Fixed print statement to "PIL is not available"

import csv  # Added for logging to CSV

# ------------------ USER SETTINGS ------------------
SERIAL_PORT = "/dev/ttyUSB0"   # change if needed
SERIAL_BAUD = 9600  # Matched to Arduino
YOLO_MODEL_PATH = "my_model/train/weights/best.pt"  # only used if ultralytics available; optional
CAM_PREVIEW_SIZE = (480, 360)
LOG_FILE = "acss_log.csv"
# ----------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Component Tester")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True

        # UI elements
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)
        frm.rowconfigure(2, weight=1)      # row with log frame expands
        frm.columnconfigure(0, weight=1)   # left column expands
        frm.columnconfigure(1, weight=1)   # right column expands
        # connection controls
        conn_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        conn_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).grid(row=0, column=1)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).grid(row=0, column=2)

        # component tests
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

        # Added for whole system test
        tk.Button(tests_frame, text="Start System", command=self.start_system).grid(row=3, column=0, columnspan=2, padx=2, pady=2, sticky="ew")
        tk.Button(tests_frame, text="Stop System", command=self.stop_system).grid(row=3, column=2, padx=2, pady=2)

        # camera controls
        cam_frame = tk.LabelFrame(frm, text="Camera / YOLO")
        cam_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack()
        self.camera_running = False
        tk.Button(cam_frame, text="Start Camera Preview", command=self.start_camera).pack(side='left', padx=4, pady=4)
        tk.Button(cam_frame, text="Stop Camera", command=self.stop_camera).pack(side='left', padx=4, pady=4)
        # CHANGED: Removed "Run Single YOLO Frame" button as per request

        # logger
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(
            log_frame, height=6, width=80, state='disabled', wrap='none'
        )
        self.log.pack(fill='both', expand=True)

        # internal state
        self.ir_monitoring = False
        self.ir_thread = None
        self.camera_thread = None
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None  # BGR OpenCV frame
        self.ir_state = False
        self.last_ir_report_time = time.time()
        self.as_values = []
        self.system_running = False
        self.system_thread = None
        self.last_ir_state = False  # For edge detection in system loop

        # try load YOLO if available
        if ULTRALYTICS_AVAILABLE:
            try:
                self.logmsg("Loading YOLO model (may take a while)...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self.logmsg("YOLO load failed: " + str(ex))

        # start serial reader background if port auto-opened later
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # CHANGED: Added dynamic button disabling for camera if not available
        if not PICAMERA2_AVAILABLE:
            for widget in cam_frame.winfo_children():
                if isinstance(widget, tk.Button) and widget['text'] in ("Start Camera Preview", "Stop Camera"):
                    widget.config(state='disabled')

    # ---------- Logging ----------
    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert('end', full_msg + "\n")
        self.log.see('end')
        self.log.config(state='disabled')
        print(full_msg)  # Console print for debugging

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
            # start reader thread
            t = threading.Thread(target=self.serial_reader_thread, daemon=True)
            t.start()
            self.logmsg("Serial reader thread started.")  # Extra debug
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
        """Send short ASCII command to Arduino with newline."""
        if not self.serial or not self.serial.is_open:
            self.logmsg("Serial not open. Cannot send: " + text)
            return
        try:
            with self.serial_lock:
                self.serial.write((text + "\n").encode())
                self.logmsg(f"TX -> {text} (sent successfully)")
        except Exception as e:
            self.logmsg("Serial write failed: " + str(e))

    def send_sort(self, char):
        if char not in ('L', 'C', 'R'):  # CHANGED: Added spaces around operators for consistency
            self.logmsg(f"Invalid sort direction: {char}")
            return
        self.logmsg(f"Sending SORT,{char} for servo test")
        self.send_cmd(f"SORT,{char}")

    # ---------- Serial reader ----------
    def serial_reader_thread(self):
        self.logmsg("Serial reader started.")
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    # if IR was last seen but now silent for >1s, assume no object
                    if self.ir_monitoring and self.ir_state and (time.time() - self.last_ir_report_time > 1.0):  # CHANGED: Gated IR timeout logic with self.ir_monitoring
                        self.ir_state = False
                        self.logmsg("IR: No object detected (timeout)")
                    continue

                self.logmsg(f"RX <- {line} (received from Arduino)")

                if self.ir_monitoring and line == "IR":  # CHANGED: Gated IR handling with self.ir_monitoring; assumes binary detection (no distance yet, as per protocol)
                    self.last_ir_report_time = time.time()
                    if not self.ir_state:  # state changed
                        self.ir_state = True
                        self.logmsg("IR: Object detected within proximity")  # NOTE: For distance, update Arduino to send e.g., "IR:distance_value\n" and parse here
                elif line.startswith("AS:"):
                    payload = line[3:]
                    vals = payload.split(",")
                    try:
                        self.as_values = [float(v) for v in vals]
                        self.logmsg(f"AS values: {self.as_values}")
                    except ValueError:
                        self.logmsg("Invalid AS values")
                elif line == "ACK":
                    self.logmsg("ACK from Arduino (action completed)")
                else:
                    self.logmsg("Serial raw: " + line)

            except Exception as e:
                self.logmsg("Serial reader exception: " + str(e))
                time.sleep(0.1)

    # ---------- IR Monitor control ----------
    def start_ir_monitor(self):
        if not (self.serial and self.serial.is_open):
            self.logmsg("Open serial first.")
            return
        if self.ir_monitoring:
            self.logmsg("IR monitor already running.")
            return
        self.ir_monitoring = True
        self.logmsg("IR monitor started (listening to serial IR events). Arduino should send 'IR' continuously when triggered.")

    def stop_ir_monitor(self):
        self.ir_monitoring = False
        self.logmsg("IR monitor stopped.")

    # ---------- AS request ----------
    def request_as(self):
        if not (self.serial and self.serial.is_open):
            self.logmsg("Open serial first.")
            return
        # send REQ_AS and wait for one response line that starts with "AS:"
        try:
            self.logmsg("Sending REQ_AS and waiting for response...")
            with self.serial_lock:
                self.serial.reset_input_buffer()
                self.serial.write(b"REQ_AS\n")
                self.logmsg("REQ_AS sent.")
                t0 = time.time()
                timeout = 2.0
                got = False
                while time.time() - t0 < timeout:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    self.logmsg(f"RX during AS wait <- {line}")
                    if line.startswith("AS:"):
                        got = True
                        break
                if not got:
                    self.logmsg("No AS response within timeout.")
                else:
                    self.logmsg("AS response received successfully.")
        except Exception as e:
            self.logmsg("REQ_AS failed: " + str(e))

    # ---------- Camera + YOLO ----------
    def start_camera(self):
        if self.camera_running:
            self.logmsg("Camera already running.")
            return
        if not PICAMERA2_AVAILABLE:
            self.logmsg("picamera2 not available on this system.")
            return
        try:
            self.picam2 = Picamera2()
            # Match your capture.py settings
            config = self.picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self.logmsg("Camera started with XRGB8888 @ 640x480.")
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
        frame_counter = 0  # CHANGED: Added counter to throttle YOLO (run every 5 frames to reduce CPU load)
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

                # --- YOLO INFERENCE (throttled) ---
                if self.yolo and frame_counter % 5 == 0:  # CHANGED: Throttled YOLO to every 5th frame for performance
                    results = self.yolo(frame, verbose=False, imgsz=640, conf=0.25, max_det=5)  # CHANGED: Made params consistent (added imgsz, conf, max_det for better control)
                    detections = results[0].boxes
                    object_count = 0

                    for det in detections:
                        xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
                        xmin, ymin, xmax, ymax = xyxy
                        classidx = int(det.cls.item())
                        classname = self.yolo.names[classidx]
                        conf = det.conf.item()

                        if conf > 0.5:
                            color = (0, 255, 0)  # you can randomize or map colors
                            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                            label = f"{classname}: {int(conf*100)}%"
                            cv2.putText(frame, label, (xmin, max(ymin - 10, 10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                            object_count += 1

                    # Show object count on screen
                    cv2.putText(frame, f"Objects: {object_count}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

                frame_counter += 1

                # --- pass to Tkinter canvas (resized for preview) ---
                with self.frame_lock:
                    self.latest_frame = frame.copy()  # Keep full-size for potential other uses
                display_frame = cv2.resize(frame, CAM_PREVIEW_SIZE)  # CHANGED: Added resize for display to fit 480x360 preview without cropping/distortion
                self.update_canvas_with_frame(display_frame)

                time.sleep(1/30)  # CHANGED: Adjusted sleep to ~30 FPS for smoother preview

            except Exception as e:
                self.logmsg("Camera loop error: " + str(e))  # CHANGED: Renamed from "YOLO loop error" since it's broader
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            # Convert to RGB for Tkinter
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            if PIL_AVAILABLE:
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                # keep reference to avoid GC
                self.cam_canvas.imgtk = imgtk
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
            else:
                # fallback: write a temporary file and use tk.PhotoImage (slow) OR skip
                # We'll fallback to OpenCV window if no PIL
                cv2.imshow("Camera Preview", bgr_frame)
                cv2.waitKey(1)
        except Exception as e:
            self.logmsg("Display frame error: " + str(e))

    # ---------- Whole System Test ----------
    def start_system(self):
        if self.system_running:
            self.logmsg("System already running.")
            return
        if not (self.serial and self.serial.is_open):
            self.open_serial()
        if not self.camera_running:
            self.start_camera()
        if not self.ir_monitoring:
            self.start_ir_monitor()
        if self.yolo is None:
            self.logmsg("YOLO model not loaded. Cannot start system.")
            return
        self.system_running = True
        self.system_thread = threading.Thread(target=self.system_loop, daemon=True)
        self.system_thread.start()
        self.logmsg("Whole system started.")

    def stop_system(self):
        if not self.system_running:
            self.logmsg("System not running.")
            return
        self.system_running = False
        self.logmsg("Whole system stopped.")

    def system_loop(self):
        self.logmsg("System loop started.")
        with open(LOG_FILE, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "yolo_class", "moisture_value", "final_class", "as_readings"])
            while self.system_running:
                if self.ir_state and not self.last_ir_state:
                    # Process copra
                    self.logmsg("IR detected - processing copra.")
                    # Capture frame
                    frame = None
                    with self.frame_lock:
                        if self.latest_frame is not None:
                            frame = self.latest_frame.copy()
                    if frame is None:
                        self.logmsg("No frame available.")
                        time.sleep(0.1)
                        continue

                    # Run YOLO
                    yolo_class = 'unknown'
                    confidence = 0.0
                    if self.yolo:
                        results = self.yolo(frame, verbose=False, imgsz=640, conf=0.25, max_det=5)
                        detections = results[0].boxes
                        if detections:
                            # Take the detection with highest confidence
                            max_det = max(detections, key=lambda d: d.conf.item())
                            classidx = int(max_det.cls.item())
                            yolo_class = self.yolo.names.get(classidx, 'unknown')  # Assume names: 0:'raw', 1:'standard', 2:'overcooked'
                            confidence = max_det.conf.item()
                            self.logmsg(f"YOLO class: {yolo_class} ({confidence:.2f})")

                    # Request AS7263
                    self.as_values = []  # Clear previous
                    self.request_as()
                    time.sleep(2.1)  # Wait for response

                    # Compute moisture
                    moisture = 0.0
                    moisture_class = 'unknown'
                    if self.as_values and len(self.as_values) == 6:
                        # Placeholder for moisture calculation (CH_R or regression); replace with calibrated model e.g., PLS or MLR
                        # For example, using channel U (760nm) as proxy, assume values 0-100 reflectance, higher moisture lower reflectance
                        moisture = 10.0 - (self.as_values[3] / 10.0)  # Arbitrary formula to fit ~5-7% range
                        if moisture <= 5.9:
                            moisture_class = 'overcooked'
                        elif 6.0 <= moisture <= 7.0:
                            moisture_class = 'standard'
                        elif moisture >= 7.1:
                            moisture_class = 'raw'
                        else:
                            moisture_class = 'unknown'
                        self.logmsg(f"Moisture: {moisture:.1f}% -> {moisture_class}")

                    # Combine results (priority to moisture if mismatch)
                    final_class = moisture_class
                    if yolo_class != 'unknown' and moisture_class != 'unknown' and yolo_class != moisture_class:
                        self.logmsg(f"Mismatch: YOLO {yolo_class} vs Moisture {moisture_class} - using moisture")

                    # Servo sorting
                    sort_map = {'overcooked': 'L', 'standard': 'C', 'raw': 'R', 'unknown': 'C'}
                    sort_dir = sort_map.get(final_class, 'C')
                    self.send_sort(sort_dir)
                    time.sleep(1.0)  # Wait for ACK/sorting

                    # Logging
                    ts = time.strftime("%Y-%m-%d %H:%M:%S")
                    writer.writerow([ts, yolo_class, f"{moisture:.1f}", final_class, ",".join(map(str, self.as_values))])
                    csvfile.flush()
                    self.logmsg(f"Processed: {final_class}")

                self.last_ir_state = self.ir_state
                time.sleep(0.1)
        self.logmsg("System loop ended.")

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
        self.camera_running = False
        self.system_running = False
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
    root.mainloop()