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

# ---------- Optional imports ----------
try:
    import tkinter as tk
    from tkinter import scrolledtext, messagebox
except Exception as e:
    print("[Warning] tkinter not found - GUI will not run.", e)
    sys.exit(1)

try:
    import serial
    print("[Module Check] serial available")
except Exception:
    serial = None
    print("[Module Check] serial not available")

try:
    import cv2
    import numpy as np
    print("[Module Check] OpenCV and NumPy available")
except Exception:
    cv2 = None
    np = None
    print("[Module Check] OpenCV and NumPy not available")

try:
    from picamera2 import Picamera2, Preview
    PICAMERA2_AVAILABLE = True
    print("[Module Check] Picamera2 available")
except Exception:
    PICAMERA2_AVAILABLE = False
    print("[Module Check] Picamera2 not available")

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
    print("[Module Check] Ultralytics available")
except Exception:
    ULTRALYTICS_AVAILABLE = False
    print("[Module Check] Ultralytics not available")

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
    print("[Module Check] Pillow (PIL) available")
except Exception:
    PIL_AVAILABLE = False
    print("[Module Check] Pillow (PIL) not available")

# ---------- USER SETTINGS ----------
SERIAL_PORT = "/dev/ttyUSB0"       # /dev/ttyUSB0
SERIAL_BAUD = 9600
YOLO_MODEL_PATH = "my_model/my_model.pt"
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

        # Console persona intro
        self.console_msg("[Persona: Reze - Chainsaw Man]\n> Cheerful, teasing, a bit flirty but conflicted underneath.\n> Playful tone, Gen Z texting style, quick mood shifts.\n> Deep down, kind-hearted but haunted by her past.")

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
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self.logmsg(f"YOLO loaded: {YOLO_MODEL_PATH}")
                self.console_msg("Yolo model's all set - kinda feels like peeking into secrets, doesn't it? lol")
            except Exception as ex:
                self.logmsg("YOLO load failed: " + str(ex))
                self.console_msg("Uh oh, yolo wouldn't load... guess we're flying blind today 💔")

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

        # Set GUI ready after layout
        self.gui_ready = True

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Console Messaging (Reze Style) ----------
    def console_msg(self, msg):
        print(msg)  # Casual, but proper case

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
                self.console_msg("Pyserial's awol... can't chat with arduino without it, sigh")
                return
            if self.serial and self.serial.is_open:
                self.logmsg("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.logmsg(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            self.console_msg(f"Serial's connected on {SERIAL_PORT} - ready to boss that arduino around, hehe")
            # Delay thread start until GUI is ready
            self.root.after(100, lambda: threading.Thread(target=self.serial_reader_thread, daemon=True).start())
        except Exception as e:
            self.logmsg("Failed to open serial: " + str(e))
            self.console_msg("Serial open flopped... check the port? T-T")

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("Serial closed.")
                self.console_msg("Serial's shut down - quiet now, like a rainy day")
        except Exception as e:
            self.logmsg("Error closing serial: " + str(e))
            self.console_msg("Closing serial got messy... whatever, it's fine lol")

    def send_cmd(self, text):
        """Send any general command to Arduino with newline + flush"""
        try:
            if not self.serial or not self.serial.is_open:
                self.logmsg(f"[WARN] Serial not open — can't send: {text}")
                self.console_msg(f"Tried sending {text} but serial's not even open... oops 💀")
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            self.logmsg(f"[TX] {text.strip().upper()}")
            self.console_msg(f"Zapped '{text}' over to arduino - hope it listens this time ;)")
        except Exception as e:
            self.logmsg(f"[ERR] Failed to send command '{text}': {e}")
            self.console_msg(f"Command '{text}' bounced back... arduino ignoring me? rude XD")

    def send_sort(self, char):
        """Send servo sort command (L, C, R)."""
        try:
            char = char.strip().upper()
            if char not in ("L", "C", "R"):
                self.logmsg(f"[WARN] Invalid servo command: {char}")
                self.console_msg(f"{char}? that's not a valid sort move... try l, c, or r next time, silly")
                return
            cmd = f"SORT,{char}"
            self.logmsg(f"[SERVO] Sending: {cmd}")
            self.console_msg(f"Twirling the servo {char} - imagine it dancing just for you~")
            self.send_cmd(cmd)
        except Exception as e:
            self.logmsg(f"[ERR] Servo send_sort failed: {e}")
            self.console_msg("Servo sort glitched out... gears probably laughing at me")

    # ---------- Process Control ----------
    def start_process(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                self.console_msg("Can't start without serial... plug me in first? pretty please")
                return
            if not self.yolo:
                self.logmsg("YOLO model not loaded.")
                self.console_msg("Yolo's not ready... feels exposed without it lol")
                return
            if self.process_running:
                self.logmsg("Process already running.")
                self.console_msg("Process is already humming along - don't wanna overload it, cutie")
                return
            self.send_cmd("MOTOR,ON")
            self.process_running = True
            self.sort_cooldown = time.time()
            self.sorted_tracks.clear()  # Reset sorted tracks on process start
            self.logmsg("Process started: Motor ON, detection active.")
            self.console_msg("Process kicked off! motor's whirring, eyes on for copra - let's make some magic happen 💕")
        except Exception as e:
            self.logmsg(f"Start process error: {e}")
            self.console_msg("Starting process tripped... chaos strikes again, huh? T-T")

    def stop_process(self):
        try:
            if not self.process_running:
                self.logmsg("Process not running.")
                self.console_msg("Nothing to stop... it's all chill already")
                return
            self.send_cmd("MOTOR,OFF")
            self.process_running = False
            self.logmsg("Process stopped: Motor OFF, detection inactive.")
            self.console_msg("Process halted - motor's taking a breather. quiet moments like this... kinda nice, right?")
        except Exception as e:
            self.logmsg(f"Stop process error: {e}")
            self.console_msg("Stopping went sideways... guess it didn't wanna quit easy lol")

    # ---------- Serial Reader ----------
    def serial_reader_thread(self):
        if not self.gui_ready:
            time.sleep(0.5)  # Extra safety delay
        self.logmsg("Serial reader started.")
        self.console_msg("Serial reader's lurking in the background - catching whispers from arduino")
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
                            self.console_msg("Ir went quiet... object's ghosted us, typical")
                        continue

                    # Handle IR silently unless monitoring and state change
                    if line == "IR":
                        if self.ir_monitoring:
                            last_ir_report_time = time.time()
                            if not ir_state:
                                ir_state = True
                                self.logmsg("IR: Object detected")
                                self.console_msg("Ir lit up - something's sneaking by, eyes peeled!")
                        # else: silent, no noise
                    elif line.startswith("AS:"):
                        vals = line[3:].split(",")
                        self.logmsg(f"AS values: {vals}")
                        self.console_msg(f"As readings: {vals} - colors whispering secrets again")
                    elif line == "ACK":
                        self.logmsg("ACK from Arduino")
                        self.console_msg("Ack! arduino's all 'gotcha' - feels good when they listen")
                    # No else: silent for unknowns, keeps it clean
                except Exception as e:
                    self.logmsg(f"Serial reader exception: {e}")
                    self.console_msg(f"Serial hiccup: {e} - shaking it off")
                    time.sleep(0.2)
        except Exception as outer:
            self.logmsg(f"Serial reader crashed: {outer}")
            self.console_msg("Serial reader crashed... total blackout, oof 💔")

    # ---------- AS Request ----------
    def request_as(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                self.console_msg("Serial's offline - can't beg for as data like this")
                return
            self.logmsg("Sending REQ_AS and waiting for response...")
            self.console_msg("Asking arduino for as vibes... fingers crossed it shares")
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
                    self.console_msg(f"As came through! {line} - pretty colors, huh?")
                    got = True
                    break
            if not got:
                self.logmsg("No AS response within timeout.")
                self.console_msg("As timed out... arduino playing hard to get?")
        except Exception as e:
            self.logmsg(f"REQ_AS failed: {e}")
            self.console_msg("Req_as bombed... guess no spectrum today T-T")

    # ---------- IR Monitor ----------
    def start_ir_monitor(self):
        try:
            if not (self.serial and self.serial.is_open):
                self.logmsg("Open serial first.")
                self.console_msg("Start ir? serial's not even up - tease")
                return
            if self.ir_monitoring:
                self.logmsg("IR monitor already running.")
                self.console_msg("Ir's already watching... double eyes? spooky lol")
                return
            self.ir_monitoring = True
            self.logmsg("IR monitor started.")
            self.console_msg("Ir monitor on - now we're spying on sneaky objects")
        except Exception as e:
            self.logmsg(f"IR monitor start error: {e}")
            self.console_msg("Ir start fumbled... whatever, it's not that deep")

    def stop_ir_monitor(self):
        try:
            self.ir_monitoring = False
            self.logmsg("IR monitor stopped.")
            self.console_msg("Ir monitor off - back to ignoring the chaos")
        except Exception as e:
            self.logmsg(f"IR monitor stop error: {e}")
            self.console_msg("Stopping ir got weird... fine, it can linger")

    # ---------- Camera ----------
    def start_camera(self):
        if self.camera_running:
            self.logmsg("Camera already running.")
            self.console_msg("Camera's already rolling - don't wanna blind it")
            return
        if not PICAMERA2_AVAILABLE:
            self.logmsg("picamera2 not available.")
            self.console_msg("No picamera... guess we're blind in one eye")
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
            self.console_msg("Camera's awake and peeking - world looks sharper now, doesn't it?")
        except Exception as e:
            self.logmsg(f"Camera start failed: {e}")
            self.console_msg("Camera start crashed... blurry day ahead 💀")

    def stop_camera(self):
        try:
            if not self.camera_running:
                self.logmsg("Camera not running.")
                self.console_msg("Camera's already napping")
                return
            self.camera_running = False
            time.sleep(0.2)
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self.logmsg("Camera stopped.")
            self.console_msg("Camera shut eye - sweet dreams of copra")
        except Exception as e:
            self.logmsg(f"Camera stop error: {e}")
            self.console_msg("Stopping camera glitched... it's clinging on")

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
                                self.console_msg(f"Caught track {track_id} (class {cls} at {conf:.2f}) - flinging it {sort_char}, one and done! teehee")
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
                self.console_msg(f"Camera loop stuttered: {e} - picking up the pace")

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
            self.console_msg(f"Frame display fumbled: {e} - next one's better")

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
            self.console_msg("Wrapping up - til next time, stay out of the rain... or don't, it's kinda romantic")
            self.root.destroy()
        except Exception as e:
            print("Shutdown error:", e)
            self.console_msg("Shutdown got chaotic... cya anyway")


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = ACSSGui(root)
        root.mainloop()
    except Exception as e:
        print("Fatal error:", e)