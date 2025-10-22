#!/usr/bin/env python3
"""
ACSS GUI (Auto mode) - FIFO-aware, fully automated.
- Start/Stop removed: processing auto-starts when serial, camera, and YOLO are available.
- Classifications sent to Arduino as the category string ("RAW", "OVERCOOKED", or "STANDARD").
- Maintains GUI and setup, but removes sort_queue and flapper-triggered sorting since Arduino handles sorting.
- Updates stats immediately after successful classification send.
- Removes NIR requests since Arduino handles NIR timing.
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time
# Optional imports (graceful degradation)
try:
    import serial
    SERIAL_AVAILABLE = True
except Exception:
    serial = None
    SERIAL_AVAILABLE = False
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except Exception:
    cv2 = None
    np = None
    CV2_AVAILABLE = False
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:
    YOLO = None
    ULTRALYTICS_AVAILABLE = False
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False

# ------------------ USER SETTINGS ------------------
CAM_PREVIEW_SIZE = (640, 480)
USERNAME = "Copra Buyer 01"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
YOLO_MODEL_PATH = "my_model/my_model.pt"
SORT_ZONE_Y = 240
FALLBACK_ZONE_Y = 100
# ----------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System (AUTO)")
        self.root.attributes('-fullscreen', True)

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

        # State
        self.camera_running = False
        self.process_running = False  # will be set true automatically when ready
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.camera_thread = None
        self.picam2 = None
        self.yolo = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.serial_reader_thread_obj = None
        self.latest_detection = None
        self.serial_error_logged = False
        self.frame_drop_logged = False
        self.yolo_log_suppressed = True
        self.moisture_sums = {'Raw': 0.0, 'Standard': 0.0, 'Overcooked': 0.0}
        self.category_map = {0: 'Overcooked', 1: 'Raw', 2: 'Standard'}
        self.stats = {'Raw': 0, 'Standard': 0, 'Overcooked': 0, 'total': 0, 'start_time': None, 'end_time': None}

        # Arduino camera index event (ACK,AT_CAM)
        self.at_cam_event = threading.Event()
        self.at_cam_idx = None  # store index from ACK,AT_CAM

        # Load YOLO (non-blocking attempt)
        if ULTRALYTICS_AVAILABLE:
            try:
                self._log_message("Loading YOLO model...")
                self.yolo = YOLO(YOLO_MODEL_PATH)
                self._log_message(f"YOLO model loaded: {YOLO_MODEL_PATH}")
            except Exception as ex:
                self._log_message(f"YOLO load failed: {ex}")

        # Auto-start serial and camera; once all are ready -> auto-start processing
        self.open_serial()
        if PICAMERA2_AVAILABLE:
            self.start_camera()
        else:
            self._log_message("picamera2 not available; camera disabled.")

        # Start a watcher thread that starts processing automatically when prerequisites met
        threading.Thread(target=self._auto_start_watcher, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- UI BUILD ----------------
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

        # Process control removed; display status instead
        status_frame = tk.Frame(left_frame)
        status_frame.grid(row=1, column=0, sticky="ew", padx=4, pady=8)
        self.status_label = tk.Label(status_frame, text="Status: Initializing...", font=("Arial", 12, "bold"))
        self.status_label.pack(fill="x")

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
            self.process_running = False
            self.camera_running = False
            self.stop_process()
            self.stop_camera()
            self.close_serial()
            self.on_close()

    # ----------------- Serial -----------------
    def open_serial(self):
        try:
            if not SERIAL_AVAILABLE:
                self._log_message("pyserial not available.")
                return
            if self.serial and getattr(self.serial, "is_open", False):
                self._log_message("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self._log_message(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            self.serial_reader_thread_obj = threading.Thread(target=self.serial_reader_thread, daemon=True)
            self.serial_reader_thread_obj.start()
            self._log_message("Serial reader thread started.")
        except Exception as e:
            self._log_message(f"Failed to open serial: {e}")

    def close_serial(self):
        try:
            if self.serial and getattr(self.serial, "is_open", False):
                # tell arduino to reset if desired
                self.send_cmd("RESET")
                self.serial.close()
                self._log_message("Serial closed.")
        except Exception as e:
            self._log_message(f"Error closing serial: {e}")

    def send_cmd(self, text):
        try:
            if not self.serial or not getattr(self.serial, "is_open", False):
                self._log_message(f"Serial not open — can't send: {text}")
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
        except Exception as e:
            self._log_message(f"Failed to send command '{text}': {e}")

    def send_classification(self, category):
        """ Send classification to Arduino as just the category string. """
        try:
            category = category.strip().upper()
            if category not in ("OVERCOOKED", "RAW", "STANDARD"):
                self._log_message(f"Invalid classification: {category}")
                return False
            if not self.serial or not getattr(self.serial, "is_open", False):
                self._log_message(f"Serial not open — can't send classification: {category}")
                return False
            cmd = category
            with self.serial_lock:
                self.serial.write((cmd + "\n").encode())
                self.serial.flush()
            # wait for ACK,ASSIGN_CLASS... line that contains our category
            t0 = time.time()
            got_ack = False
            while time.time() - t0 < 1.5:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line: continue
                if line.startswith("ACK,ASSIGN_CLASS") and category in line:
                    got_ack = True
                    break
            if not got_ack:
                self._log_message(f"No ACK for classification {category}")
            else:
                self._log_message(f"Sent classification {category} — ACK received.")
            return got_ack
        except Exception as e:
            self._log_message(f"Classification send failed: {e}")
            return False

    def serial_reader_thread(self):
        self._log_message("Serial reader started.")
        try:
            while self.serial and getattr(self.serial, "is_open", False) and self.running:
                try:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    if not line: continue
                    # AS data (optional, not used for classification)
                    if line.startswith("AS:"):
                        vals = line[3:].split(",")
                        self._log_message(f"NIR data received: {vals}")
                    # Item at camera (ACK,AT_CAM,...)
                    elif line.startswith("ACK,AT_CAM"):
                        parts = line.split(",")
                        # look for IDX=... in remaining parts
                        idx_val = None
                        for p in parts:
                            if "IDX=" in p or "index=" in p.lower():
                                try:
                                    idx_val = p.split("=")[1]
                                except Exception:
                                    idx_val = None
                        self.at_cam_idx = idx_val
                        self._log_message(f"Item at camera (idx={self.at_cam_idx})")
                        self.at_cam_event.set()
                    elif line.startswith("ACK,AT_NIR"):
                        self._log_message(f"NIR window opened: {line}")
                    elif line == "ACK":  # generic ack - ignore
                        pass
                    else:
                        self._log_message(f"Serial: {line}")
                except Exception as e:
                    if not self.serial_error_logged:
                        self._log_message(f"Serial reader exception: {e}")
                        self.serial_error_logged = True
                    time.sleep(0.01)
        except Exception as outer:
            self._log_message(f"Serial reader crashed: {outer}")

    # ----------------- Process control (auto) -----------------
    def _auto_start_watcher(self):
        """ Wait until serial is open, camera is running, and YOLO loaded (if available), then start processing automatically. """
        start_attempted = False
        while self.running:
            serial_ready = (self.serial is not None and getattr(self.serial, "is_open", False))
            camera_ready = self.camera_running
            yolo_ready = (self.yolo is not None) if ULTRALYTICS_AVAILABLE else True
            if serial_ready and camera_ready and yolo_ready:
                if not start_attempted:
                    # start processing
                    self._log_message("Prerequisites met — starting AUTO processing.")
                    self.status_label.config(text="Status: AUTO running")
                    self.start_process()
                    start_attempted = True  # done; just monitor
            else:
                # show status
                parts = []
                parts.append("Serial" if serial_ready else "Serial:missing")
                parts.append("Camera" if camera_ready else "Camera:missing")
                parts.append("YOLO" if yolo_ready else "YOLO:loading")
                status = " | ".join(parts)
                try:
                    self.status_label.config(text=f"Status: Waiting ({status})")
                except Exception:
                    pass
            time.sleep(0.5)

    def start_process(self):
        """Mark the start time and enable processing."""
        if self.process_running:
            self._log_message("Process already running.")
            return
        self.stats['start_time'] = time.time()
        self.stats['end_time'] = None
        self.process_running = True
        self._log_message("AUTO processing enabled.")
        # Ensure Arduino is in auto mode
        self.send_cmd("MODE,AUTO")

    def stop_process(self):
        """Stop processing."""
        if not self.process_running:
            return
        self.process_running = False
        self.send_cmd("RESET")
        self.stats['end_time'] = time.time()
        self._log_message("Processing stopped.")

    def _update_stats_after_drop(self, category, moisture):
        try:
            self.stats[category] += 1
            self.stats['total'] = self.stats['Raw'] + self.stats['Standard'] + self.stats['Overcooked']
            self.moisture_sums[category] += moisture
            self.root.after(0, self.update_stats)
        except Exception as e:
            self._log_message(f"Stats update after drop error: {e}")

    # ----------------- Camera -----------------
    def start_camera(self):
        if self.camera_running:
            self._log_message("Camera already running.")
            return
        if not PICAMERA2_AVAILABLE:
            self._log_message("picamera2 not available.")
            return
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": CAM_PREVIEW_SIZE})
            self.picam2.configure(config)
            self.picam2.start()
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            self._log_message("Camera started.")
        except Exception as e:
            self._log_message(f"Camera start failed: {e}")
            self.camera_running = False

    def stop_camera(self):
        try:
            if not self.camera_running:
                return
            self.camera_running = False
            if self.picam2:
                self.picam2.stop()
                self.picam2 = None
            self._log_message("Camera stopped.")
        except Exception as e:
            self._log_message(f"Camera stop error: {e}")

    def camera_loop(self):
        last_frame_time = time.time()
        display_skip = 0
        while self.camera_running:
            try:
                frame = self.picam2.capture_array()
                # normalize frame
                if CV2_AVAILABLE:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                else:
                    frame = frame[:, :, :3]

                current_time = time.time()
                if current_time - last_frame_time > 0.2:
                    if not self.frame_drop_logged:
                        self._log_message(f"Frame drop detected: {current_time - last_frame_time:.3f}s")
                        self.frame_drop_logged = True
                last_frame_time = current_time

                results = None
                # Only attempt detection if process is running and there's a camera-at-item event
                if self.yolo and self.process_running and self.at_cam_event.is_set():
                    try:
                        t0 = time.time()
                        results = self.yolo.predict(source=frame, conf=0.3, max_det=3, verbose=False)
                        t1 = time.time()
                        self._log_message(f"YOLO inference time: {t1 - t0:.3f}s")
                    except Exception as ex:
                        if self.yolo_log_suppressed:
                            self._log_message(f"YOLO predict error: {ex}")
                            self.yolo_log_suppressed = False
                        results = None

                has_detection = results and results[0].boxes and len(results[0].boxes) > 0

                if self.process_running and self.at_cam_event.is_set():
                    if has_detection:
                        boxes = results[0].boxes.xyxy.cpu().numpy()
                        cls_tensor = results[0].boxes.cls.cpu().numpy().astype(int)
                        conf_tensor = results[0].boxes.conf.cpu().numpy()
                        candidates = []
                        for i in range(len(cls_tensor)):
                            y_center = (boxes[i, 1] + boxes[i, 3]) / 2
                            cls = cls_tensor[i]
                            conf = conf_tensor[i]
                            if conf > 0.3 and y_center < FALLBACK_ZONE_Y:
                                # compute moisture approximation (kept your heuristics)
                                if cls == 0:
                                    moisture = 4.0 + (conf - 0.3) * (6.0 - 4.0) / 0.7
                                elif cls == 1:
                                    moisture = 7.1 + (conf - 0.3) * (14.0 - 7.1) / 0.7
                                elif cls == 2:
                                    moisture = 6.0 + (conf - 0.3) * (7.0 - 6.0) / 0.7
                                else:
                                    cls = 0
                                    moisture = 4.0
                                moisture = round(moisture, 1)
                                candidates.append((cls, conf, y_center, moisture))
                        if candidates:
                            # choose the candidate closest to top (smallest y_center)
                            candidates.sort(key=lambda x: x[2])
                            leading = candidates[0]
                            cls, conf, y_center, moisture = leading
                            category = self.category_map.get(cls, 'Overcooked')
                            # Log classification before sending to ensure it appears before NIR logs
                            self._log_message(f"Classified {category} (moisture {moisture:.1f}%) conf {conf:.2f} idx={self.at_cam_idx}")
                            # Send classification and update stats if successful
                            ok = self.send_classification(category)
                            if ok:
                                self._update_stats_after_drop(category, moisture)
                            # clear the at_cam_event so we don't classify the same item repeatedly
                            self.at_cam_event.clear()
                        else:
                            category = 'Overcooked'
                            moisture = 4.0
                            self._log_message(f"No suitable candidate - Classified {category} (moisture {moisture:.1f}%) idx={self.at_cam_idx}")
                            ok = self.send_classification(category)
                            if ok:
                                self._update_stats_after_drop(category, moisture)
                            self.at_cam_event.clear()
                    else:
                        category = 'Overcooked'
                        moisture = 4.0
                        self._log_message(f"No detection - Classified {category} (moisture {moisture:.1f}%) idx={self.at_cam_idx}")
                        ok = self.send_classification(category)
                        if ok:
                            self._update_stats_after_drop(category, moisture)
                        self.at_cam_event.clear()

                # prepare frame for display
                display_frame = frame
                if results and results[0].boxes:
                    try:
                        display_frame = results[0].plot()
                    except Exception:
                        display_frame = frame
                if CV2_AVAILABLE and display_frame is not None:
                    display_frame = cv2.resize(display_frame, CAM_PREVIEW_SIZE)
                self.update_canvas_with_frame(display_frame)
                time.sleep(0.01)
            except Exception as e:
                self._log_message(f"Camera loop error: {e}")
                time.sleep(0.2)

    def update_canvas_with_frame(self, bgr_frame):
        try:
            if bgr_frame is None:
                return
            if CV2_AVAILABLE:
                rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            else:
                rgb = bgr_frame
            if PIL_AVAILABLE:
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(img)
                # Avoid leaving multiple images on canvas: clear and draw
                self.cam_canvas.delete("all")
                self.cam_canvas.imgtk = imgtk
                self.cam_canvas.create_image(0, 0, anchor='nw', image=imgtk)
            else:
                # fallback to cv2 window (not ideal for fullscreen)
                cv2.imshow("Camera Preview", bgr_frame)
                cv2.waitKey(1)
        except Exception as e:
            self._log_message(f"Display frame error: {e}")

    # ----------------- Stats & utilities -----------------
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
            self._log_message(f"Stats update error: {type(e).__name__}: {str(e)}")

    def _reset_stats(self):
        for cat in ["Raw", "Standard", "Overcooked"]:
            self.stats[cat] = 0
            self.moisture_sums[cat] = 0.0
        self.stats['total'] = 0
        self.stats['start_time'] = None
        self.stats['end_time'] = None
        self.update_stats()

    def _log_message(self, msg):
        # minimal duplicate suppression
        if ("Frame drop" in msg and self.frame_drop_logged) or ("Serial reader exception" in msg and self.serial_error_logged):
            return
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        try:
            self.log.config(state='normal')
            self.log.insert("end", full_msg + "\n")
            self.log.see("end")
            self.log.config(state='disabled')
        except Exception:
            print(full_msg)
        if "Frame drop" in msg:
            self.frame_drop_logged = True
        if "Serial reader exception" in msg:
            self.serial_error_logged = True

    def on_close(self):
        self.running = False
        self.process_running = False
        self.camera_running = False
        try:
            self.stop_process()
            self.stop_camera()
            self.close_serial()
            self._log_message("Application closed.")
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()