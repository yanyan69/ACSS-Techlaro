import tkinter as tk
import sqlite3
from datetime import datetime
import os
import serial
from serial.tools import list_ports
import time
import picamera2  # For RPi5 camera
import cv2  # For image processing
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO
import threading

DB_FILE = 'data/acss_stats.db'
BAUDRATE = 115200  # Matches Arduino

class ACSS_App:
    def __init__(self, root):
        self.root = root
        self.root.title('ACSS Control Panel')
        self.root.geometry('900x500')
        self.sorting_running = False
        self.sidebar_expanded = True
        self.processed_image_count = 0
        self.processed_sensor_count = 0
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.init_db()
        self.setup_arduino()  # Auto-detect and connect
        self.setup_ui()
        
        # YOLO and camera attributes
        self.model_path = 'my_model/train/weights/best.pt'  # Your model path
        self.model = None
        self.picam2 = None
        self.video_thread = None
        self.stop_event = threading.Event()
        self.setup_model_and_camera()  # Auto-check/setup

    def init_db(self):
        os.makedirs(os.path.dirname(DB_FILE), exist_ok=True)
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("""
            CREATE TABLE IF NOT EXISTS stats (
                id INTEGER PRIMARY KEY AUTOINCREMENT, 
                timestamp TEXT,
                processed_image_count INTEGER DEFAULT 0,
                processed_sensor_count INTEGER DEFAULT 0
            )
        """)
        conn.commit()
        conn.close()

    def setup_arduino(self):
        def find_arduino_port():
            ports = list_ports.comports()
            for port in ports:
                desc_lower = port.description.lower()
                if any(keyword in desc_lower for keyword in ['arduino', 'uno', 'mega', 'nano', 'ch340', 'ch341', 'usb serial', 'serial']):
                    print(f"Found Arduino on port: {port.device} ({port.description})")
                    return port.device
            print("No Arduino found. Available ports:")
            for port in ports:
                print(f"Port: {port.device}, Description: {port.description}")
            return None

        self.arduino = None
        try:
            port = find_arduino_port()
            if port is None:
                print("Arduino not found. Using /dev/ttyUSB0 as fallback.")
                port = '/dev/ttyUSB0'  # Or 'COM3' for Windows fallback
            self.arduino = serial.Serial(port=port, baudrate=BAUDRATE, timeout=1)
            time.sleep(2)
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")

    def setup_model_and_camera(self):
        # Auto-check model file
        if not os.path.exists(self.model_path):
            print(f"Error: YOLO model not found at {self.model_path}. Detection disabled.")
            return
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            self.model = None

        # Auto-check camera (will fail if not on RPi or disconnected)
        try:
            self.picam2 = picamera2.Picamera2()
            config = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
            self.picam2.configure(config)
            print("PiCamera detected and configured.")
        except Exception as e:
            print(f"Error initializing PiCamera: {e}. Using placeholder mode.")
            self.picam2 = None

    def setup_ui(self):
        self.sidebar = tk.Frame(self.root, bg='#1E1E1E', width=200)
        self.sidebar.grid(row=0, column=0, sticky='ns')
        self.sidebar.grid_propagate(False)
        
        toggle_btn = tk.Button(self.sidebar, text='☰', command=self.toggle_sidebar, bg='#1E1E1E', fg='white', bd=0)
        toggle_btn.pack(fill='x')

        self.buttons = [
            tk.Button(self.sidebar, text="Main Interface", command=self.show_main_interface),
            tk.Button(self.sidebar, text="Camera View", command=self.show_camera_view),
            tk.Button(self.sidebar, text="Statistics", command=self.show_statistics),
            tk.Button(self.sidebar, text="Component Status", command=self.show_component_status),
            tk.Button(self.sidebar, text="About", command=self.show_about),
            tk.Button(self.sidebar, text="Exit", command=self.shutdown_app)
        ]

        for btn in self.buttons:
            btn.pack(fill='x')

        self.main_frame = tk.Frame(self.root, bg='white')
        self.main_frame.grid(row=0, column=1, sticky='nsew')

        # Create widgets once
        self.toggle_button = tk.Button(
            self.main_frame,
            text='Start',
            command=self.toggle_sorting,
            width=10,
            height=2,
            bg='green',
            fg='white',
            font=('Arial', 14, 'bold')
        )
        self.count_label = tk.Label(self.main_frame, text="Objects detected: 0", font=("Arial", 14), bg="white")
        self.video_label = tk.Label(self.main_frame)
        self.status_label = tk.Label(self.main_frame, text="Status: Idle")  # For component status

        # Initially show main interface
        self.show_main_interface()

        # Start listening to Arduino if connected
        if self.arduino:
            self.root.after(100, self.listen_to_arduino)

    def toggle_sidebar(self):
        if self.sidebar_expanded:
            self.sidebar.config(width=50)
            for btn in self.buttons:
                btn.config(text="", width=2)
        else:
            self.sidebar.config(width=200)
            texts = ["Main Interface", "Camera View", "Statistics", "Component Status", "About", "Exit"]
            for btn, text in zip(self.buttons, texts):
                btn.config(text=text, width=20)
        self.sidebar_expanded = not self.sidebar_expanded
        
    def toggle_sorting(self):
        if not self.sorting_running:
            self.start_detection()
            self.toggle_button.config(text='Stop', bg='red')
            if self.arduino:
                self.send_command("START_SORTING")
        else:
            self.stop_detection()
            self.toggle_button.config(text='Start', bg='green')
            if self.arduino:
                self.send_command("STOP_SORTING")
        self.sorting_running = not self.sorting_running

    def start_detection(self):
        if self.model is None or self.picam2 is None:
            print("Detection disabled due to missing model or camera.")
            return
        self.stop_event.clear()
        try:
            self.picam2.start()
        except Exception as e:
            print(f"Error starting camera: {e}")
            return
        self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
        self.video_thread.start()

    def stop_detection(self):
        self.stop_event.set()
        if self.picam2:
            try:
                self.picam2.stop()
            except Exception as e:
                print(f"Error stopping camera: {e}")
        self.video_label.config(image='')  # Clear video feed
        self.count_label.config(text="Objects detected: 0")

    def video_loop(self):
        bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
                      (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

        while not self.stop_event.is_set():
            if self.picam2 is None:
                time.sleep(0.03)
                continue
            try:
                frame_bgra = self.picam2.capture_array()
                frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
            except Exception as e:
                print(f"Error capturing frame: {e}")
                continue

            results = self.model(frame, verbose=False)
            detections = results[0].boxes

            object_count = 0

            for i in range(len(detections)):
                xyxy_tensor = detections[i].xyxy.cpu()
                xyxy = xyxy_tensor.numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy.astype(int)
                classidx = int(detections[i].cls.item())
                conf = detections[i].conf.item()

                if conf > 0.5:
                    color = bbox_colors[classidx % 10]
                    cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)
                    label = f'{self.model.names[classidx]}: {int(conf*100)}%'
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    label_ymin = max(ymin, labelSize[1] + 10)
                    cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED)
                    cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    object_count += 1

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)

            def update_image():
                self.video_label.imgtk = imgtk
                self.video_label.config(image=imgtk)
                self.count_label.config(text=f"Objects detected: {object_count}")

            self.root.after(0, update_image)
            time.sleep(0.03)  # ~30 FPS

    def clear_main_frame(self):
        for widget in self.main_frame.winfo_children():
            widget.pack_forget()

    def show_main_interface(self):
        self.clear_main_frame()
        self.toggle_button.pack(pady=10)
        self.count_label.pack(pady=10)
        self.video_label.pack()

    def show_camera_view(self):
        self.clear_main_frame()
        self.toggle_button.pack_forget()
        self.count_label.pack_forget()
        self.video_label.pack(expand=True, fill='both')

    def show_statistics(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Statistics", font=("Arial", 16)).pack(pady=20)
        stats_label = tk.Label(self.main_frame, text=f"Processed Images: {self.processed_image_count}\nProcessed Sensors: {self.processed_sensor_count}")
        stats_label.pack(pady=10)

    def show_component_status(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Component Status", font=("Arial", 16)).pack(pady=20)

        # Test buttons (with try-except for robustness)
        tk.Button(self.main_frame, text="Test Motor ON", width=15, command=lambda: self.safe_send("MOTOR_ON")).pack(pady=5)
        tk.Button(self.main_frame, text="Test Motor OFF", width=15, command=lambda: self.safe_send("MOTOR_OFF")).pack(pady=5)
        tk.Button(self.main_frame, text="Test Servo RAW (0°)", width=15, command=lambda: self.safe_send("SERVO_TEST_0")).pack(pady=5)
        tk.Button(self.main_frame, text="Test Servo STD (60°)", width=15, command=lambda: self.safe_send("SERVO_TEST_60")).pack(pady=5)
        tk.Button(self.main_frame, text="Test Servo REJ (120°)", width=15, command=lambda: self.safe_send("SERVO_TEST_120")).pack(pady=5)
        tk.Button(self.main_frame, text="Read NIR Sensor", width=15, command=self.read_and_display_nir).pack(pady=5)
        tk.Button(self.main_frame, text="Check Proximity", width=15, command=self.check_proximity).pack(pady=5)

        self.status_label.pack(pady=10)

    def safe_send(self, cmd):
        try:
            self.send_command(cmd)
        except Exception as e:
            self.status_label.config(text=f"Error: {e}")

    def read_and_display_nir(self):
        try:
            self.send_command("READ_NIR")
            time.sleep(1)
            response = self.read_serial()
            self.status_label.config(text=f"NIR Data: {response}")
        except Exception as e:
            self.status_label.config(text=f"Error reading NIR: {e}")

    def check_proximity(self):
        try:
            self.send_command("PROX_STATUS")
            time.sleep(0.5)
            response = self.read_serial()
            self.status_label.config(text=f"Proximity: {response}")
        except Exception as e:
            self.status_label.config(text=f"Error checking proximity: {e}")

    def show_about(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="About ACSS", font=("Arial", 16)).pack(pady=20)
    
    def shutdown_app(self):
        self.stop_detection()
        if self.arduino:
            try:
                self.send_command("STOP_SORTING")
                self.arduino.close()
            except Exception as e:
                print(f"Error closing Arduino: {e}")
        self.root.destroy()

    def send_command(self, cmd):
        if not self.arduino:
            raise ValueError("No Arduino connected.")
        self.arduino.write((cmd + '\n').encode())

    def read_serial(self):
        if not self.arduino:
            return "No Arduino"
        try:
            return self.arduino.readline().decode('utf-8').strip()
        except:
            return "Read error"

    def listen_to_arduino(self):
        if self.sorting_running and self.arduino:
            response = self.read_serial()
            if response == "DETECTED":
                category = self.process_copra()
                self.send_command(f"SERVO_{category.upper()}")
                self.processed_image_count += 1
                self.processed_sensor_count += 1
                self.update_db()
        self.root.after(100, self.listen_to_arduino)

    def process_copra(self):
        if self.picam2 is None or self.model is None:
            return "REJ"  # Fallback
        try:
            frame_bgra = self.picam2.capture_array()
            frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
        except Exception as e:
            print(f"Error capturing for processing: {e}")
            return "REJ"

        results = self.model(frame, verbose=False)
        detections = results[0].boxes

        if len(detections) == 0:
            return "REJ"

        # Get highest conf detection
        max_conf_idx = np.argmax([d.conf.item() for d in detections])
        class_name = self.model.names[int(detections[max_conf_idx].cls.item())].upper()  # e.g., 'RAW', 'STANDARD', 'REJECTED'

        # Read NIR for validation
        self.send_command("READ_NIR")
        time.sleep(1)
        nir_data = self.read_serial()
        moisture_class = self.classify_moisture(nir_data)

        # Combine (RQ1 fusion: adjust if mismatch)
        if class_name == "RAW" and moisture_class != "HIGH":
            return "REJ"
        elif class_name == "STANDARD" and moisture_class != "MEDIUM":
            return "REJ"
        elif class_name == "REJECTED":
            return "REJ"
        return class_name[:3]  # 'RAW', 'STD', 'REJ' for servo

    def classify_moisture(self, nir_data):
        if "W=" not in nir_data:
            return "UNKNOWN"
        w_value = float(nir_data.split("W=")[1].split(",")[0])
        if w_value > 15:  # From calibration
            return "HIGH"
        elif 10 <= w_value <= 15:
            return "MEDIUM"
        else:
            return "LOW"

    def update_db(self):
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("""
            INSERT INTO stats (timestamp, processed_image_count, processed_sensor_count)
            VALUES (?, ?, ?)
        """, (datetime.now().isoformat(), self.processed_image_count, self.processed_sensor_count))
        conn.commit()
        conn.close()

if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown_app)
    root.mainloop()