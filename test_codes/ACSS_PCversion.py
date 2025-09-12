import tkinter as tk
from tkinter import scrolledtext
import sqlite3
from datetime import datetime
import os
import serial
from serial.tools import list_ports
import time
import cv2
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO
import threading

DB_FILE = 'data/acss_stats.db'
BAUDRATE = 115200
IMAGE_PATH = 'resources/images/'

class ACSS_App:
    def __init__(self, root):
        self.root = root
        self.root.title('Automated Copra Segregation System (PC Version)')
        self.root.geometry('1024x600')
        self.root.attributes('-fullscreen', True)
        self.root.bind('<Key-f>', lambda e: self.toggle_fullscreen())
        self.root.bind('<Key-space>', lambda e: self.toggle_sorting())
        self.root.bind('<Key-Escape>', lambda e: self.show_component_status())
        self.root.bind('<Key-a>', lambda e: self.safe_send("SERVO_TEST_0"))
        self.root.bind('<Key-w>', lambda e: self.safe_send("SERVO_TEST_90"))
        self.root.bind('<Key-d>', lambda e: self.safe_send("SERVO_TEST_180"))
        self.root.bind('<Key-s>', lambda e: self.toggle_motor())
        self.root.bind('<Key-c>', lambda e: self.toggle_camera_view())
        self.sorting_running = False
        self.sidebar_expanded = True
        self.camera_visible = True
        self.motor_running = False
        self.log_visible = False
        self.processed_image_count = 0
        self.processed_sensor_count = 0
        self.raw_units = 0
        self.standard_units = 0
        self.overcooked_units = 0
        self.raw_moistures = []
        self.standard_moistures = []
        self.overcooked_moistures = []
        self.start_time = None
        self.current_user = "default_user"
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.init_db()
        self.setup_arduino()
        self.setup_model_and_camera()
        self.setup_ui()
        
        if self.arduino:
            self.root.after(100, self.listen_to_arduino)

    def log_action(self, action):
        try:
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            c.execute("INSERT INTO activity_log (timestamp, username, action) VALUES (?, ?, ?)",
                      (datetime.now().isoformat(), self.current_user, action))
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"Error logging action: {e}")

    def toggle_fullscreen(self):
        self.root.attributes('-fullscreen', not self.root.attributes('-fullscreen'))
        self.log_action("Toggled fullscreen")

    def toggle_motor(self):
        self.motor_running = not self.motor_running
        cmd = "MOTOR_ON" if self.motor_running else "MOTOR_OFF"
        self.safe_send(cmd)
        self.log_action(f"Toggled motor {'ON' if self.motor_running else 'OFF'} (s)")

    def toggle_camera_view(self):
        self.camera_visible = not self.camera_visible
        if self.camera_visible:
            self.video_label.pack(expand=True, fill='both')
        else:
            self.video_label.pack_forget()
        self.log_action("Toggled camera view (c)")

    def toggle_log(self):
        self.log_visible = not self.log_visible
        if self.log_visible:
            self.log_text.pack(fill='both', expand=True, padx=20, pady=10)
            self.update_log_display()
        else:
            self.log_text.pack_forget()
        self.log_action("Toggled log display")

    def init_db(self):
        os.makedirs(os.path.dirname(DB_FILE), exist_ok=True)
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("""
            CREATE TABLE IF NOT EXISTS stats (
                id INTEGER PRIMARY KEY AUTOINCREMENT, 
                timestamp TEXT,
                username TEXT,
                processed_image_count INTEGER DEFAULT 0,
                processed_sensor_count INTEGER DEFAULT 0,
                raw_units INTEGER DEFAULT 0,
                standard_units INTEGER DEFAULT 0,
                overcooked_units INTEGER DEFAULT 0,
                raw_avg_moisture REAL DEFAULT 0.0,
                standard_avg_moisture REAL DEFAULT 0.0,
                overcooked_avg_moisture REAL DEFAULT 0.0,
                total_time TEXT,
                start_time TEXT
            )
        """)
        c.execute("""
            CREATE TABLE IF NOT EXISTS activity_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                username TEXT,
                action TEXT
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
        retries = 5
        for i in range(retries):
            try:
                port = find_arduino_port()
                if port is None:
                    port = 'COM3'  # Windows fallback
                self.arduino = serial.Serial(port=port, baudrate=BAUDRATE, timeout=1)
                time.sleep(2)
                print(f"Arduino connected on {port}")
                self.log_action(f"Connected to Arduino on {port}")
                return
            except Exception as e:
                print(f"Arduino connection attempt {i+1}/{retries} failed: {e}")
                time.sleep(2)
        print("Failed to connect to Arduino after retries.")
        self.log_action("Failed to connect to Arduino")

    def setup_model_and_camera(self):
        self.model_path = 'my_model/train/weights/best.pt'
        if not os.path.exists(self.model_path):
            print(f"Error: YOLO model not found at {self.model_path}. Detection disabled.")
            self.model = None
            self.log_action(f"YOLO model not found at {self.model_path}")
        else:
            try:
                self.model = YOLO(self.model_path)
                print("YOLO model loaded.")
                self.log_action("Loaded YOLO model")
            except Exception as e:
                print(f"Error loading YOLO model: {e}")
                self.model = None
                self.log_action(f"Error loading YOLO model: {e}")

        try:
            self.cap = cv2.VideoCapture(0)  # Default webcam
            if not self.cap.isOpened():
                raise Exception("Webcam not detected")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print("Webcam detected and configured.")
            self.log_action("Webcam configured")
        except Exception as e:
            print(f"Error initializing webcam: {e}. Using placeholder mode.")
            self.cap = None
            self.log_action(f"Error initializing webcam: {e}")

    def setup_ui(self):
        self.sidebar = tk.Frame(self.root, bg='#1E1E1E', width=250)
        self.sidebar.grid(row=0, column=0, sticky='ns')
        self.sidebar.grid_propagate(False)
        
        toggle_btn = tk.Button(self.sidebar, text='☰', command=self.toggle_sidebar, bg='#1E1E1E', fg='white', bd=0, font=('Arial', 16))
        toggle_btn.grid(row=0, column=0, columnspan=2, sticky='ew', pady=10)

        icon_files = ['main_interface_icon.png', 'statistics_icon.png', 'component_status_icon.png', 'about_icon.png', 'exit_icon.png']
        self.icons = []
        for f in icon_files:
            try:
                img = tk.PhotoImage(file=os.path.join(IMAGE_PATH, f))
                self.icons.append(img)
            except Exception as e:
                print(f"Error loading icon {f}: {e}")
                self.icons.append(None)
                self.log_action(f"Error loading icon {f}")
        self.icon_buttons = []
        self.icon_labels = []
        commands = [self.show_main_interface, self.show_statistics, self.show_component_status, self.show_about, self.shutdown_app]
        texts = ["Main Interface", "Statistics", "Component Testing", "About", "Exit"]

        for i, (icon, cmd, text) in enumerate(zip(self.icons, commands, texts)):
            btn = tk.Button(self.sidebar, image=icon, command=cmd, bg='#1E1E1E', bd=0)
            btn.grid(row=i+1, column=0, sticky='ew', padx=5, pady=10)
            self.icon_buttons.append(btn)
            label = tk.Label(self.sidebar, text=text, bg='#1E1E1E', fg='white', font=('Arial', 14))
            label.grid(row=i+1, column=1, sticky='w')
            self.icon_labels.append(label)

        self.main_frame = tk.Frame(self.root, bg='white')
        self.main_frame.grid(row=0, column=1, sticky='nsew')

        self.toggle_button = tk.Button(
            self.main_frame,
            text='Start',
            command=self.toggle_sorting,
            width=15,
            height=3,
            bg='green',
            fg='white',
            font=('Arial', 16, 'bold')
        )
        self.count_label = tk.Label(self.main_frame, text="Objects detected: 0", font=("Arial", 16), bg="white")
        self.video_label = tk.Label(self.main_frame)
        self.status_label = tk.Label(self.main_frame, text="Status: Idle", font=("Arial", 14), bg="white")
        self.log_text = scrolledtext.ScrolledText(self.main_frame, height=10, font=("Arial", 12), state='disabled')

        self.show_main_interface()

    def toggle_sidebar(self):
        self.sidebar_expanded = not self.sidebar_expanded
        if self.sidebar_expanded:
            self.sidebar.config(width=250)
            for label in self.icon_labels:
                label.grid()
        else:
            self.sidebar.config(width=64)
            for label in self.icon_labels:
                label.grid_remove()
        self.log_action("Toggled sidebar")

    def toggle_sorting(self):
        if not self.sorting_running:
            self.start_detection()
            self.toggle_button.config(text='Stop', bg='red')
            if self.arduino:
                self.safe_send("START_SORTING")
            self.start_time = datetime.now()
            self.log_action("Started sorting")
        else:
            self.stop_detection()
            self.toggle_button.config(text='Start', bg='green')
            if self.arduino:
                self.safe_send("STOP_SORTING")
            self.start_time = None
            self.log_action("Stopped sorting")
        self.sorting_running = not self.sorting_running

    def start_detection(self):
        if self.model is None or self.cap is None:
            self.status_label.config(text="Error: Missing model or camera")
            self.log_action("Failed to start detection: missing model or camera")
            return
        self.stop_event = threading.Event()
        self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
        self.video_thread.start()

    def stop_detection(self):
        if hasattr(self, 'stop_event'):
            self.stop_event.set()
        if self.cap:
            try:
                self.cap.release()
                self.log_action("Webcam released")
            except Exception as e:
                print(f"Error releasing webcam: {e}")
                self.log_action(f"Error releasing webcam: {e}")
        self.video_label.config(image='')
        self.count_label.config(text="Objects detected: 0")

    def video_loop(self):
        bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
                      (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

        while not self.stop_event.is_set():
            if self.cap is None:
                time.sleep(0.03)
                continue
            try:
                ret, frame = self.cap.read()
                if not ret:
                    raise Exception("Failed to capture frame")
            except Exception as e:
                print(f"Error capturing frame: {e}")
                self.log_action(f"Error capturing frame: {e}")
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
            time.sleep(0.03)

    def clear_main_frame(self):
        for widget in self.main_frame.winfo_children():
            widget.pack_forget()

    def show_main_interface(self):
        self.clear_main_frame()
        self.toggle_button.pack(pady=20, anchor='center')
        self.count_label.pack(pady=10, anchor='center')
        if self.camera_visible:
            self.video_label.pack(expand=True, fill='both')
        self.log_action("Opened Main Interface")

    def show_statistics(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Statistics", font=("Arial", 24, 'bold')).pack(pady=20, anchor='center')
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("SELECT * FROM stats ORDER BY username, timestamp")
        rows = c.fetchall()
        conn.close()

        if not rows:
            tk.Label(self.main_frame, text="No stats available.", font=("Arial", 16)).pack(pady=10, anchor='center')
            self.log_action("Viewed Statistics (empty)")
            return

        prev_user = None
        for row in rows:
            user = row[2]
            if user != prev_user:
                tk.Label(self.main_frame, text=f"User: {user} (Last Modified: {row[1]})", font=("Arial", 18, 'bold')).pack(pady=10, anchor='w', padx=20)
                prev_user = user

            tk.Label(self.main_frame, text=f"Raw: {row[5]} units, Avg Moisture: {row[8]:.2f}%", font=("Arial", 14)).pack(anchor='w', padx=20)
            tk.Label(self.main_frame, text=f"Standard: {row[6]} units, Avg Moisture: {row[9]:.2f}%", font=("Arial", 14)).pack(anchor='w', padx=20)
            tk.Label(self.main_frame, text=f"Overcooked: {row[7]} units, Avg Moisture: {row[10]:.2f}%", font=("Arial", 14)).pack(anchor='w', padx=20)
            overall_units = row[5] + row[6] + row[7]
            overall_moisture = (row[8] * row[5] + row[9] * row[6] + row[10] * row[7]) / overall_units if overall_units > 0 else 0
            tk.Label(self.main_frame, text=f"Overall: {overall_units} units, Avg Moisture: {overall_moisture:.2f}%", font=("Arial", 14)).pack(anchor='w', padx=20)
            tk.Label(self.main_frame, text=f"Process Time: {row[11]}, Started: {row[12]}", font=("Arial", 14)).pack(anchor='w', padx=20)
            tk.Frame(self.main_frame, height=2, bg="black").pack(fill='x', pady=10)

        self.log_action("Viewed Statistics")

    def show_component_status(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Component Testing", font=("Arial", 24, 'bold')).pack(pady=20, anchor='center')

        tk.Label(self.main_frame, text="Servo Motor", font=("Arial", 16)).pack(pady=5, anchor='center')
        frame = tk.Frame(self.main_frame, bg='white')
        frame.pack(fill='x')
        tk.Button(frame, text="Left (a)", command=lambda: self.safe_send("SERVO_TEST_0"), width=12, height=2, font=("Arial", 14)).pack(side='left', padx=10)
        tk.Button(frame, text="Neutral (w)", command=lambda: self.safe_send("SERVO_TEST_90"), width=12, height=2, font=("Arial", 14)).pack(side='left', padx=10)
        tk.Button(frame, text="Right (d)", command=lambda: self.safe_send("SERVO_TEST_180"), width=12, height=2, font=("Arial", 14)).pack(side='left', padx=10)
        tk.Frame(self.main_frame, height=2, bg="black").pack(fill='x', pady=10)

        tk.Label(self.main_frame, text="DC Motor", font=("Arial", 16)).pack(pady=5, anchor='center')
        tk.Button(self.main_frame, text="Start/Stop (s)", command=self.toggle_motor, width=15, height=2, font=("Arial", 14)).pack(anchor='center')
        tk.Frame(self.main_frame, height=2, bg="black").pack(fill='x', pady=10)

        tk.Label(self.main_frame, text="NIR Sensor", font=("Arial", 16)).pack(pady=5, anchor='center')
        tk.Button(self.main_frame, text="Read NIR", command=self.read_and_display_nir, width=15, height=2, font=("Arial", 14)).pack(anchor='center')
        tk.Frame(self.main_frame, height=2, bg="black").pack(fill='x', pady=10)

        tk.Label(self.main_frame, text="Proximity Sensor", font=("Arial", 16)).pack(pady=5, anchor='center')
        tk.Button(self.main_frame, text="Check Proximity", command=self.check_proximity, width=15, height=2, font=("Arial", 14)).pack(anchor='center')
        tk.Frame(self.main_frame, height=2, bg="black").pack(fill='x', pady=10)

        tk.Label(self.main_frame, text="Camera View", font=("Arial", 16)).pack(pady=5, anchor='center')
        tk.Button(self.main_frame, text="Toggle Camera (c)", command=self.toggle_camera_view, width=15, height=2, font=("Arial", 14)).pack(anchor='center')
        tk.Frame(self.main_frame, height=2, bg="black").pack(fill='x', pady=10)

        frame = tk.Frame(self.main_frame, bg='white')
        frame.pack(fill='x')
        tk.Button(frame, text="Verify All", command=self.verify_all_components, width=15, height=2, font=("Arial", 14)).pack(side='left', padx=10, anchor='center')
        tk.Button(frame, text="Show/Hide Log", command=self.toggle_log, width=15, height=2, font=("Arial", 14)).pack(side='left', padx=10, anchor='center')

        self.status_label.pack(pady=20, anchor='center')
        self.log_action("Opened Component Testing")

    def show_about(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Automated Copra Segregation System", font=("Arial", 24, 'bold')).pack(pady=20, anchor='center')
        tk.Label(self.main_frame, text="Made by Techlaro Company", font=("Arial", 16)).pack(pady=10, anchor='center')

        members = [
            ("Member 1", "Lead Engineer", "member1.png"),
            ("Member 2", "Software Developer", "member2.png"),
            ("Member 3", "Hardware Specialist", "member3.png"),
            ("Member 4", "Data Scientist", "member4.png")
        ]
        self.member_photos = []
        for name, pos, img_file in members:
            frame = tk.Frame(self.main_frame, bg='white')
            frame.pack(fill='x', pady=5)
            try:
                photo = tk.PhotoImage(file=os.path.join(IMAGE_PATH, img_file))
                self.member_photos.append(photo)
                tk.Label(frame, image=photo).pack(side='left', padx=10)
            except Exception as e:
                print(f"Error loading member photo {img_file}: {e}")
                tk.Label(frame, text="[Photo]").pack(side='left', padx=10)
                self.log_action(f"Error loading member photo {img_file}")
            tk.Label(frame, text=f"{name} - {pos}", font=("Arial", 14)).pack(side='left', padx=10)

        tk.Label(self.main_frame, text="Description: The ACSS automates copra sorting using a YOLO model for visual classification and an AS7263 NIR sensor for moisture detection, improving efficiency for small-scale buyers.", font=("Arial", 14), wraplength=800).pack(pady=10, padx=20)
        tk.Label(self.main_frame, text="Goals: Enhance sorting accuracy, reduce manual errors, and provide a cost-effective automation solution for copra traders in Marinduque.", font=("Arial", 14), wraplength=800).pack(pady=10, padx=20)
        tk.Label(self.main_frame, text="Info: Built with Raspberry Pi 5, Arduino Uno, AS7263 NIR sensor, servo, DC motor, and proximity sensor. Supports real-time sorting with high accuracy.", font=("Arial", 14), wraplength=800).pack(pady=10, padx=20)

        footer = tk.Label(self.main_frame, text="© 2025 Techlaro Company | Version 1.0 | Contact: info@techlaro.com", font=("Arial", 12), bg="gray", fg="white")
        footer.pack(side='bottom', fill='x', pady=10)
        self.log_action("Opened About")

    def verify_all_components(self):
        status = []
        if not self.arduino:
            status.append("Arduino: Not connected")
        else:
            self.safe_send("SERVO_TEST_90")
            time.sleep(1)
            status.append("Servo: Moved to neutral")
            self.safe_send("MOTOR_ON")
            time.sleep(0.5)
            self.safe_send("MOTOR_OFF")
            status.append("Motor: Toggled on/off")
            self.send_command("READ_NIR")
            time.sleep(1)
            nir_response = self.read_serial()
            status.append(f"NIR: {nir_response if nir_response else 'No response'}")
            self.send_command("PROX_STATUS")
            time.sleep(0.5)
            prox_response = self.read_serial()
            status.append(f"Proximity: {prox_response if prox_response else 'No response'}")
        if self.cap:
            status.append("Camera: Detected")
        else:
            status.append("Camera: Not detected")
        self.status_label.config(text="Verify Results:\n" + "\n".join(status))
        self.log_action("Verified all components")

    def update_log_display(self):
        try:
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            c.execute("SELECT timestamp, username, action FROM activity_log ORDER BY timestamp DESC")
            rows = c.fetchall()
            conn.close()
            self.log_text.config(state='normal')
            self.log_text.delete(1.0, tk.END)
            for row in rows:
                self.log_text.insert(tk.END, f"{row[0]} | {row[1]} | {row[2]}\n")
            self.log_text.config(state='disabled')
        except Exception as e:
            self.status_label.config(text=f"Error loading log: {e}")
            self.log_action(f"Error loading log: {e}")

    def safe_send(self, cmd):
        try:
            self.send_command(cmd)
            self.log_action(f"Sent command: {cmd}")
        except Exception as e:
            self.status_label.config(text=f"Error: {e}")
            self.log_action(f"Error sending {cmd}: {e}")

    def read_and_display_nir(self):
        try:
            self.send_command("READ_NIR")
            time.sleep(1)
            response = self.read_serial()
            self.status_label.config(text=f"NIR Data: {response}")
            self.log_action("Read NIR sensor")
        except Exception as e:
            self.status_label.config(text=f"Error reading NIR: {e}")
            self.log_action(f"Error reading NIR: {e}")

    def check_proximity(self):
        try:
            self.send_command("PROX_STATUS")
            time.sleep(0.5)
            response = self.read_serial()
            self.status_label.config(text=f"Proximity: {response}")
            self.log_action("Checked proximity sensor")
        except Exception as e:
            self.status_label.config(text=f"Error checking proximity: {e}")
            self.log_action(f"Error checking proximity: {e}")

    def shutdown_app(self):
        if hasattr(self, 'stop_event'):
            self.stop_detection()
        if self.arduino:
            try:
                self.send_command("STOP_SORTING")
                self.arduino.close()
                self.log_action("Closed Arduino connection")
            except Exception as e:
                print(f"Error closing Arduino: {e}")
                self.log_action(f"Error closing Arduino: {e}")
        self.root.destroy()
        self.log_action("Shutdown application")

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
                self.log_action(f"Processed copra: {category}")
        self.root.after(100, self.listen_to_arduino)

    def process_copra(self):
        if self.cap is None or self.model is None:
            self.log_action("Failed to process copra: missing model or camera")
            return "REJ"
        try:
            ret, frame = self.cap.read()
            if not ret:
                raise Exception("Failed to capture frame")
        except Exception as e:
            print(f"Error capturing for processing: {e}")
            self.log_action(f"Error capturing for processing: {e}")
            return "REJ"

        results = self.model(frame, verbose=False)
        detections = results[0].boxes

        if len(detections) == 0:
            self.log_action("No copra detected")
            return "REJ"

        max_conf_idx = np.argmax([d.conf.item() for d in detections])
        class_name = self.model.names[int(detections[max_conf_idx].cls.item())].upper()

        self.send_command("READ_NIR")
        time.sleep(1)
        nir_data = self.read_serial()
        moisture_class, w_value = self.classify_moisture(nir_data)

        if class_name == "RAW" and moisture_class != "HIGH":
            category = "REJ"
        elif class_name == "STANDARD" and moisture_class != "MEDIUM":
            category = "REJ"
        elif class_name == "REJECTED" or class_name == "OVERCOOKED":
            category = "REJ"
        else:
            category = class_name[:3]

        if category == "RAW":
            self.raw_units += 1
            self.raw_moistures.append(w_value)
        elif category == "STD":
            self.standard_units += 1
            self.standard_moistures.append(w_value)
        elif category == "REJ":
            self.overcooked_units += 1
            self.overcooked_moistures.append(w_value)

        return category

    def classify_moisture(self, nir_data):
        if "W=" not in nir_data:
            self.log_action("No NIR data received")
            return "UNKNOWN", 0
        try:
            w_value = float(nir_data.split("W=")[1].split(",")[0])
            if w_value > 15:
                return "HIGH", w_value
            elif 10 <= w_value <= 15:
                return "MEDIUM", w_value
            else:
                return "LOW", w_value
        except:
            self.log_action("Error parsing NIR data")
            return "UNKNOWN", 0

    def update_db(self):
        try:
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            raw_avg = sum(self.raw_moistures) / len(self.raw_moistures) if self.raw_moistures else 0
            std_avg = sum(self.standard_moistures) / len(self.standard_moistures) if self.standard_moistures else 0
            over_avg = sum(self.overcooked_moistures) / len(self.overcooked_moistures) if self.overcooked_moistures else 0
            total_time = str(datetime.now() - self.start_time) if self.start_time else "0:00:00"
            start_time = self.start_time.isoformat() if self.start_time else datetime.now().isoformat()
            c.execute("""
                INSERT INTO stats (
                    timestamp, username, processed_image_count, processed_sensor_count,
                    raw_units, standard_units, overcooked_units,
                    raw_avg_moisture, standard_avg_moisture, overcooked_avg_moisture,
                    total_time, start_time
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                datetime.now().isoformat(),
                self.current_user,
                self.processed_image_count,
                self.processed_sensor_count,
                self.raw_units,
                self.standard_units,
                self.overcooked_units,
                raw_avg,
                std_avg,
                over_avg,
                total_time,
                start_time
            ))
            conn.commit()
            conn.close()
            self.log_action("Updated stats database")
        except Exception as e:
            print(f"Error updating DB: {e}")
            self.log_action(f"Error updating DB: {e}")

if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown_app)
    root.mainloop()