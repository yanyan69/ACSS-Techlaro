import sqlite3, datetime
import os, serial, time
from serial.tools import list_ports
import cv2, numpy as np
from ultralytics import YOLO
from constants import DB_FILE, IMAGE_PATH
from tkinter import messagebox

class HardwareInterface:
    def __init__(self, app):
        self.app = app
        self.sorting_running = False
        self.motor_running = False
        self.processed_image_count = 0
        self.processed_sensor_count = 0
        self.raw_units = 0
        self.standard_units = 0
        self.overcooked_units = 0
        self.raw_moistures = []
        self.standard_moistures = []
        self.overcooked_moistures = []
        self.start_time = None
        self.arduino = None
        self.arduino_retry_count = 0
        try:
            self.init_db()
            self.setup_model_and_camera()
            self.start_arduino_detection()
        except Exception as e:
            print(f"HardwareInterface init error: {str(e)}")
            self.log_action(f"Hardware init failed: {str(e)}")
            raise

    def init_db(self):
        """Initialize the SQLite database for stats and activity logs."""
        try:
            print("Initializing database")
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
            print("Database initialized")
        except Exception as e:
            print(f"Database init error: {str(e)}")
            self.log_action(f"Database init error: {str(e)}")
            raise

    def log_action(self, action):
        """Log an action to the database."""
        try:
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            c.execute("INSERT INTO activity_log (timestamp, username, action) VALUES (?, ?, ?)",
                      (datetime.datetime.now().isoformat(), self.app.current_user, action))
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"Error logging action: {e}")

    def find_arduino_port(self):
        """Find an Arduino port, returning None if no Arduino is connected."""
        try:
            ports = list_ports.comports()
            for port in ports:
                desc_lower = port.description.lower()
                if any(keyword in desc_lower for keyword in ['arduino', 'uno', 'mega', 'nano', 'ch340', 'ch341']):
                    print(f"Found Arduino on port: {port.device} ({port.description})")
                    return port.device
            print("No Arduino found. Available ports:")
            for port in ports:
                print(f"Port: {port.device}, Description: {port.description}")
            return None
        except Exception as e:
            print(f"Error finding Arduino port: {str(e)}")
            self.log_action(f"Error finding Arduino port: {str(e)}")
            return None

    def setup_arduino(self):
        """Attempt to connect to an Arduino."""
        if self.arduino:
            return
        port = self.find_arduino_port()
        if port:
            try:
                self.arduino = serial.Serial(port=port, baudrate=115200, timeout=2)
                time.sleep(2)
                print(f"Arduino connected on {port}")
                self.log_action(f"Connected to Arduino on {port}")
                if hasattr(self.app, 'status_label'):
                    self.app.status_label.config(text=f"Status: Arduino connected on {port}")
                self.app.root.after(100, self.app.listen_to_arduino)
                self.arduino_retry_count = 0
            except Exception as e:
                print(f"Failed to connect to Arduino on {port}: {e}")
                self.arduino = None
                self.log_action(f"Failed to connect to Arduino on {port}: {e}")
                if hasattr(self.app, 'status_label'):
                    self.app.status_label.config(text=f"Status: Failed to connect to Arduino: {e}")
        else:
            print("No Arduino port detected, skipping connection")
            self.log_action("No Arduino port detected, skipping connection")

    def start_arduino_detection(self):
        """Skip Arduino detection since no Arduino is connected."""
        try:
            print("Skipping Arduino detection (no Arduino connected)")
            self.log_action("Skipping Arduino detection")
            self.arduino = None
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text="Status: No Arduino connected")
        except Exception as e:
            print(f"Arduino detection error: {str(e)}")
            self.log_action(f"Arduino detection error: {str(e)}")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"Status: No Arduino connected")

    def setup_model_and_camera(self):
        """Set up the YOLO model and camera."""
        try:
            print("Setting up YOLO model")
            self.model_path = os.path.join(IMAGE_PATH, '..', '..', 'my_model', 'train', 'weights', 'best.pt')
            print(f"Resolved YOLO model path: {self.model_path}")
            if not os.path.exists(self.model_path):
                print(f"Warning: YOLO model not found at {self.model_path}. Detection disabled.")
                self.model = None
                self.log_action(f"YOLO model not found at {self.model_path}")
            else:
                self.model = YOLO(self.model_path)
                print("YOLO model loaded.")
                self.log_action("Loaded YOLO model")

            print("Setting up webcam")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise Exception("Webcam not detected")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print("Webcam detected and configured.")
            self.log_action("Webcam configured")
        except Exception as e:
            print(f"Model/camera setup error: {str(e)}")
            self.model = None
            self.cap = None
            self.log_action(f"Model/camera setup error: {str(e)}")
            raise

    def start_detection(self):
        """Start the sorting detection process."""
        if not self.arduino:
            self.log_action("Cannot start detection: Arduino not connected")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text="Status: Cannot start, Arduino not connected")
            return
        self.sorting_running = True
        self.start_time = datetime.datetime.now()
        self.log_action("Started detection")

    def stop_detection(self):
        """Stop the sorting detection process."""
        self.sorting_running = False
        self.update_db()
        self.log_action("Stopped detection")
        if self.arduino:
            self.send_command("STOP_SORTING")

    def toggle_motor(self):
        """Toggle the motor status."""
        if not self.arduino:
            self.log_action("Cannot toggle motor: Arduino not connected")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text="Status: Cannot toggle motor, Arduino not connected")
            return
        self.motor_running = not self.motor_running
        cmd = "MOTOR_ON" if self.motor_running else "MOTOR_OFF"
        self.safe_send(cmd)
        self.log_action(f"Toggled motor {'ON' if self.motor_running else 'OFF'}")

    def send_command(self, cmd):
        """Send a command to the Arduino."""
        if not self.arduino:
            raise ValueError("No Arduino connected.")
        self.arduino.write((cmd + '\n').encode())

    def read_serial(self):
        """Read from the Arduino serial port."""
        if not self.arduino:
            return "No Arduino"
        try:
            return self.arduino.readline().decode('utf-8').strip()
        except:
            return "Read error"

    def safe_send(self, cmd):
        """Safely send a command to the Arduino with error handling."""
        try:
            self.send_command(cmd)
            self.log_action(f"Sent command: {cmd}")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"Command sent: {cmd}")
        except Exception as e:
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"Error: {e}")
            self.log_action(f"Error sending {cmd}: {e}")

    def read_and_display_nir(self):
        """Read and display NIR sensor data."""
        if not self.arduino:
            self.log_action("Cannot read NIR: Arduino not connected")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text="Status: Cannot read NIR, Arduino not connected")
            return
        try:
            self.send_command("READ_NIR")
            time.sleep(1)
            response = self.read_serial()
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"NIR Data: {response}")
            self.log_action("Read NIR sensor")
        except Exception as e:
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"Error reading NIR: {e}")
            self.log_action(f"Error reading NIR: {e}")

    def check_proximity(self):
        """Check proximity sensor status."""
        if not self.arduino:
            self.log_action("Cannot check proximity: Arduino not connected")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text="Status: Cannot check proximity, Arduino not connected")
            return
        try:
            self.send_command("PROX_STATUS")
            time.sleep(0.5)
            response = self.read_serial()
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"Proximity: {response}")
            self.log_action("Checked proximity sensor")
        except Exception as e:
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text=f"Error checking proximity: {e}")
            self.log_action(f"Error checking proximity: {e}")

    def verify_all_components(self):
        """Verify all components and update status."""
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
        if hasattr(self.app, 'status_label'):
            self.app.status_label.config(text="Verify Results:\n" + "\n".join(status))
        self.log_action("Verified all components")

    def process_copra(self):
        """Process a copra image and return the category."""
        if not self.arduino:
            self.log_action("Cannot process copra: Arduino not connected")
            if hasattr(self.app, 'status_label'):
                self.app.status_label.config(text="Status: Cannot process, Arduino not connected")
            return "REJ"
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

        self.processed_image_count += 1
        self.processed_sensor_count += 1
        self.update_db()
        return category

    def classify_moisture(self, nir_data):
        """Classify moisture level from NIR data."""
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

    def calculate_stats(self):
        """Calculate statistics for display and export."""
        data = []
        for category, units, moistures in [
            ('Raw', self.raw_units, self.raw_moistures),
            ('Standard', self.standard_units, self.standard_moistures),
            ('Overcooked', self.overcooked_units, self.overcooked_moistures)
        ]:
            avg_moisture = sum(moistures) / len(moistures) if moistures else 0
            data.append([category, units, round(avg_moisture, 1)])
        overall_units = sum(row[1] for row in data)
        overall_moisture = sum(row[1] * row[2] for row in data) / overall_units if overall_units else 0
        data.append(['Overall', overall_units, round(overall_moisture, 1)])
        return data

    def update_db(self):
        """Update the stats database."""
        try:
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            raw_avg = sum(self.raw_moistures) / len(self.raw_moistures) if self.raw_moistures else 0
            std_avg = sum(self.standard_moistures) / len(self.standard_moistures) if self.standard_moistures else 0
            over_avg = sum(self.overcooked_moistures) / len(self.overcooked_moistures) if self.overcooked_moistures else 0
            total_time = str(datetime.datetime.now() - self.start_time) if self.start_time else "0:00:00"
            start_time = self.start_time.isoformat() if self.start_time else datetime.datetime.now().isoformat()
            c.execute("""
                INSERT INTO stats (
                    timestamp, username, processed_image_count, processed_sensor_count,
                    raw_units, standard_units, overcooked_units,
                    raw_avg_moisture, standard_avg_moisture, overcooked_avg_moisture,
                    total_time, start_time
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                datetime.datetime.now().isoformat(),
                self.app.current_user,
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

    def shutdown(self):
        """Shutdown hardware connections."""
        if self.sorting_running:
            self.stop_detection()
        if self.arduino:
            try:
                self.send_command("STOP_SORTING")
                self.arduino.close()
                self.log_action("Closed Arduino connection")
            except Exception as e:
                print(f"Error closing Arduino: {e}")
                self.log_action(f"Error closing Arduino: {e}")
        if self.cap:
            self.cap.release()
            self.log_action("Released camera")