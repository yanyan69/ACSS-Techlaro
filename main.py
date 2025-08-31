import tkinter as tk
import sqlite3
from datetime import datetime
import os
import serial
import serial.tools.list_ports
import time
import tkinter.messagebox
from picamera2 import Picamera2, Preview
from PIL import Image, ImageTk

DB_FILE = 'data/acss_stats.db'

# 🔹 Setup Arduino Serial Connection
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if any(keyword in port.description.lower() for keyword in ['arduino', 'uno', 'ch340', 'ch341', 'usb serial', 'serial']):
            print(f"Found Arduino on port: {port.device} ({port.description})")
            return port.device
    print("No Arduino found. Available ports:")
    for port in ports:
        print(f"Port: {port.device}, Description: {port.description}")
    return None

try:
    port = find_arduino_port()
    if port is None:
        print("Arduino not found. Using /dev/ttyUSB0 as fallback.")
        port = '/dev/ttyUSB0'
    arduino = serial.Serial(port=port, baudrate=9600, timeout=1)
    time.sleep(2)
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    arduino = None

class ACSS_App:
    def __init__(self, root):
        self.root = root
        self.root.title('ACSS Control Panel')
        self.root.geometry('900x500')
        self.sorting_running = False
        self.sidebar_expanded = True
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.picam2 = None
        self.init_db()
        self.setup_ui()

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

    def setup_ui(self):
        self.sidebar_expanded = True
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

        self.status_label = tk.Label(self.main_frame, text="Arduino: Not Connected" if arduino is None else "Arduino: Connected", fg="red" if arduino is None else "green")
        self.status_label.pack(pady=10)

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
        self.toggle_button.pack(pady=50)

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
        self.sorting_running = not self.sorting_running
        if self.sorting_running:
            self.toggle_button.config(text='Stop', bg='red')
        else:
            self.toggle_button.config(text='Start', bg='green')

    def show_main_interface(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Main Interface", font=("Arial", 16)).pack(pady=20)
        self.stop_camera()

    def show_camera_view(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Camera View", font=("Arial", 16)).pack(pady=20)
        try:
            self.picam2 = Picamera2()
            camera_config = self.picam2.create_still_configuration(main={"size": (640, 480)})
            self.picam2.configure(camera_config)
            self.picam2.start_preview(Preview.NULL)  # Use Preview.QTGL for monitor
            self.picam2.start()
            tk.Label(self.main_frame, text="Camera started. Click 'Capture' to save an image.", font=("Arial", 12)).pack(pady=10)
            tk.Button(self.main_frame, text="Capture", command=self.capture_image, width=15).pack(pady=5)
            self.camera_label = tk.Label(self.main_frame)
            self.camera_label.pack()
            self.update_camera_feed()
        except Exception as e:
            tk.messagebox.showerror("Camera Error", f"Failed to initialize camera: {e}")
            print(f"Camera error: {e}")
            self.stop_camera()

    def capture_image(self):
        if self.picam2 and self.picam2.started:
            try:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.picam2.capture_file(f"capture_{timestamp}.jpg")
                tk.messagebox.showinfo("Success", f"Image saved as capture_{timestamp}.jpg")
            except Exception as e:
                tk.messagebox.showerror("Camera Error", f"Failed to capture image: {e}")
                print(f"Capture error: {e}")

    def update_camera_feed(self):
        if self.picam2 and self.picam2.started:
            try:
                frame = self.picam2.capture_array()
                if frame.shape[2] == 4:
                    frame = frame[:, :, :3]
                image = Image.fromarray(frame)
                image = image.resize((640, 480), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(image)
                self.camera_label.config(image=photo)
                self.camera_label.image = photo
                self.root.after(100, self.update_camera_feed)
            except Exception as e:
                print(f"Error updating camera feed: {e}")
                self.stop_camera()
                tk.messagebox.showerror("Camera Error", f"Failed to update camera feed: {e}")

    def show_statistics(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Statistics", font=("Arial", 16)).pack(pady=20)
        self.stop_camera()

    def show_component_status(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Component Status", font=("Arial", 16)).pack(pady=20)
        tk.Button(self.main_frame, text="Stop Servo", width=15, command=lambda: self.send_servo_command('1')).pack(pady=5)
        tk.Button(self.main_frame, text="Rotate +60°", width=15, command=lambda: self.send_servo_command('2')).pack(pady=5)
        tk.Button(self.main_frame, text="Rotate -60°", width=15, command=lambda: self.send_servo_command('3')).pack(pady=5)
        self.stop_camera()

    def send_servo_command(self, cmd):
        if arduino is None:
            print("Arduino not connected!")
            tk.messagebox.showerror("Error", "Arduino not connected. Please check the connection.")
            return
        try:
            arduino.write(cmd.encode())
            time.sleep(0.5)
            if arduino.in_waiting > 0:
                response = arduino.readline().decode('utf-8').strip()
                print(f"Arduino response: {response}")
                tk.messagebox.showinfo("Success", response)
        except Exception as e:
            print(f"Error sending command: {e}")
            tk.messagebox.showerror("Error", f"Failed to send command: {e}")

    def show_about(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="About ACSS", font=("Arial", 16)).pack(pady=20)
        self.stop_camera()

    def stop_camera(self):
        if self.picam2:
            try:
                self.picam2.stop()
                self.picam2.close()
                self.picam2 = None
            except Exception as e:
                print(f"Error stopping camera: {e}")

    def shutdown_app(self):
        self.stop_camera()
        if arduino is not None:
            arduino.close()
        self.root.destroy()

    def clear_main_frame(self):
        for widget in self.main_frame.winfo_children():
            if widget != self.status_label:
                widget.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.mainloop()