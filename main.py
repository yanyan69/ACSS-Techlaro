import tkinter as tk
import sqlite3
from datetime import datetime
import os
import serial
import serial.tools.list_ports
import time

DB_FILE = 'data/acss_stats.db'

# 🔹 Setup Arduino Serial Connection
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'CH340' in port.description or 'CH341' in port.description:
            return port.device
    return None

try:
    port = find_arduino_port()
    if port is None:
        raise Exception("Arduino not found. Please check the connection.")
    arduino = serial.Serial(port=port, baudrate=9600, timeout=1)
    time.sleep(2)  # wait for Arduino reset
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    arduino = None  # Allow the app to run without Arduino

class ACSS_App:
    def __init__(self, root):
        self.root = root
        self.root.title('ACSS Control Panel')
        self.root.geometry('900x500')
        self.sorting_running = False
        self.sidebar_expanded = True
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
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
            self.sidebar.config(width=50)  # Collapse
            for btn in self.buttons:
                btn.config(text="", width=2)
        else:
            self.sidebar.config(width=200)  # Expand
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
        
    def show_camera_view(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Camera View", font=("Arial", 16)).pack(pady=20)
    
    def show_statistics(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Statistics", font=("Arial", 16)).pack(pady=20)
        
    def show_component_status(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="Component Status", font=("Arial", 16)).pack(pady=20)

        # 🔹 Servo control buttons
        tk.Button(self.main_frame, text="Rotate 0°", width=15, command=lambda: self.send_servo_command('1')).pack(pady=5)
        tk.Button(self.main_frame, text="Rotate +60°", width=15, command=lambda: self.send_servo_command('2')).pack(pady=5)
        tk.Button(self.main_frame, text="Rotate -60°", width=15, command=lambda: self.send_servo_command('3')).pack(pady=5)

    def send_servo_command(self, cmd):
        try:
            arduino.write(cmd.encode())
            print(f"Sent command {cmd} to Arduino")
        except Exception as e:
            print("Error sending command:", e)

    def show_about(self):
        self.clear_main_frame()
        tk.Label(self.main_frame, text="About ACSS", font=("Arial", 16)).pack(pady=20)
    
    def shutdown_app(self):
        self.root.destroy()

    def clear_main_frame(self):
        for widget in self.main_frame.winfo_children():
            widget.destroy()
    
    def send_servo_command(self, cmd):
        if arduino is None:
            print("Arduino not connected!")
            return
        try:
            arduino.write(cmd.encode())
            print(f"Sent command {cmd} to Arduino")
        except Exception as e:
            print("Error sending command:", e)

if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.mainloop()