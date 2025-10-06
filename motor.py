#!/usr/bin/env python3
"""
ACSS Motor Control GUI for Raspberry Pi 5.
- Sends commands to Arduino: "MOTOR,ON" and "MOTOR,OFF"
- Simple serial log window for debugging
"""

import sys, threading, time
import tkinter as tk
from tkinter import scrolledtext

try:
    import serial
except Exception:
    serial = None
    print("⚠️ pyserial not available — install with: pip install pyserial")

# ---------- USER SETTINGS ----------
SERIAL_PORT = "/dev/ttyUSB0"   # ← Change this to match your Arduino port (check using: ls /dev/ttyUSB* or /dev/ttyACM*)
SERIAL_BAUD = 9600
# ----------------------------------

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Motor Test GUI")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True

        # ---------- Layout ----------
        frm = tk.Frame(root)
        frm.pack(padx=10, pady=10)

        conn_frame = tk.LabelFrame(frm, text="Arduino Connection")
        conn_frame.pack(fill='x', padx=5, pady=5)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT} @ {SERIAL_BAUD}").pack(side='left', padx=4)
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).pack(side='left', padx=4)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).pack(side='left', padx=4)

        ctrl_frame = tk.LabelFrame(frm, text="Motor Control")
        ctrl_frame.pack(fill='x', padx=5, pady=5)
        tk.Button(ctrl_frame, text="MOTOR ON", command=lambda: self.send_cmd("MOTOR,ON")).pack(side='left', padx=10, pady=5)
        tk.Button(ctrl_frame, text="MOTOR OFF", command=lambda: self.send_cmd("MOTOR,OFF")).pack(side='left', padx=10, pady=5)

        log_frame = tk.LabelFrame(frm, text="Serial Log")
        log_frame.pack(fill='both', expand=True, padx=5, pady=5)
        self.log = scrolledtext.ScrolledText(log_frame, height=8, state='disabled', wrap='none')
        self.log.pack(fill='both', expand=True)

        # Auto open serial
        self.logmsg("Attempting to open serial automatically...")
        self.open_serial()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Logging ----------
    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert('end', full + "\n")
        self.log.see('end')
        self.log.config(state='disabled')
        print(full)

    # ---------- Serial ----------
    def open_serial(self):
        try:
            if serial is None:
                self.logmsg("pyserial not available.")
                return
            if self.serial and self.serial.is_open:
                self.logmsg("Serial already open.")
                return
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)  # Increased timeout for reliability
            time.sleep(3)  # Increased wait for Arduino reset (some boards need more time)
            self.logmsg(f"✅ Opened serial {SERIAL_PORT} @ {SERIAL_BAUD}")
            threading.Thread(target=self.serial_reader, daemon=True).start()
        except Exception as e:
            self.logmsg(f"❌ Failed to open serial: {e}")

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("🔌 Serial closed.")
        except Exception as e:
            self.logmsg(f"Error closing serial: {e}")

    def send_cmd(self, text):
        """Send command to Arduino with newline."""
        try:
            if not self.serial or not self.serial.is_open:
                self.logmsg(f"[WARN] Serial not open — can't send '{text}'")
                return
            full_cmd = (text.strip().upper() + "\n").encode('utf-8')
            with self.serial_lock:
                self.serial.write(full_cmd)
                self.serial.flush()
            self.logmsg(f"[TX] {text.strip().upper()}")
            self.logmsg(f"[DEBUG] Sent bytes: {full_cmd}")  # Added for verbose debugging
        except Exception as e:
            self.logmsg(f"[ERR] Send failed: {e}")

    # ---------- Serial Reader ----------
    def serial_reader(self):
        self.logmsg("Serial reader started.")
        while self.serial and self.serial.is_open and self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if line:
                    self.logmsg(f"[RX] {line}")
                else:
                    self.logmsg("[DEBUG] Read timeout - no data received yet")  # Added for verbose debugging (comment out if too spammy)
            except Exception as e:
                self.logmsg(f"[Reader ERR] {e}")
                time.sleep(0.2)

    # ---------- Shutdown ----------
    def on_close(self):
        self.running = False
        self.close_serial()
        self.root.destroy()
        self.logmsg("🟢 GUI closed safely.")


if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()