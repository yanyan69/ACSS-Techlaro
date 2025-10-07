#!/usr/bin/env python3
"""
ACSS Data Gathering GUI for Raspberry Pi 5
- Collects AS7263 spectral sensor readings via Arduino serial
- Continuously displays sensor data
- Saves to CSV only when class button is clicked
"""

import sys, threading, time, csv, os

try:
    import tkinter as tk
    from tkinter import scrolledtext
except Exception as e:
    print("tkinter missing - GUI won't run.", e)
    sys.exit(1)

try:
    import serial
except Exception:
    serial = None
    print("pyserial not available")

# ------------------ USER SETTINGS ------------------
SERIAL_PORT = "COM6"   # update for your Pi /dev/ttyUSB0
SERIAL_BAUD = 9600
CSV_FILENAME = "dataset_gathering/data/as7263_dataset.csv"
# ---------------------------------------------------

class DataGatherGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Data Gatherer")
        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.file_lock = threading.Lock()
        self.latest_vals = None  # last received AS7263 values

        # create folder if missing
        os.makedirs("data", exist_ok=True)

        # create/open CSV file with header
        if not os.path.exists(CSV_FILENAME):
            with open(CSV_FILENAME, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "label", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6"])

        # UI
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)

        # --- Serial controls ---
        conn_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        conn_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn_frame, text="Open Serial", command=self.open_serial).grid(row=0, column=1)
        tk.Button(conn_frame, text="Close Serial", command=self.close_serial).grid(row=0, column=2)

        # --- Label buttons ---
        label_frame = tk.LabelFrame(frm, text="Save Current Reading with Label")
        label_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)

        tk.Button(label_frame, text="Standard", command=lambda: self.save_current("standard")).pack(side="left", padx=5)
        tk.Button(label_frame, text="Raw", command=lambda: self.save_current("raw")).pack(side="left", padx=5)
        tk.Button(label_frame, text="Overcooked", command=lambda: self.save_current("overcooked")).pack(side="left", padx=5)

        # --- Log output ---
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=2, column=0, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, height=12, width=80, state='disabled', wrap='none')
        self.log.pack(fill='both', expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # --- Logging helpers ---
    def logmsg(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.config(state='normal')
        self.log.insert('end', full_msg + "\n")
        self.log.see('end')
        self.log.config(state='disabled')
        print(full_msg)

    # --- Serial handling ---
    def open_serial(self):
        if serial is None:
            self.logmsg("pyserial not available.")
            return
        if self.serial and self.serial.is_open:
            self.logmsg("Serial already open.")
            return
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
            self.logmsg(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}")
            threading.Thread(target=self.serial_reader, daemon=True).start()
        except Exception as e:
            self.logmsg("Failed to open serial: " + str(e))

    def close_serial(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logmsg("Serial closed.")
        except Exception as e:
            self.logmsg("Error closing serial: " + str(e))

    def save_current(self, lbl):
        if self.latest_vals and len(self.latest_vals) == 6:
            with self.file_lock:
                with open(CSV_FILENAME, "a", newline="") as f:
                    writer = csv.writer(f)
                    timestamp = time.strftime("%m/%d/%Y %H:%M:%S")  # full timestamp
                    writer.writerow([timestamp, lbl] + self.latest_vals)
            self.logmsg(f"Saved one sample with label={lbl}: {self.latest_vals}")
        else:
            self.logmsg("No valid sensor data yet.")

    def serial_reader(self):
        while self.serial and self.serial.is_open:
            try:
                line = self.serial.readline().decode("utf-8").strip()
                if not line:
                    continue

                try:
                    vals = [float(v) for v in line.split(",")]
                    if len(vals) == 6:
                        self.latest_vals = vals
                        self.logmsg("RX <- " + line)
                        continue
                except ValueError:
                    pass

                self.logmsg("DBG <- " + line)

            except Exception as e:
                self.logmsg(f"Serial error: {e}")
                break

    def on_close(self):
        self.running = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = DataGatherGui(root)
    root.mainloop()
