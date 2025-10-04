#!/usr/bin/env python3
import serial
import csv
import time
import tkinter as tk
from tkinter import ttk

# === Arduino connection ===
ser = serial.Serial('COM6', 115200, timeout=1)

filename = "data/as7263_log.csv"

# Create CSV with header if it doesn’t exist
try:
    with open(filename, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Label", "CH_R", "CH_S", "CH_T", "CH_U", "CH_V", "CH_W"])
except FileExistsError:
    pass

# Label selection (default = "unlabeled")
current_label = "unlabeled"

def set_label(lbl):
    global current_label
    current_label = lbl
    label_var.set(f"Label: {lbl}")

def read_serial():
    line = ser.readline().decode("utf-8").strip()
    if line:
        data_var.set(f"RAW: {line}")  # just show raw line
        try:
            parts = line.split(",")
            ch_vals = [float(x) for x in parts if x.strip() != ""]
            if len(ch_vals) == 6 and not all(v == 0.0 for v in ch_vals):
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                with open(filename, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, current_label] + ch_vals)
        except Exception as e:
            data_var.set(f"RAW (parse error): {line}")
    root.after(1000, read_serial)


# === Tkinter GUI ===
root = tk.Tk()
root.title("AS7263 Live Logger")

label_var = tk.StringVar(value="Label: unlabeled")
data_var = tk.StringVar(value="Waiting for data...")

ttk.Label(root, textvariable=label_var, font=("Arial", 14)).pack(pady=5)
ttk.Label(root, textvariable=data_var, font=("Consolas", 12), wraplength=500).pack(pady=10)

frame = ttk.Frame(root)
frame.pack(pady=10)

ttk.Button(frame, text="Standard", command=lambda: set_label("standard")).grid(row=0, column=0, padx=5)
ttk.Button(frame, text="Raw", command=lambda: set_label("raw")).grid(row=0, column=1, padx=5)
ttk.Button(frame, text="Overcooked", command=lambda: set_label("overcooked")).grid(row=0, column=2, padx=5)

# Start periodic serial reading
read_serial()

root.mainloop()
