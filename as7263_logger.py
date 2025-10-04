#!/usr/bin/env python3
import serial
import csv
import time

# Adjust this to your Arduino port (check with `ls /dev/ttyUSB*`)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

filename = "data/as7263_log.csv"

# Create CSV with header if it doesn’t exist
try:
    with open(filename, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Label", "CH_R", "CH_S", "CH_T", "CH_U", "CH_V", "CH_W"])
except FileExistsError:
    pass

# Mapping for numeric input
label_map = {
    "1": "standard",
    "2": "raw",
    "3": "overcooked"
}

print("=== AS7263 Data Logger ===")
print("Press [1]=standard, [2]=raw, [3]=overcooked, [q]=quit")

while True:
    cmd = input("Log sample? (1/2/3/q): ").strip().lower()

    if cmd == "q":
        print("Exiting logger.")
        break
    elif cmd in label_map:
        # Read from Arduino
        line = ser.readline().decode("utf-8").strip()
        if line:
            try:
                ch_vals = [float(x) for x in line.split(",")]
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                label = label_map[cmd]

                # Save to CSV
                with open(filename, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, label] + ch_vals)

                print(f"Saved: {timestamp}, {label}, {ch_vals}")
            except Exception as e:
                print("Parse error:", e, "| Line was:", line)
        else:
            print("No data received.")
    else:
        print("Invalid input. Use 1, 2, 3, or q.")