#!/usr/bin/env python3
"""
ACSS Moisture Sorting Deployment Script
- Reads AS7263 values via Arduino
- Computes moisture (calibrated index)
- Classifies raw/standard/overcooked
- Sends servo sorting commands
- Logs all readings
"""

import serial
import time
import csv

# ================= SETTINGS =================
SERIAL_PORT = '/dev/ttyUSB0'  # Arduino
BAUD_RATE = 115200
LOG_FILE = "data_gathering/data/moisture_log.csv"

# Thresholds based on your analysis
OVERCOOKED_MAX = 5.9
STANDARD_MIN = 6.0
STANDARD_MAX = 7.0
RAW_MIN = 7.1

# Servo positions
SERVO_POS = {"overcooked": 60, "standard": 90, "raw": 120}
# ======================================================

# Setup CSV file
import os
os.makedirs("data", exist_ok=True)
try:
    with open(LOG_FILE, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Moisture(%)", "Class", "CH_R","CH_S","CH_T","CH_U","CH_V","CH_W"])
except FileExistsError:
    pass

# Open serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # wait for Arduino to boot

def classify_moisture(m):
    """Return class label based on thresholds"""
    if m <= OVERCOOKED_MAX:
        return "overcooked"
    elif STANDARD_MIN <= m <= STANDARD_MAX:
        return "standard"
    elif m >= RAW_MIN:
        return "raw"
    else:
        return "unknown"

def send_servo(label):
    """Send servo sorting command to Arduino"""
    if label in SERVO_POS:
        cmd = f"SORT,{label[0].upper()}\n"  # L/C/R based on first letter
        ser.write(cmd.encode())
        # optional: wait for ACK
        ack = ser.readline().decode().strip()
        print(f"Servo command sent: {cmd.strip()} | Arduino: {ack}")

print("=== ACSS Moisture Sorter Deployment ===")
print("Press Ctrl+C to stop.")

try:
    while True:
        # Request AS7263 reading
        ser.write(b"REQ_AS\n")
        line = ser.readline().decode().strip()
        if not line:
            continue

        try:
            ch_vals = [float(x) for x in line.split(",")]
            # Example: use channel 1 (CH_R) as moisture proxy
            moisture = ch_vals[0]

            if moisture == 0:
                continue  # skip invalid reading

            label = classify_moisture(moisture)

            # Log to CSV
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(LOG_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, moisture, label] + ch_vals)

            print(f"{timestamp} | Moisture={moisture:.2f}% | Class={label}")

            # Send servo command
            send_servo(label)

        except ValueError:
            continue

        time.sleep(0.5)  # adjust as needed

except KeyboardInterrupt:
    print("\nStopping deployment script.")
    ser.close()