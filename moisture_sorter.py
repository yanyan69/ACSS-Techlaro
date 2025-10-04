#!/usr/bin/env python3
import serial
import csv
import time

# ================= SETTINGS =================
SERIAL_PORT = '/dev/ttyUSB0'   # Change if needed
BAUD_RATE = 115200
FILENAME = "moisture_log.csv"

# Thresholds (easy to edit)
RAW_MIN = 20.1        # moisture > RAW_MIN → Raw
STANDARD_MIN = 10.0   # moisture between STANDARD_MIN and RAW_MIN → Standard
STANDARD_MAX = 20.0
OVERCOOKED_MAX = 9.9  # moisture < OVERCOOKED_MAX → Overcooked

# ======================================================

def classify_moisture(moisture):
    """Return label based on moisture thresholds."""
    if moisture >= RAW_MIN:
        return "Raw"
    elif STANDARD_MIN <= moisture <= STANDARD_MAX:
        return "Standard"
    elif moisture <= OVERCOOKED_MAX:
        return "Overcooked"
    else:
        return "Unknown"

# Make sure CSV has headers
try:
    with open(FILENAME, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Moisture(%)", "Class", "CH_R", "CH_S", "CH_T", "CH_U", "CH_V", "CH_W"])
except FileExistsError:
    pass

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print("=== AS7263 Moisture Sorter (CLI) ===")
print("Logging to", FILENAME)
print("Press Ctrl+C to stop.")

while True:
    try:
        line = ser.readline().decode("utf-8").strip()

        if not line:
            continue

        try:
            # Parse sensor data
            ch_vals = [float(x) for x in line.split(",")]

            # Example: using CH_R (index 0) as proxy for "moisture"
            # TODO: replace formula with calibration curve if needed
            moisture = ch_vals[0]

            if moisture == 0:
                continue  # Skip noise/empty

            # Classify
            label = classify_moisture(moisture)

            # Save to CSV
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(FILENAME, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, moisture, label] + ch_vals)

            # Print summary line
            print(f"{timestamp} | Moisture={moisture:.2f}% | Class={label}")

        except ValueError:
            continue  # Skip bad lines

        time.sleep(1)  # Delay to reduce spamming

    except KeyboardInterrupt:
        print("\nExiting logger.")
        break