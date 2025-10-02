from picamera2 import Picamera2
import cv2   # CHANGED: use OpenCV instead of Tkinter GUI
import time
import os

# === Setup camera ===
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})  # CHANGED: use OpenCV compatible format
picam2.configure(config)
picam2.start()

# === Create folders if not exist ===
folders = ["standard", "raw", "overcooked"]
for folder in folders:
    os.makedirs(folder, exist_ok=True)

print("Press '1' for Standard, '2' for Raw, '3' for Overcooked, 'q' to quit.")

while True:
    # Capture a frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # CHANGED: convert to BGR for OpenCV display

    # Display live feed
    cv2.imshow("Copra Image Capture", frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF

    if key == ord('1'):  # CHANGED: Capture to "standard"
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"standard/{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

    elif key == ord('2'):  # CHANGED: Capture to "raw"
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"raw/{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

    elif key == ord('3'):  # CHANGED: Capture to "overcooked"
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"overcooked/{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

    elif key == ord('q'):  # Quit
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()