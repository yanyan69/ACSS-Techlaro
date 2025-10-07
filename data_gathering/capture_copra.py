from picamera2 import Picamera2
import cv2
import time
import os

# === Setup camera ===
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
picam2.configure(config)
picam2.start()

# === Create folders if not exist ===
folders = ["raw_image/standard", "raw_image/raw", "raw_image/overcooked"]
for folder in folders:
    os.makedirs(folder, exist_ok=True)

print("Press '1' for Standard, '2' for Raw, '3' for Overcooked, 'q' to quit.")

# Toast state
toast_message = ""
toast_start_time = 0
TOAST_DURATION = 1.5  # seconds

while True:
    # Capture a frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    # === Draw Toast if active ===
    if toast_message and (time.time() - toast_start_time < TOAST_DURATION):
        alpha = 1 - ((time.time() - toast_start_time) / TOAST_DURATION)  # fade out
        overlay = frame.copy()
        h, w, _ = frame.shape

        # Background rectangle (semi-transparent)
        cv2.rectangle(
            overlay,
            (int(w/2 - 150), h - 60),
            (int(w/2 + 150), h - 20),
            (0, 0, 0),
            -1
        )
        cv2.addWeighted(overlay, 0.6 * alpha, frame, 1 - 0.6 * alpha, 0, frame)

        # Toast text (centered)
        cv2.putText(
            frame,
            toast_message,
            (int(w/2 - 130), h - 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )
    else:
        toast_message = ""  # clear message after fade

    # Display live feed
    cv2.imshow("Copra Image Capture", frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF

    if key == ord('1'):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"raw_image/standard/standard_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        toast_message = "Captured: Standard"
        toast_start_time = time.time()

    elif key == ord('2'):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"raw_image/raw/raw_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        toast_message = "Captured: Raw"
        toast_start_time = time.time()

    elif key == ord('3'):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"raw_image/overcooked/overcooked_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        toast_message = "Captured: Overcooked"
        toast_start_time = time.time()

    elif key == ord('q'):
        break

    # === NEW: Slow down loop to ~2 FPS ===
    time.sleep(1)

# Cleanup
cv2.destroyAllWindows()
picam2.stop()