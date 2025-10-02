"""
DESCRIPTION:
this code to capture images from the Raspberry Pi Camera

AUTHOR   : Solehin Rizal
WEBSITE  : www.cytron.io
EMAIL    : solehin@cytron.io
"""

import cv2
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

save_path = "/home/pi/" # replace with your path directory
image_count = 0

# Define ROI (x1, y1, x2, y2)
roi = (200, 100, 440, 380)  # Adjust these values for your ROI

print("Press 'c' to capture an image and 'q' to quit.")

while True:
    # Capture a frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    # Draw the ROI rectangle on the frame
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue rectangle for ROI

    # Display the live feed
    cv2.imshow("Capturing Images", frame)

    # Save an image when 'c' is pressed
    key = cv2.waitKey(1)
    if key & 0xFF == ord('c'):
        # Crop the ROI from the frame
        cropped_frame = frame[y1:y2, x1:x2]

        # Save the cropped image
        image_path = f"{save_path}image_{image_count}.jpg"
        cv2.imwrite(image_path, cropped_frame)
        print(f"Saved: {image_path}")
        image_count += 1

    # Exit on 'q' key press
    elif key & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()