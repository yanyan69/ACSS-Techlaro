import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import cv2
import os
import time

# Setup folders
folders = ["standard", "raw", "overcooked"]
for folder in folders:
    os.makedirs(folder, exist_ok=True)

# Setup camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Capture function
def capture_image(mode):
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"{mode}/{mode}_{timestamp}.jpg"
    image = picam2.capture_array()
    cv2.imwrite(filename, image)

    # Show indicator
    status_label.config(text=f"Captured: {mode}", fg="green")
    window.after(1500, lambda: status_label.config(text=""))  # Clear after 1.5s

# Live preview update
def update_frame():
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = ImageTk.PhotoImage(Image.fromarray(frame))
    camera_label.imgtk = img
    camera_label.configure(image=img)
    camera_label.after(10, update_frame)

# GUI
window = tk.Tk()
window.title("Copra Image Capture")
window.geometry("800x600")

camera_label = tk.Label(window)
camera_label.pack()

# Buttons
button_frame = tk.Frame(window)
button_frame.pack()

tk.Button(button_frame, text="Standard", command=lambda: capture_image("standard"), bg="lightblue").grid(row=0, column=0, padx=5, pady=5)
tk.Button(button_frame, text="Raw", command=lambda: capture_image("raw"), bg="lightgreen").grid(row=0, column=1, padx=5, pady=5)
tk.Button(button_frame, text="Overcooked", command=lambda: capture_image("overcooked"), bg="orange").grid(row=0, column=2, padx=5, pady=5)

status_label = tk.Label(window, text="", font=("Arial", 14))
status_label.pack(pady=10)

update_frame()
window.mainloop()
