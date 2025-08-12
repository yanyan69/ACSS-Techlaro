from picamera2 import Picamera2
import tkinter as tk
from PIL import Image, ImageTk
import time
import os

# === Setup camera ===
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# === Create folders if not exist ===
folders = ["standard", "raw", "overcooked"]
for folder in folders:
    os.makedirs(folder, exist_ok=True)

# === Capture function ===
def capture_image(folder):
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"{folder}/{timestamp}.jpg"
    picam2.capture_file(filename)
    print(f"Saved: {filename}")

# === Update camera preview ===
def update_frame():
    frame = picam2.capture_array()
    img = Image.fromarray(frame)
    img_tk = ImageTk.PhotoImage(image=img)
    camera_label.imgtk = img_tk
    camera_label.configure(image=img_tk)
    camera_label.after(10, update_frame)

# === GUI setup ===
root = tk.Tk()
root.title("Copra Image Capture")

camera_label = tk.Label(root)
camera_label.pack()

btn_frame = tk.Frame(root)
btn_frame.pack()

tk.Button(btn_frame, text="Standard", width=15, command=lambda: capture_image("standard")).grid(row=0, column=0, padx=5, pady=5)
tk.Button(btn_frame, text="Raw", width=15, command=lambda: capture_image("raw")).grid(row=0, column=1, padx=5, pady=5)
tk.Button(btn_frame, text="Overcooked", width=15, command=lambda: capture_image("overcooked")).grid(row=0, column=2, padx=5, pady=5)

# Start preview
update_frame()
root.mainloop()
