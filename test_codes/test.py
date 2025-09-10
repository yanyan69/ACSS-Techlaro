import tkinter as tk
import threading
import cv2
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO
import time

from picamera2 import Picamera2  # import picamera2

DB_FILE = 'data/acss_stats.db'

class ACSS_App:
    def __init__(self, root):
        self.root = root
        self.root.title('ACSS Control Panel')
        self.root.geometry('900x500')

        self.sorting_running = False
        self.sidebar_expanded = True

        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        self.setup_ui()

        # YOLO related attributes
        self.model_path = 'my_model/train/weights/best.pt'  # Change to your model path
        self.model = None
        self.picam2 = None
        self.video_thread = None
        self.stop_event = threading.Event()

        # Stats counters
        self.processed_image_count = 0
        self.processed_sensor_count = 0  # Placeholder if you add sensor data

    def setup_ui(self):
        self.sidebar = tk.Frame(self.root, bg='#1E1E1E', width=200)
        self.sidebar.grid(row=0, column=0, sticky='ns')
        self.sidebar.grid_propagate(False)

        toggle_btn = tk.Button(self.sidebar, text='☰', command=self.toggle_sidebar, bg='#1E1E1E', fg='white', bd=0)
        toggle_btn.pack(fill='x')

        self.buttons = [
            tk.Button(self.sidebar, text="Main Interface", command=self.show_main_interface),
            tk.Button(self.sidebar, text="Camera View", command=self.show_camera_view),
            tk.Button(self.sidebar, text="Statistics", command=self.show_statistics),
            tk.Button(self.sidebar, text="Component Status", command=self.show_component_status),
            tk.Button(self.sidebar, text="About", command=self.show_about),
            tk.Button(self.sidebar, text="Exit", command=self.shutdown_app)
        ]

        for btn in self.buttons:
            btn.pack(fill='x')

        self.main_frame = tk.Frame(self.root, bg='white')
        self.main_frame.grid(row=0, column=1, sticky='nsew')

        # Create widgets once
        self.toggle_button = tk.Button(
            self.main_frame,
            text='Start',
            command=self.toggle_sorting,
            width=10,
            height=2,
            bg='green',
            fg='white',
            font=('Arial', 14, 'bold')
        )
        self.count_label = tk.Label(self.main_frame, text="Objects detected: 0", font=("Arial", 14), bg="white")
        self.video_label = tk.Label(self.main_frame)

        # Initially show main interface widgets
        self.show_main_interface()

    def toggle_sidebar(self):
        if self.sidebar_expanded:
            self.sidebar.config(width=50)  # Collapse
            for btn in self.buttons:
                btn.config(text="", width=2)
        else:
            self.sidebar.config(width=200)  # Expand
            texts = ["Main Interface", "Camera View", "Statistics", "Component Status", "About", "Exit"]
            for btn, text in zip(self.buttons, texts):
                btn.config(text=text, width=20)
        self.sidebar_expanded = not self.sidebar_expanded

    def toggle_sorting(self):
        if not self.sorting_running:
            self.start_detection()
            self.toggle_button.config(text='Stop', bg='red')
        else:
            self.stop_detection()
            self.toggle_button.config(text='Start', bg='green')
        self.sorting_running = not self.sorting_running

    def start_detection(self):
        self.stop_event.clear()
        if self.model is None:
            self.model = YOLO(self.model_path)

        if self.picam2 is None:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
            self.picam2.configure(config)
            self.picam2.start()

        self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
        self.video_thread.start()

    def stop_detection(self):
        self.stop_event.set()
        if self.picam2:
            self.picam2.stop()
            self.picam2 = None
        self.video_label.config(image='')  # Clear video feed
        self.count_label.config(text="Objects detected: 0")
        self.processed_image_count = 0

    def video_loop(self):
        bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
                      (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

        while not self.stop_event.is_set():
            frame_bgra = self.picam2.capture_array()
            # Convert BGRA to BGR
            frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)

            results = self.model(frame, verbose=False)
            detections = results[0].boxes

            object_count = 0

            for i in range(len(detections)):
                xyxy_tensor = detections[i].xyxy.cpu()
                xyxy = xyxy_tensor.numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy.astype(int)
                classidx = int(detections[i].cls.item())
                conf = detections[i].conf.item()

                if conf > 0.5:
                    color = bbox_colors[classidx % 10]
                    cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)
                    label = f'{self.model.names[classidx]}: {int(conf*100)}%'
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    label_ymin = max(ymin, labelSize[1] + 10)
                    cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED)
                    cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    object_count += 1

            self.processed_image_count += 1

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)

            def update_image():
                self.video_label.imgtk = imgtk
                self.video_label.config(image=imgtk)
                self.count_label.config(text=f"Objects detected: {object_count}")

            self.root.after(0, update_image)
            time.sleep(0.03)  # ~30 FPS

    def clear_main_frame(self):
        for widget in self.main_frame.winfo_children():
            widget.pack_forget()

    def show_main_interface(self):
        self.clear_main_frame()
        self.toggle_button.pack(pady=10)
        self.count_label.pack(pady=10)
        self.video_label.pack()

    def show_camera_view(self):
        self.clear_main_frame()
        # Hide controls, show video feed fullscreen
        self.toggle_button.pack_forget()
        self.count_label.pack_forget()
        self.video_label.pack(expand=True, fill='both')

    def show_statistics(self):
        self.clear_main_frame()
        label = tk.Label(self.main_frame, text="Statistics Section (to implement)", font=("Arial", 16))
        label.pack(pady=20)

    def show_component_status(self):
        self.clear_main_frame()
        label = tk.Label(self.main_frame, text="Component Status Section (to implement)", font=("Arial", 16))
        label.pack(pady=20)

    def show_about(self):
        self.clear_main_frame()
        label = tk.Label(self.main_frame, text="About Section (to implement)", font=("Arial", 16))
        label.pack(pady=20)

    def shutdown_app(self):
        self.stop_detection()
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown_app)
    root.mainloop()
