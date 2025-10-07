#!/usr/bin/env python3
"""
ACSS GUI Skeleton with Tabs (Modern Design + Frames + Log + About + Exit)
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import time

# ------------------ USER SETTINGS ------------------
CAM_PREVIEW_SIZE = (640, 480)
USERNAME = "User001"
# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System")
        root.geometry("1024x600")

        # Notebook (Tabs)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        # Create tabs
        self.home_tab = ttk.Frame(notebook)
        self.statistics_tab = ttk.Frame(notebook)
        self.settings_tab = ttk.Frame(notebook)
        self.about_tab = ttk.Frame(notebook)
        self.exit_tab = ttk.Frame(notebook)

        notebook.add(self.home_tab, text="Home")
        notebook.add(self.statistics_tab, text="Statistics")
        notebook.add(self.settings_tab, text="Settings")
        notebook.add(self.about_tab, text="About")
        notebook.add(self.exit_tab, text="Exit")

        # Build tab contents
        self._build_home_tab()
        self._build_statistics_tab()
        self._build_settings_tab()
        self._build_about_tab()
        self._build_exit_tab()

        # State tracking (GUI-related only)
        self.camera_running = False

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # -------------------- HOME --------------------
    def _build_home_tab(self):
        frm = tk.Frame(self.home_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=3)  # Left side wider for camera
        frm.columnconfigure(1, weight=2)  # Right side narrower for log to fit

        # Left side: Camera + Button
        left_frame = tk.Frame(frm)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        left_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=0)
        left_frame.columnconfigure(0, weight=1)

        cam_frame = tk.LabelFrame(left_frame, text="Camera Preview")
        cam_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0],
                                    height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(padx=4, pady=4)

        self.start_btn = tk.Button(left_frame, text="Start Camera",
                                   font=("Arial", 14, "bold"), bg="green", fg="white",
                                   command=self._toggle_camera)
        self.start_btn.grid(row=1, column=0, sticky="ew", padx=4, pady=8)

        # Right side: Log (adjusted size, with word wrap)
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, state='normal', wrap='word', height=20, width=40)  # Adjusted width and height to fit, wrap='word' for text wrapping
        self.log.pack(fill='both', expand=True)

    def _toggle_camera(self):
        if not self.camera_running:
            self.start_btn.config(text="Stop Camera", bg="red")
            self._log_message("Camera started.")
            self.camera_running = True
        else:
            self.start_btn.config(text="Start Camera", bg="green")
            self._log_message("Camera stopped.")
            self.camera_running = False

    def _log_message(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log.insert("end", full_msg + "\n")
        self.log.see("end")

    # ----------------- STATISTICS -----------------
    def _build_statistics_tab(self):
        frm = tk.Frame(self.statistics_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        stats_frame = tk.LabelFrame(frm, text="Processing Statistics")
        stats_frame.pack(fill="both", expand=True, padx=4, pady=4)

        # Add grid configuration for flexibility
        stats_frame.columnconfigure(0, weight=1)
        stats_frame.columnconfigure(1, weight=1)
        stats_frame.columnconfigure(2, weight=1)

        tk.Label(stats_frame, text=f"Username: {USERNAME}", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)

        headers = ["Category", "Pieces Processed", "Avg. Moisture"]
        for col, header in enumerate(headers):
            tk.Label(stats_frame, text=header, font=("Arial", 10, "bold")).grid(row=1, column=col, padx=8, pady=4)

        categories = ["Standard", "Raw", "Overcooked"]
        for i, cat in enumerate(categories, start=2):
            tk.Label(stats_frame, text=cat).grid(row=i, column=0, padx=8, pady=4, sticky="w")
            tk.Label(stats_frame, text="0").grid(row=i, column=1, padx=8, pady=4)
            tk.Label(stats_frame, text="0.0%").grid(row=i, column=2, padx=8, pady=4)

        row_offset = len(categories) + 2
        tk.Label(stats_frame, text="Total Pieces Processed:", font=("Arial", 10, "bold")).grid(row=row_offset, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="0").grid(row=row_offset, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Total Avg Moisture:", font=("Arial", 10, "bold")).grid(row=row_offset+1, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="0.0%").grid(row=row_offset+1, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing Start Time:", font=("Arial", 10, "bold")).grid(row=row_offset+2, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="N/A").grid(row=row_offset+2, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing End Time:", font=("Arial", 10, "bold")).grid(row=row_offset+3, column=0, padx=8, pady=4, sticky="w")
        tk.Label(stats_frame, text="N/A").grid(row=row_offset+3, column=1, padx=8, pady=4)

    # ------------------- SETTINGS -----------------
    def _build_settings_tab(self):
        frm = tk.Frame(self.settings_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)

        # Live status dashboard
        status_frame = tk.LabelFrame(frm, text="Live Component Status")
        status_frame.grid(row=2, column=0, sticky="nsew", padx=4, pady=4)
        self.serial_status_label = tk.Label(status_frame, text="Serial Connected: False", font=("Arial", 12))
        self.serial_status_label.pack(anchor="w", pady=2)
        self.camera_status_label = tk.Label(status_frame, text="Camera Running: False", font=("Arial", 12))
        self.camera_status_label.pack(anchor="w", pady=2)
        self.ir_status_label = tk.Label(status_frame, text="IR Proximity Sensor (distance): False ; N/A cm", font=("Arial", 12))
        self.ir_status_label.pack(anchor="w", pady=2)
        self.motor_status_label = tk.Label(status_frame, text="Motor: Off", font=("Arial", 12))
        self.motor_status_label.pack(anchor="w", pady=2)
        self.servo_status_label = tk.Label(status_frame, text="Servo Position: Center", font=("Arial", 12))
        self.servo_status_label.pack(anchor="w", pady=2)
        self.as_status_label = tk.Label(status_frame, text="AS7263 Values: N/A", font=("Arial", 12))
        self.as_status_label.pack(anchor="w", pady=2)

    # ------------------- ABOUT --------------------
    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        about_frame = tk.LabelFrame(frm, text="About ACSS")
        about_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(about_frame,
                 text="Automated Copra Segregation System\n\n"
                      "Version: 1.0 (GUI Prototype)\n"
                      "Developed for Technopreneurship Project\n"
                      "Marinduque State University, 2025",
                 font=("Arial", 12), justify="center").pack(pady=40)

    # ------------------- EXIT ---------------------
    def _build_exit_tab(self):
        frm = tk.Frame(self.exit_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        exit_frame = tk.LabelFrame(frm, text="Exit Application")
        exit_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(exit_frame, text="Click the button below to close the program.",
                 font=("Arial", 12)).pack(pady=20)

        tk.Button(exit_frame, text="Exit Program", fg="white", bg="red",
                  font=("Arial", 14, "bold"), width=20,
                  command=self._exit_program).pack(pady=30)

    def _exit_program(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit?"):
            self.on_close()

    # ---------- Shutdown ----------
    def on_close(self):
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()