#!/usr/bin/env python3
"""
ACSS GUI with ttkbootstrap (Modern Design + Frames + Log + About + Exit)
"""

import ttkbootstrap as ttkb
from ttkbootstrap.constants import *
from ttkbootstrap.scrolled import ScrolledText
from ttkbootstrap.tableview import Tableview
from tkinter import messagebox, Canvas, PhotoImage
import tkinter as tk  # For some variables and protocols

# ------------------ USER SETTINGS ------------------
CAM_PREVIEW_SIZE = (640, 480)
USERNAME = "User001"
# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System")
        root.geometry("1280x720")  # Increased window size to prevent clipping and ensure proper frame dimensions

        # Apply theme
        self.style = ttkb.Style(theme="superhero")  # Modern dark theme; alternatives: 'litera', 'cosmo', etc.

        # Disable maximize if desired
        # root.resizable(0, 0)  # Uncomment to disable resizing

        # Handle window close event
        root.protocol("WM_DELETE_WINDOW", self._handle_close)

        # Notebook (Tabs)
        notebook = ttkb.Notebook(root, bootstyle=PRIMARY)
        notebook.pack(fill=BOTH, expand=True)

        # Create tabs
        self.home_tab = ttkb.Frame(notebook)
        self.statistics_tab = ttkb.Frame(notebook)
        self.settings_tab = ttkb.Frame(notebook)
        self.about_tab = ttkb.Frame(notebook)
        self.exit_tab = ttkb.Frame(notebook)

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

        # State tracking for Start/Stop button
        self.camera_running = False

    def _handle_close(self):
        if messagebox.askokcancel("Close", "Are you sure you want to close the application?"):
            self.root.destroy()

    # -------------------- HOME --------------------
    def _build_home_tab(self):
        frm = ttkb.Frame(self.home_tab)
        frm.pack(fill=BOTH, expand=True, padx=8, pady=8)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        # Left side: Camera + Button
        left_frame = ttkb.Frame(frm)
        left_frame.grid(row=0, column=0, sticky=NSEW, padx=4, pady=4)
        left_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=0)
        left_frame.columnconfigure(0, weight=1)

        cam_frame = ttkb.Labelframe(left_frame, text="Camera Preview", bootstyle=INFO)
        cam_frame.grid(row=0, column=0, sticky=NSEW, padx=4, pady=4)
        self.cam_canvas = Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0],
                                 height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(padx=4, pady=4)

        self.start_btn = ttkb.Button(left_frame, text="Start Camera",
                                     bootstyle=(SUCCESS, OUTLINE), command=self._toggle_camera)
        self.start_btn.grid(row=1, column=0, sticky=EW, padx=4, pady=8)

        # Right side: Log (fills vertical space)
        log_frame = ttkb.Labelframe(frm, text="Log", bootstyle=INFO)
        log_frame.grid(row=0, column=1, sticky=NSEW, padx=4, pady=4)
        self.log = ScrolledText(log_frame, autohide=True, wrap='none', width=40)  # Set width to prevent overly wide log causing clipping
        self.log.pack(fill=BOTH, expand=True)

    def _toggle_camera(self):
        if not self.camera_running:
            self.start_btn.configure(text="Stop Camera", bootstyle=(DANGER, OUTLINE))
            self._log_message("Camera started.")
        else:
            self.start_btn.configure(text="Start Camera", bootstyle=(SUCCESS, OUTLINE))
            self._log_message("Camera stopped.")
        self.camera_running = not self.camera_running

    def _log_message(self, msg):
        self.log.insert(END, msg + "\n")
        self.log.see(END)

    # ----------------- STATISTICS -----------------
    def _build_statistics_tab(self):
        frm = ttkb.Frame(self.statistics_tab)
        frm.pack(fill=BOTH, expand=True, padx=8, pady=8)

        stats_frame = ttkb.Labelframe(frm, text="Processing Statistics", bootstyle=SECONDARY)
        stats_frame.pack(fill=BOTH, expand=True, padx=4, pady=4)

        # Username at top
        ttkb.Label(stats_frame, text=f"Username: {USERNAME}",
                   font=("Arial", 12, "bold"), bootstyle=LIGHT).grid(row=0, column=0, columnspan=3,
                                                                     sticky=W, pady=(5, 10))

        # Use Tableview for statistics
        coldata = [
            {"text": "Category", "stretch": False, "width": 150},
            {"text": "Pieces Processed", "stretch": False, "width": 150},
            {"text": "Avg. Moisture", "stretch": False, "width": 150},
        ]
        rowdata = [
            ("Standard", "0", "0.0%"),
            ("Raw", "0", "0.0%"),
            ("Overcooked", "0", "0.0%"),
        ]
        self.stats_table = Tableview(
            master=stats_frame,
            coldata=coldata,
            rowdata=rowdata,
            searchable=False,
            bootstyle=PRIMARY,
            paginated=False,
            autoalign=False
        )
        self.stats_table.grid(row=1, column=0, columnspan=3, sticky=NSEW, padx=4, pady=4)

        # Totals and processing info
        totals_frame = ttkb.Frame(stats_frame)
        totals_frame.grid(row=2, column=0, columnspan=3, sticky=W, pady=10)

        self.total_pieces_lbl = ttkb.Label(totals_frame, text="Total Pieces Processed: 0", bootstyle=INFO)
        self.total_pieces_lbl.pack(anchor=W, pady=4)

        self.total_moisture_lbl = ttkb.Label(totals_frame, text="Total Avg Moisture: 0.0%", bootstyle=INFO)
        self.total_moisture_lbl.pack(anchor=W, pady=4)

        self.start_time_lbl = ttkb.Label(totals_frame, text="Processing Start Time: N/A", bootstyle=INFO)
        self.start_time_lbl.pack(anchor=W, pady=4)

        self.end_time_lbl = ttkb.Label(totals_frame, text="Processing End Time: N/A", bootstyle=INFO)
        self.end_time_lbl.pack(anchor=W, pady=4)

    # ------------------- SETTINGS -----------------
    def _build_settings_tab(self):
        frm = ttkb.Frame(self.settings_tab)
        frm.pack(fill=BOTH, expand=True, padx=8, pady=8)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        # System settings
        settings_frame = ttkb.Labelframe(frm, text="System Settings", bootstyle=WARNING)
        settings_frame.grid(row=0, column=0, sticky=NSEW, padx=4, pady=4)

        ttkb.Label(settings_frame, text="Camera Resolution:").grid(row=0, column=0, sticky=W, padx=5, pady=5)
        self.res_entry = ttkb.Entry(settings_frame)
        self.res_entry.grid(row=0, column=1, padx=5, pady=5)

        ttkb.Label(settings_frame, text="Storage Path:").grid(row=1, column=0, sticky=W, padx=5, pady=5)
        self.path_entry = ttkb.Entry(settings_frame)
        self.path_entry.grid(row=1, column=1, padx=5, pady=5)

        ttkb.Button(settings_frame, text="Save Settings", bootstyle=SUCCESS).grid(row=2, column=0, columnspan=2, pady=10)

        # Component test area
        tests_frame = ttkb.Labelframe(frm, text="Component Tests", bootstyle=WARNING)
        tests_frame.grid(row=0, column=1, sticky=NSEW, padx=4, pady=4)

        # Servo buttons
        servo_frame = ttkb.Frame(tests_frame)
        servo_frame.pack(pady=5)
        ttkb.Button(servo_frame, text="Servo: LEFT", bootstyle=INFO).pack(side=LEFT, padx=2)
        ttkb.Button(servo_frame, text="Servo: CENTER", bootstyle=INFO).pack(side=LEFT, padx=2)
        ttkb.Button(servo_frame, text="Servo: RIGHT", bootstyle=INFO).pack(side=LEFT, padx=2)

        # Motor buttons
        motor_frame = ttkb.Frame(tests_frame)
        motor_frame.pack(pady=5)
        ttkb.Button(motor_frame, text="Motor ON", bootstyle=SUCCESS).pack(side=LEFT, padx=2)
        ttkb.Button(motor_frame, text="Motor OFF", bootstyle=DANGER).pack(side=LEFT, padx=2)

    # ------------------- ABOUT --------------------
    def _build_about_tab(self):
        frm = ttkb.Frame(self.about_tab)
        frm.pack(fill=BOTH, expand=True, padx=8, pady=8)

        about_frame = ttkb.Labelframe(frm, text="About ACSS", bootstyle=SECONDARY)
        about_frame.pack(fill=BOTH, expand=True, padx=4, pady=4)

        ttkb.Label(about_frame,
                   text="Automated Copra Segregation System\n\n"
                        "Version: 1.0 (GUI Prototype)\n"
                        "Developed for Technopreneurship Project\n"
                        "Marinduque State University, 2025",
                   font=("Arial", 12), bootstyle=LIGHT, anchor=CENTER, justify=CENTER).pack(pady=40)

    # ------------------- EXIT ---------------------
    def _build_exit_tab(self):
        frm = ttkb.Frame(self.exit_tab)
        frm.pack(fill=BOTH, expand=True, padx=8, pady=8)

        exit_frame = ttkb.Labelframe(frm, text="Exit Application", bootstyle=DANGER)
        exit_frame.pack(fill=BOTH, expand=True, padx=4, pady=4)

        ttkb.Label(exit_frame, text="Click the button below to close the program.",
                   font=("Arial", 12), bootstyle=LIGHT).pack(pady=20)

        ttkb.Button(exit_frame, text="Exit Program", bootstyle=DANGER,
                    width=20, command=self._exit_program).pack(pady=30)

    def _exit_program(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit?"):
            self.root.destroy()


if __name__ == "__main__":
    root = ttkb.Window()
    app = ACSSGui(root)
    root.mainloop()