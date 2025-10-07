#!/usr/bin/env python3
"""
ACSS Component Test GUI - Modern Layout (Tabs + ttk)
Window fixed to 1024x600
"""

import tkinter as tk
from tkinter import ttk, scrolledtext

# ------------------ USER SETTINGS ------------------
SERIAL_PORT = "/dev/ttyUSB0"   # placeholder only
CAM_PREVIEW_SIZE = (480, 360)
WINDOW_SIZE = "1024x600"
# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Component Tester (Modern GUI)")
        root.geometry(WINDOW_SIZE)
        root.resizable(False, False)  # fixed window size

        # Create notebook (tabs)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, padx=5, pady=5)

        # ---- Tab 1: Connection ----
        conn_tab = ttk.Frame(notebook)
        notebook.add(conn_tab, text="Connection")

        conn_frame = ttk.LabelFrame(conn_tab, text="Serial / Arduino")
        conn_frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        ttk.Button(conn_frame, text="Open Serial").grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(conn_frame, text="Close Serial").grid(row=0, column=2, padx=5, pady=5)

        # ---- Tab 2: Tests ----
        tests_tab = ttk.Frame(notebook)
        notebook.add(tests_tab, text="Tests")

        tests_frame = ttk.LabelFrame(tests_tab, text="Component Tests")
        tests_frame.pack(fill="x", padx=10, pady=10)

        ttk.Button(tests_frame, text="Servo: LEFT").grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(tests_frame, text="Servo: CENTER").grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(tests_frame, text="Servo: RIGHT").grid(row=0, column=2, padx=5, pady=5)

        ttk.Button(tests_frame, text="Motor ON").grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(tests_frame, text="Motor OFF").grid(row=1, column=1, padx=5, pady=5)

        ttk.Button(tests_frame, text="Request AS7263").grid(row=2, column=0, padx=5, pady=5)
        ttk.Button(tests_frame, text="Start IR Monitor").grid(row=2, column=1, padx=5, pady=5)
        ttk.Button(tests_frame, text="Stop IR Monitor").grid(row=2, column=2, padx=5, pady=5)

        # ---- Tab 3: Camera ----
        cam_tab = ttk.Frame(notebook)
        notebook.add(cam_tab, text="Camera")

        cam_frame = ttk.LabelFrame(cam_tab, text="Camera / YOLO")
        cam_frame.pack(fill="both", padx=10, pady=10, expand=True)

        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(pady=10)

        btn_frame = ttk.Frame(cam_frame)
        btn_frame.pack(pady=5)
        ttk.Button(btn_frame, text="Start Camera Preview").pack(side='left', padx=10)
        ttk.Button(btn_frame, text="Stop Camera").pack(side='left', padx=10)

        # ---- Tab 4: Logs ----
        log_tab = ttk.Frame(notebook)
        notebook.add(log_tab, text="Logs")

        log_frame = ttk.LabelFrame(log_tab, text="System Log")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.log = scrolledtext.ScrolledText(
            log_frame, height=20, width=120, state='normal', wrap='word', font=("Consolas", 10)
        )
        self.log.pack(fill="both", expand=True)

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()