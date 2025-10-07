#!/usr/bin/env python3
"""
ACSS Component Test GUI - GUI Layout Only
(Stripped down version without backend logic)
"""

import tkinter as tk
from tkinter import scrolledtext

# ------------------ USER SETTINGS ------------------
SERIAL_PORT = "/dev/ttyUSB0"   # placeholder only
CAM_PREVIEW_SIZE = (480, 360)
# ---------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("ACSS Component Tester (GUI Only)")

        # UI elements
        frm = tk.Frame(root)
        frm.pack(padx=8, pady=8)
        frm.rowconfigure(2, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        # connection controls
        conn_frame = tk.LabelFrame(frm, text="Serial / Arduino")
        conn_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        tk.Label(conn_frame, text=f"Port: {SERIAL_PORT}").grid(row=0, column=0, sticky="w")
        tk.Button(conn_frame, text="Open Serial").grid(row=0, column=1)
        tk.Button(conn_frame, text="Close Serial").grid(row=0, column=2)

        # component tests
        tests_frame = tk.LabelFrame(frm, text="Tests")
        tests_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        tk.Button(tests_frame, text="Servo: LEFT").grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: CENTER").grid(row=0, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Servo: RIGHT").grid(row=0, column=2, padx=2, pady=2)

        tk.Button(tests_frame, text="Motor ON").grid(row=1, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Motor OFF").grid(row=1, column=1, padx=2, pady=2)

        tk.Button(tests_frame, text="Request AS7263").grid(row=2, column=0, padx=2, pady=2)
        tk.Button(tests_frame, text="Start IR Monitor").grid(row=2, column=1, padx=2, pady=2)
        tk.Button(tests_frame, text="Stop IR Monitor").grid(row=2, column=2, padx=2, pady=2)

        # camera controls
        cam_frame = tk.LabelFrame(frm, text="Camera / YOLO")
        cam_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack()
        tk.Button(cam_frame, text="Start Camera Preview").pack(side='left', padx=4, pady=4)
        tk.Button(cam_frame, text="Stop Camera").pack(side='left', padx=4, pady=4)

        # logger
        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(
            log_frame, height=6, width=80, state='normal', wrap='none'
        )
        self.log.pack(fill='both', expand=True)

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()