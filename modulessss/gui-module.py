#!/usr/bin/env python3
"""
Isolated ACSS GUI - Extracted and standalone version without hardware dependencies.
- Removed serial, camera, YOLO, and processing logic.
- GUI builds and runs with tabs, logs, and stats.
- Simulated status and logs for demonstration.
- No auto-start; status is static.
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import sys, threading, time

# ------------------ USER SETTINGS ------------------
CAM_PREVIEW_SIZE = (640, 480)
USERNAME = "Copra Buyer 01"
# ----------------------------------------------------

class ACSSGui:
    def __init__(self, root):
        self.root = root
        root.title("Automated Copra Segregation System (AUTO)")
        self.root.attributes('-fullscreen', True)

        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)

        self.home_tab = ttk.Frame(notebook)
        self.statistics_tab = ttk.Frame(notebook)
        self.about_tab = ttk.Frame(notebook)
        self.exit_tab = ttk.Frame(notebook)

        notebook.add(self.home_tab, text="Home")
        notebook.add(self.statistics_tab, text="Statistics")
        notebook.add(self.about_tab, text="About")
        notebook.add(self.exit_tab, text="Exit")

        self._build_home_tab()
        self._build_statistics_tab()
        self._build_about_tab()
        self._build_exit_tab()

        # State (simplified)
        self.running = True
        self.stats = {'Raw': 0, 'Standard': 0, 'Overcooked': 0, 'total': 0, 'start_time': None, 'end_time': None}
        self.moisture_sums = {'Raw': 0.0, 'Standard': 0.0, 'Overcooked': 0.0}

        # Simulated status
        self.status_label.config(text="Status: GUI Isolated (No Hardware)")

        # Example log messages
        self._log_message("GUI initialized.")
        self._log_message("No hardware dependencies loaded.")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- UI BUILD ----------------
    def _build_home_tab(self):
        frm = tk.Frame(self.home_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=3)
        frm.columnconfigure(1, weight=2)

        left_frame = tk.Frame(frm)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        left_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=0)
        left_frame.columnconfigure(0, weight=1)

        cam_frame = tk.LabelFrame(left_frame, text="Camera Preview")
        cam_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.cam_canvas = tk.Canvas(cam_frame, width=CAM_PREVIEW_SIZE[0], height=CAM_PREVIEW_SIZE[1], bg='black')
        self.cam_canvas.pack(padx=4, pady=4)

        status_frame = tk.Frame(left_frame)
        status_frame.grid(row=1, column=0, sticky="ew", padx=4, pady=8)
        self.status_label = tk.Label(status_frame, text="Status: Initializing...", font=("Arial", 12, "bold"))
        self.status_label.pack(fill="x")

        log_frame = tk.LabelFrame(frm, text="Log")
        log_frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, state='disabled', wrap='word', height=20, width=40)
        self.log.pack(fill='both', expand=True)

    def _build_statistics_tab(self):
        frm = tk.Frame(self.statistics_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        stats_frame = tk.LabelFrame(frm, text="Processing Statistics")
        stats_frame.pack(fill="both", expand=True, padx=4, pady=4)
        stats_frame.columnconfigure(0, weight=1)
        stats_frame.columnconfigure(1, weight=1)
        stats_frame.columnconfigure(2, weight=1)

        tk.Label(stats_frame, text=f"Username: {USERNAME}", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)

        headers = ["Category", "Pieces Processed", "Avg. Moisture"]
        for col, header in enumerate(headers):
            tk.Label(stats_frame, text=header, font=("Arial", 10, "bold")).grid(row=1, column=col, padx=8, pady=4)

        categories = ["Standard", "Raw", "Overcooked"]
        self.stat_pieces = {}
        self.stat_moisture = {}
        for i, cat in enumerate(categories, start=2):
            tk.Label(stats_frame, text=cat).grid(row=i, column=0, padx=8, pady=4, sticky="w")
            self.stat_pieces[cat] = tk.Label(stats_frame, text="0")
            self.stat_pieces[cat].grid(row=i, column=1, padx=8, pady=4)
            self.stat_moisture[cat] = tk.Label(stats_frame, text="0.0%")
            self.stat_moisture[cat].grid(row=i, column=2, padx=8, pady=4)

        row_offset = len(categories) + 2
        tk.Label(stats_frame, text="Total Pieces Processed:", font=("Arial", 10, "bold")).grid(row=row_offset, column=0, padx=8, pady=4, sticky="w")
        self.total_pieces_label = tk.Label(stats_frame, text="0")
        self.total_pieces_label.grid(row=row_offset, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Total Avg Moisture:", font=("Arial", 10, "bold")).grid(row=row_offset+1, column=0, padx=8, pady=4, sticky="w")
        self.total_moisture_label = tk.Label(stats_frame, text="0.0%")
        self.total_moisture_label.grid(row=row_offset+1, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing Start Time:", font=("Arial", 10, "bold")).grid(row=row_offset+2, column=0, padx=8, pady=4, sticky="w")
        self.start_time_label = tk.Label(stats_frame, text="N/A")
        self.start_time_label.grid(row=row_offset+2, column=1, padx=8, pady=4)

        tk.Label(stats_frame, text="Processing End Time:", font=("Arial", 10, "bold")).grid(row=row_offset+3, column=0, padx=8, pady=4, sticky="w")
        self.end_time_label = tk.Label(stats_frame, text="N/A")
        self.end_time_label.grid(row=row_offset+3, column=1, padx=8, pady=4)

        tk.Button(stats_frame, text="Reset Statistics", font=("Arial", 10), command=self._reset_stats).grid(row=row_offset+4, column=0, columnspan=3, pady=10)

    def _build_about_tab(self):
        frm = tk.Frame(self.about_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        about_frame = tk.LabelFrame(frm, text="About ACSS")
        about_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(about_frame, text="Automated Copra Segregation System\n\nVersion: 1.0\nDeveloped for Practice and Design 2\nMarinduque State University, 2025", font=("Arial", 12), justify="center").pack(pady=40)

    def _build_exit_tab(self):
        frm = tk.Frame(self.exit_tab)
        frm.pack(fill="both", expand=True, padx=8, pady=8)

        exit_frame = tk.LabelFrame(frm, text="Exit Application")
        exit_frame.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(exit_frame, text="Click the button below to close the program.", font=("Arial", 12)).pack(pady=20)
        tk.Button(exit_frame, text="Exit Program", fg="white", bg="red", font=("Arial", 14, "bold"), width=20, command=self._exit_program).pack(pady=30)

    def _exit_program(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit?"):
            self.running = False
            self.on_close()

    # ----------------- Stats & utilities -----------------
    def update_stats(self):
        try:
            for cat in ["Standard", "Raw", "Overcooked"]:
                pieces = self.stats[cat]
                self.stat_pieces[cat].config(text=str(pieces))
                moisture_avg = (self.moisture_sums[cat] / pieces) if pieces > 0 else 0.0
                self.stat_moisture[cat].config(text=f"{moisture_avg:.1f}%")
            self.total_pieces_label.config(text=str(self.stats['total']))
            total_pieces = self.stats['total']
            total_moisture = sum(self.moisture_sums.values())
            total_avg = (total_moisture / total_pieces) if total_pieces > 0 else 0.0
            self.total_moisture_label.config(text=f"{total_avg:.1f}%")
            start_str = time.strftime("%H:%M:%S", time.localtime(self.stats['start_time'])) if self.stats['start_time'] else "N/A"
            end_str = time.strftime("%H:%M:%S", time.localtime(self.stats['end_time'])) if self.stats['end_time'] else "N/A"
            self.start_time_label.config(text=start_str)
            self.end_time_label.config(text=end_str)
            self.root.update()
        except Exception as e:
            self._log_message(f"Stats update error: {type(e).__name__}: {str(e)}")

    def _reset_stats(self):
        for cat in ["Raw", "Standard", "Overcooked"]:
            self.stats[cat] = 0
            self.moisture_sums[cat] = 0.0
        self.stats['total'] = 0
        self.stats['start_time'] = None
        self.stats['end_time'] = None
        self.update_stats()

    def _log_message(self, msg):
        ts = time.strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        try:
            self.log.config(state='normal')
            self.log.insert("end", full_msg + "\n")
            self.log.see("end")
            self.log.config(state='disabled')
        except Exception:
            print(full_msg)

    def on_close(self):
        self.running = False
        try:
            self._log_message("Application closed.")
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = ACSSGui(root)
    root.mainloop()