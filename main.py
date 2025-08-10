import tkinter as tk 
import sqlite3 
from datetime import datetime 
import os

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
        self.init_db()
        self.setup_ui()
        
    def init_db(self):
        os.makedirs(os.path.dirname(DB_FILE), exist_ok=True)
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("""
            CREATE TABLE IF NOT EXISTS stats (
                id INTEGER PRIMARY KEY AUTOINCREMENT, 
                timestamp TEXT,
                processed_image_count INTEGER DEFAULT 0,
                processed_sensor_count INTEGER DEFAULT 0
            )
        """
        )
        conn.commit()
        conn.close()
        
    def setup_ui(self):
        self.sidebar_expanded = True
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
        self.toggle_button.pack(pady=50)

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
        self.sorting_running = not self.sorting_running
        if self.sorting_running:
            self.toggle_button.config(text='Stop', bg='red')
        else:
            self.toggle_button.config(text='Start', bg='green')
        
    def show_main_interface(self):
        print('main interface')
        
    def show_camera_view(self):
        print('this is a camera view')
    
    def show_statistics(self):
        print('statistics')
        
    def show_component_status(self):
        print('component status')
        
    def show_about(self):
        print('about me')
    
    def shutdown_app(self):
        self.root.destroy()
        
if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.mainloop()