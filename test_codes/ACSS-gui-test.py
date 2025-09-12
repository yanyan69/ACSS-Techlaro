import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
import os
import webbrowser  # Added for cross-platform PDF opening

# Style constants (edit these to change colors, fonts, sizes, similar to CSS)
COLORS = {
    'bg_primary': '#FFFFFF',  # Sidebar background
    'bg_secondary': '#FFFFFF',  # Main frame background
    'btn_active': '#4CAF50',  # Active button color
    'btn_inactive': '#F44336',  # Inactive button color (e.g., Cancel Camera)
    'text_primary': '#374151',  # Primary text color
    'text_secondary': '#000000',  # Secondary text color
    'accent': '#1d4ed8',  # Accent color for buttons like Export PDF
    'highlight': '#D3D3D3',  # Sidebar button highlight color
}
FONTS = {
    'heading': ('Arial', 24, 'bold'),  # For main titles (e.g., About Us)
    'subheading': ('Arial', 18, 'bold'),  # For section subtitles (e.g., Our Company)
    'paragraph': ('Arial', 14),  # For body text
    'paragraph_bold': ('Arial', 14, 'bold'),  # For bold body text
    'table': ('Arial', 16),  # For table text
    'log': ('Arial', 12),  # For log text
    'button': ('Arial', 14),  # For button text
    'confirm': ('Arial', 16, 'bold'),  # For confirmation dialog
}
SIZES = {
    'sidebar_width': 300,  # Fixed width for sidebar
    'padding': 10,
    'icon_size': 64,
    'start_stop_size': 128,
    'small_button_size': 64,
    'photo_size': 100,
    'camera_height': 400,  # Fixed height for camera view to leave space for buttons
}
IMAGE_PATH = 'resources/icons/'
PROFILE_PATH = 'resources/profiles/'

class ACSS_App:
    def __init__(self, root):
        self.root = root
        self.root.title('Automated Copra Segregation System (PC Version)')
        self.root.geometry('1024x600')
        self.root.attributes('-fullscreen', True)
        self.root.bind('<Key-f>', lambda e: self.toggle_fullscreen())
        self.camera_visible = False
        self.sorting_running = False
        self.log_visible = False
        self.active_menu = None
        self.log_entries = []  # For dynamic logging
        self.stats_data = {  # Dynamic stats
            'Raw': {'units': 0, 'moisture': []},
            'Standard': {'units': 0, 'moisture': []},
            'Overcooked': {'units': 0, 'moisture': []},
        }
        self.process_start_time = None  # For dynamic process time

        # Ensure directories exist
        os.makedirs(IMAGE_PATH, exist_ok=True)
        os.makedirs(PROFILE_PATH, exist_ok=True)

        # Load icons
        self.load_icons()

        # Configure root grid
        self.setup_root_grid()

        # Initialize UI
        self.setup_styles()
        self.setup_ui()

    def load_icons(self):
        """Load all icons for buttons with individual try-except."""
        self.start_icon = self._load_icon('start_icon.png', (SIZES['start_stop_size'], SIZES['start_stop_size']))
        self.stop_icon = self._load_icon('stop_icon.png', (SIZES['start_stop_size'], SIZES['start_stop_size']))
        self.small_start_icon = self._load_icon('start_icon.png', (SIZES['small_button_size'], SIZES['small_button_size']))
        self.small_stop_icon = self._load_icon('stop_icon.png', (SIZES['small_button_size'], SIZES['small_button_size']))
        self.camera_icon = self._load_icon('camera_icon.png', (SIZES['small_button_size'], SIZES['small_button_size']))
        self.logo_icon = self._load_icon('logo.png', (SIZES['icon_size'], SIZES['icon_size']))

    def _load_icon(self, filename, size=None, photo_class=ImageTk.PhotoImage):
        """Helper to load a single icon with error handling."""
        try:
            path = os.path.join(IMAGE_PATH, filename)
            if os.path.exists(path):
                img = Image.open(path)
                if size:
                    img = img.resize(size)
                return photo_class(img)
            else:
                print(f"Icon file not found: {filename}")
        except Exception as e:
            print(f"Error loading icon {filename}: {e}")
        return None

    def setup_root_grid(self):
        """Configure the main grid layout for the root window."""
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=0)  # Sidebar column
        self.root.grid_columnconfigure(1, weight=0)  # Separator column
        self.root.grid_columnconfigure(2, weight=1)  # Main frame column

    def setup_styles(self):
        """Configure global styles for widgets."""
        self.root.configure(bg=COLORS['bg_secondary'])
        style = ttk.Style()
        style.configure("Treeview.Heading", font=FONTS['table'])
        style.configure("Treeview", font=FONTS['table'], rowheight=30)
        style.configure("StartStop.TButton", font=FONTS['button'], padding=10)

    def setup_ui(self):
        """Set up the main UI components."""
        self.create_main_frame()
        self.create_separator()
        self.create_sidebar()
        self.set_active_menu(self.show_main_interface, self.menu_frames[0])

    def create_separator(self):
        """Create the vertical separator between sidebar and main frame."""
        self.separator = tk.Frame(self.root, bg='black', width=2)
        self.separator.grid(row=0, column=1, sticky='ns')

    def create_main_frame(self):
        """Create the main frame and its widgets."""
        self.main_frame = tk.Frame(self.root, bg=COLORS['bg_secondary'])
        self.main_frame.grid(row=0, column=2, sticky='nsew')

        # Bind resize for about page
        self.main_frame.bind('<Configure>', self._resize_about_canvas)

        # Create main interface widgets
        self.create_main_widgets()

    def create_main_widgets(self):
        """Create widgets for the main interface."""
        # Start/Stop button [Button]
        self.toggle_button = ttk.Button(
            self.main_frame,
            command=self.toggle_sorting,
            style="StartStop.TButton",
            text="Start" if not self.sorting_running else "Stop",
            compound='center'
        )
        self.update_toggle_button()

        # Camera toggle button [Button]
        self.camera_toggle = tk.Button(
            self.main_frame,
            image=self.camera_icon,
            command=self.toggle_camera_view,
            bg=COLORS['bg_secondary'],
            bd=0
        )

        # Video label frame for fixed height
        self.video_frame = tk.Frame(self.main_frame, height=SIZES['camera_height'], bg='gray')
        self.video_label = tk.Label(
            self.video_frame,
            text="[Camera Feed Placeholder]",
            bg='gray'
        )
        self.video_label.pack(expand=True, fill='both')

        # Status log [Paragraph]
        self.stats_log = tk.Label(
            self.main_frame,
            text="Objects: 0 | Status: Idle",
            font=FONTS['paragraph'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        )

        # Cancel camera button [Button]
        self.cancel_camera_button = tk.Button(
            self.main_frame,
            text="Cancel Camera",
            command=self.toggle_camera_view,
            bg=COLORS['btn_inactive'],
            fg=COLORS['text_primary'],
            font=FONTS['button'],
            width=15,
            height=2
        )

        # Log text area
        self.log_text = scrolledtext.ScrolledText(
            self.main_frame,
            height=10,
            font=FONTS['log'],
            state='disabled'
        )

    def create_sidebar(self):
        """Create the fixed sidebar with logo and title at top, menu items with text below icons."""
        self.sidebar = tk.Frame(self.root, bg=COLORS['bg_primary'], width=SIZES['sidebar_width'])
        self.sidebar.grid(row=0, column=0, sticky='ns')
        self.sidebar.grid_propagate(False)

        # Header frame for logo and title
        self.sidebar_header = tk.Frame(self.sidebar, bg=COLORS['bg_primary'])
        self.sidebar_header.pack(fill='x', pady=SIZES['padding'])

        # Logo placeholder [Image]
        self.logo_label = tk.Label(
            self.sidebar_header,
            image=self.logo_icon,
            text="[Logo]" if not self.logo_icon else "",
            bg=COLORS['bg_primary'],
            fg=COLORS['text_primary'],
            font=FONTS['heading']
        )
        self.logo_label.pack(side='left', padx=SIZES['padding'])
        self.logo_label.image = self.logo_icon

        # Title [Heading]
        tk.Label(
            self.sidebar_header,
            text="ACSS",
            font=FONTS['heading'],
            bg=COLORS['bg_primary'],
            fg=COLORS['text_primary']
        ).pack(side='left', padx=SIZES['padding'])

        # Inner frame for menu items
        self.sidebar_inner = tk.Frame(self.sidebar, bg=COLORS['bg_primary'])
        self.sidebar_inner.pack(fill='both', expand=True)

        # Menu items (top-anchored, excluding Exit)
        menu_items = [
            ('Main Interface', 'main_interface_icon.png', self.show_main_interface),
            ('Statistics', 'statistics_icon.png', self.show_statistics),
            ('Component Testing', 'component_status_icon.png', self.show_component_status),
            ('About', 'about_icon.png', self.show_about),
        ]
        self.menu_frames = []
        for text, icon_file, cmd in menu_items:
            frame = tk.Frame(self.sidebar_inner, bg=COLORS['bg_primary'], relief='ridge', borderwidth=2)
            frame.pack(fill='x', pady=SIZES['padding'], padx=5, anchor='n')
            frame.bind('<Button-1>', lambda e, c=cmd, f=frame: self.set_active_menu(c, f))
            self.menu_frames.append(frame)

            icon = self._load_icon(icon_file, (SIZES['icon_size'], SIZES['icon_size']))

            icon_label = tk.Label(frame, image=icon, bg=COLORS['bg_primary'])
            icon_label.image = icon
            icon_label.pack(pady=(10, 5))
            icon_label.bind('<Button-1>', lambda e, c=cmd, f=frame: self.set_active_menu(c, f))

            # Menu item text [Paragraph]
            text_label = tk.Label(
                frame,
                text=text,
                bg=COLORS['bg_primary'],
                fg=COLORS['text_primary'],
                font=FONTS['paragraph']
            )
            text_label.pack(pady=(0, 5))
            text_label.bind('<Button-1>', lambda e, c=cmd, f=frame: self.set_active_menu(c, f))

        # Exit button (bottom-anchored) [Button]
        exit_frame = tk.Frame(self.sidebar_inner, bg=COLORS['bg_primary'], relief='ridge', borderwidth=2)
        exit_frame.pack(side='bottom', fill='x', pady=SIZES['padding'], padx=5)
        exit_frame.bind('<Button-1>', lambda e, f=exit_frame: self.set_active_menu(self.confirm_exit, f))
        self.menu_frames.append(exit_frame)

        exit_icon = self._load_icon('exit_icon.png', (SIZES['icon_size'], SIZES['icon_size']))

        exit_icon_label = tk.Label(exit_frame, image=exit_icon, bg=COLORS['bg_primary'])
        exit_icon_label.image = exit_icon
        exit_icon_label.pack(pady=(10, 5))
        exit_icon_label.bind('<Button-1>', lambda e, f=exit_frame: self.set_active_menu(self.confirm_exit, f))

        # Exit text [Paragraph]
        exit_text_label = tk.Label(
            exit_frame,
            text="Exit",
            bg=COLORS['bg_primary'],
            fg=COLORS['text_primary'],
            font=FONTS['paragraph']
        )
        exit_text_label.pack(pady=(0, 5))
        exit_text_label.bind('<Button-1>', lambda e, f=exit_frame: self.set_active_menu(self.confirm_exit, f))

    def set_active_menu(self, cmd, frame):
        """Highlight the active sidebar menu item and execute its command."""
        for f in self.menu_frames:
            f.config(bg=COLORS['bg_primary'])
            for child in f.winfo_children():
                child.config(bg=COLORS['bg_primary'])
        frame.config(bg=COLORS['highlight'])
        for child in frame.winfo_children():
            child.config(bg=COLORS['highlight'])
        self.active_menu = cmd
        cmd()

    def update_toggle_button(self):
        """Update the Start/Stop button's appearance."""
        img = (self.small_stop_icon if self.sorting_running else self.small_start_icon) if self.camera_visible else (self.stop_icon if self.sorting_running else self.start_icon)
        text = "Stop" if self.sorting_running else "Start"
        self.toggle_button.config(image=img, text=text)
        self.toggle_button.image = img

    def clear_main_frame(self):
        """Clear all widgets in the main frame."""
        for widget in self.main_frame.winfo_children():
            widget.pack_forget()

    def toggle_fullscreen(self):
        """Toggle fullscreen mode."""
        self.root.attributes('-fullscreen', not self.root.attributes('-fullscreen'))

    def toggle_sorting(self):
        """Toggle the sorting process and update UI."""
        self.sorting_running = not self.sorting_running
        self.update_toggle_button()
        objects = 5 if self.sorting_running else 0
        status = 'Sorting' if self.sorting_running else 'Idle'
        self.stats_log.config(text=f"Objects: {objects} | Status: {status}")
        # Simulate dynamic stats update (in real app, tie to actual events)
        if self.sorting_running:
            import datetime
            self.process_start_time = datetime.datetime.now()
            self.add_log("Sorting started")
        else:
            self.add_log("Sorting stopped")
        # Refresh current view without forcing menu change
        if self.active_menu:
            self.active_menu()

    def toggle_camera_view(self):
        """Toggle the camera view and update UI."""
        self.camera_visible = not self.camera_visible
        self.update_toggle_button()
        self.add_log(f"Camera view {'enabled' if self.camera_visible else 'disabled'}")
        # Refresh current view without forcing menu change
        if self.active_menu:
            self.active_menu()

    def show_main_interface(self):
        """Display the main interface with centered Start/Stop and Camera button below it, or camera view with controls below."""
        self.clear_main_frame()
        # Create a frame to center content
        center_frame = tk.Frame(self.main_frame, bg=COLORS['bg_secondary'])
        center_frame.pack(expand=True, fill='both')
        inner_frame = tk.Frame(center_frame, bg=COLORS['bg_secondary'])
        inner_frame.pack(expand=True)
        if self.camera_visible:
            # Camera view layout (reduced height)
            self.video_frame.pack(in_=inner_frame, pady=(0, SIZES['padding']))
            bottom_frame = tk.Frame(inner_frame, bg=COLORS['bg_secondary'])
            bottom_frame.pack()
            # Status log [Paragraph]
            self.stats_log.pack(in_=bottom_frame, side='left', padx=SIZES['padding'])
            # Start/Stop button [Button]
            self.toggle_button.pack(in_=bottom_frame, side='left', padx=SIZES['padding'])
            # Camera toggle button [Button]
            self.camera_toggle.pack(in_=bottom_frame, side='left', padx=SIZES['padding'])
            # Cancel camera button [Button]
            self.cancel_camera_button.pack(in_=bottom_frame, side='left', padx=SIZES['padding'])
        else:
            # Default layout: Start/Stop centered, Camera below it
            self.toggle_button.pack(in_=inner_frame, pady=20)
            self.camera_toggle.pack(in_=inner_frame, pady=5)

    def show_statistics(self):
        """Display the statistics interface."""
        self.clear_main_frame()
        # Create a frame to center content
        center_frame = tk.Frame(self.main_frame, bg=COLORS['bg_secondary'])
        center_frame.pack(expand=True, fill='both')
        # Heading
        tk.Label(
            center_frame,
            text="Statistics",
            font=FONTS['heading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=SIZES['padding'])

        # Calculate dynamic stats
        overall_units = sum(cat['units'] for cat in self.stats_data.values())
        overall_moisture = sum(sum(cat['moisture']) for cat in self.stats_data.values()) / overall_units if overall_units else 0
        tree_data = []
        for category, data in self.stats_data.items():
            avg_moisture = sum(data['moisture']) / data['units'] if data['units'] else 0
            tree_data.append((category, data['units'], round(avg_moisture, 1)))
        tree_data.append(('Overall', overall_units, round(overall_moisture, 1)))

        # Table
        tree = ttk.Treeview(center_frame, columns=('Category', 'Units', 'Avg Moisture'), show='headings', height=6)
        tree.heading('Category', text='Category')
        tree.heading('Units', text='Units')
        tree.heading('Avg Moisture', text='Avg Moisture (%)')
        tree.column('Category', width=200, anchor='center')
        tree.column('Units', width=100, anchor='center')
        tree.column('Avg Moisture', width=150, anchor='center')
        for values in tree_data:
            tree.insert('', 'end', values=values)
        tree.pack(pady=SIZES['padding'], fill='x', padx=SIZES['padding'])

        # Paragraph with dynamic time
        if self.process_start_time:
            import datetime
            duration = datetime.datetime.now() - self.process_start_time
            hours, remainder = divmod(duration.seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            process_time = f"{hours}:{minutes:02d}:{seconds:02d}"
            started = self.process_start_time.strftime("%Y-%m-%d %H:%M:%S")
        else:
            process_time = "0:00:00"
            started = "N/A"
        tk.Label(
            center_frame,
            text=f"Process Time: {process_time}, Started: {started}",
            font=FONTS['paragraph'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5)

        # Button
        tk.Button(
            center_frame,
            text="Export to PDF",
            command=self.export_pdf,
            width=15,
            height=2,
            bg=COLORS['accent'],
            fg=COLORS['text_primary'],
            font=FONTS['button']
        ).pack(pady=SIZES['padding'])

    def export_pdf(self):
        """Export statistics to PDF."""
        # Gather dynamic data
        data = []
        for category, cat_data in self.stats_data.items():
            avg_moisture = sum(cat_data['moisture']) / cat_data['units'] if cat_data['units'] else 0
            data.append([category, cat_data['units'], round(avg_moisture, 1)])
        overall_units = sum(row[1] for row in data)
        overall_moisture = sum(row[1] * row[2] for row in data) / overall_units if overall_units else 0
        data.append(['Overall', overall_units, round(overall_moisture, 1)])

        if self.process_start_time:
            import datetime
            duration = datetime.datetime.now() - self.process_start_time
            hours, remainder = divmod(duration.seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            process_time = f"{hours}:{minutes:02d}:{seconds:02d}"
            started = self.process_start_time.strftime("%Y-%m-%d %H:%M:%S")
        else:
            process_time = "0:00:00"
            started = "N/A"

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.axis('off')
        fig.text(0.5, 0.95, 'ACSS Statistics Receipt', ha='center', fontsize=16, fontweight='bold')
        col_labels = ['Category', 'Units', 'Avg Moisture (%)']
        table = ax.table(cellText=data, colLabels=col_labels, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 1.2)
        fig.text(0.5, 0.05, f'Process Time: {process_time} | Started: {started}\n© 2025 Techlaro Company', ha='center', fontsize=10)
        pdf_path = 'stats.pdf'
        fig.savefig(pdf_path, format='pdf', bbox_inches='tight')
        plt.close(fig)
        # Cross-platform open
        webbrowser.open(f'file://{os.path.abspath(pdf_path)}')

    def show_component_status(self):
        """Display the component status interface."""
        self.clear_main_frame()
        # Create a frame to center content
        center_frame = tk.Frame(self.main_frame, bg=COLORS['bg_secondary'])
        center_frame.pack(expand=True, fill='both')
        # Heading
        tk.Label(
            center_frame,
            text="Component Testing",
            font=FONTS['heading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=SIZES['padding'])

        components = [
            ("Servo Motor", [
                ("Left (a)", lambda: self._component_action("Servo: Moved to left")),
                ("Neutral (w)", lambda: self._component_action("Servo: Moved to neutral")),
                ("Right (d)", lambda: self._component_action("Servo: Moved to right")),
            ]),
            ("DC Motor", [
                ("Start/Stop (s)", self.toggle_motor),
            ]),
            ("NIR Sensor", [
                ("Read NIR", lambda: self._component_action("NIR: Simulated reading")),
            ]),
            ("Proximity Sensor", [
                ("Check Proximity", lambda: self._component_action("Proximity: Detected")),
            ]),
            ("Camera View", [
                ("Toggle Camera (c)", self.toggle_camera_view),
            ]),
        ]

        # Add key bindings
        self.root.bind('<Key-a>', lambda e: self._component_action("Servo: Moved to left"))
        self.root.bind('<Key-w>', lambda e: self._component_action("Servo: Moved to neutral"))
        self.root.bind('<Key-d>', lambda e: self._component_action("Servo: Moved to right"))
        self.root.bind('<Key-s>', lambda e: self.toggle_motor())
        self.root.bind('<Key-c>', lambda e: self.toggle_camera_view())

        for title, buttons in components:
            # Subheading
            tk.Label(
                center_frame,
                text=title,
                font=FONTS['subheading'],
                bg=COLORS['bg_secondary'],
                fg=COLORS['text_secondary']
            ).pack(pady=5)
            frame = tk.Frame(center_frame, bg=COLORS['bg_secondary'])
            frame.pack(pady=5)
            for text, cmd in buttons:
                # Button
                tk.Button(
                    frame,
                    text=text,
                    command=cmd,
                    width=15,
                    height=2,
                    font=FONTS['button'],
                    bg=COLORS['btn_active'],
                    fg=COLORS['text_primary']
                ).pack(side='left', padx=5)
            tk.Frame(center_frame, height=2, bg='black').pack(fill='x', pady=5)

        frame = tk.Frame(center_frame, bg=COLORS['bg_secondary'])
        frame.pack(pady=5)
        # Button
        tk.Button(
            frame,
            text="Verify All",
            command=lambda: self._component_action("Verify Results: All components OK"),
            width=15,
            height=2,
            font=FONTS['button'],
            bg=COLORS['btn_active'],
            fg=COLORS['text_primary']
        ).pack(side='left', padx=5)
        # Button
        tk.Button(
            frame,
            text="Show/Hide Log",
            command=self.toggle_log,
            width=15,
            height=2,
            font=FONTS['button'],
            bg=COLORS['btn_active'],
            fg=COLORS['text_primary']
        ).pack(side='left', padx=5)

        # Paragraph
        self.stats_log.pack(in_=center_frame, pady=SIZES['padding'])

    def show_about(self):
        """Display the scrollable about interface with centered widgets."""
        self.clear_main_frame()
        # Create canvas for scrolling
        self.about_canvas = tk.Canvas(self.main_frame, bg=COLORS['bg_secondary'])
        self.about_scrollbar = tk.Scrollbar(self.main_frame, orient="vertical", command=self.about_canvas.yview)
        self.about_inner = tk.Frame(self.about_canvas, bg=COLORS['bg_secondary'])

        self.about_inner.bind("<Configure>", lambda e: self.about_canvas.configure(scrollregion=self.about_canvas.bbox("all")))
        self.about_canvas.create_window((0, 0), window=self.about_inner, anchor="n")
        self.about_canvas.configure(yscrollcommand=self.about_scrollbar.set)

        self.about_canvas.pack(side='left', fill='both', expand=True)
        self.about_scrollbar.pack(side='right', fill='y')

        # Heading
        tk.Label(
            self.about_inner,
            text="About Us - Techlaro Company",
            font=FONTS['heading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=SIZES['padding'])

        # Company Overview
        overview_frame = tk.Frame(self.about_inner, bg=COLORS['bg_secondary'], width=800)
        overview_frame.pack(pady=10)
        overview_frame.pack_propagate(False)

        # Subheading
        tk.Label(
            overview_frame,
            text="Our Company",
            font=FONTS['subheading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')
        # Paragraph
        tk.Label(
            overview_frame,
            text="Techlaro Company is a dynamic and innovative technology firm dedicated to providing cutting-edge solutions to meet the evolving needs of our clients. Founded on the principles of collaboration, expertise, and a passion for technology, we strive to deliver high-quality services and products that empower businesses and individuals alike. Our team of skilled professionals brings together a wealth of experience in various aspects of software development and database management.",
            font=FONTS['paragraph'],
            wraplength=780,
            justify='left',
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')

        # Subheading
        tk.Label(
            overview_frame,
            text="Our Mission",
            font=FONTS['subheading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')
        # Paragraph
        tk.Label(
            overview_frame,
            text="To empower our clients with robust and scalable technology solutions through collaboration, innovation, and unwavering commitment to excellence.",
            font=FONTS['paragraph'],
            wraplength=780,
            justify='left',
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')

        # Subheading
        tk.Label(
            overview_frame,
            text="Our Vision",
            font=FONTS['subheading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')
        # Paragraph
        tk.Label(
            overview_frame,
            text="To be a leading technology company recognized for its expertise, customer-centric approach, and contribution to technological advancement.",
            font=FONTS['paragraph'],
            wraplength=780,
            justify='left',
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')

        # Subheading
        tk.Label(
            overview_frame,
            text="Our Core Values",
            font=FONTS['subheading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=5, anchor='w')
        values_frame = tk.Frame(overview_frame, bg=COLORS['bg_secondary'])
        values_frame.pack(pady=5, anchor='w')
        core_values = [
            ("Collaboration:", "We believe in the power of teamwork and open communication."),
            ("Innovation:", "We are committed to exploring and implementing the latest technologies."),
            ("Excellence:", "We strive for the highest standards in everything we do."),
            ("Integrity:", "We conduct our business with honesty and transparency."),
            ("Customer Focus:", "Our clients' success is our top priority."),
        ]
        for title, desc in core_values:
            frame = tk.Frame(values_frame, bg=COLORS['bg_secondary'])
            frame.pack(pady=2, anchor='w')
            # Paragraph (Bold)
            tk.Label(
                frame,
                text=title,
                font=FONTS['paragraph_bold'],
                bg=COLORS['bg_secondary'],
                fg=COLORS['text_secondary']
            ).pack(side='left')
            # Paragraph
            tk.Label(
                frame,
                text=desc,
                font=FONTS['paragraph'],
                bg=COLORS['bg_secondary'],
                fg=COLORS['text_secondary']
            ).pack(side='left', padx=5)

        # Subheading
        tk.Label(
            self.about_inner,
            text="Our Team",
            font=FONTS['subheading'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        ).pack(pady=SIZES['padding'])

        team_frame = tk.Frame(self.about_inner, bg=COLORS['bg_secondary'])
        team_frame.pack(pady=10)
        team_frame.grid_columnconfigure((0, 1), weight=1)
        team_frame.grid_rowconfigure((0, 1), weight=1)

        members = [
            ("Christian L. Narvaez", "Full-Stack Developer", "christian.png"),
            ("John Paul F. Armenta", "Back-end Developer", "johnpaul.png"),
            ("Jerald James D. Preclaro", "Front-end Developer", "jerald.png"),
            ("Marielle B. Maming", "Database Administrator", "marielle.png"),
        ]
        for idx, (name, pos, img_file) in enumerate(members):
            row = idx // 2
            col = idx % 2
            frame = tk.Frame(team_frame, bg=COLORS['bg_secondary'])
            frame.grid(row=row, column=col, padx=20, pady=10)
            photo = self._load_profile_image(img_file)
            photo_label = tk.Label(frame, image=photo, text="[Photo]" if not photo else "", bg='gray')
            photo_label.image = photo
            photo_label.pack()
            # Subheading
            tk.Label(
                frame,
                text=name,
                font=FONTS['subheading'],
                fg=COLORS['text_secondary']
            ).pack(pady=2)
            # Paragraph
            tk.Label(
                frame,
                text=pos,
                font=FONTS['paragraph'],
                fg=COLORS['text_secondary']
            ).pack()

        # Footer [Paragraph]
        tk.Label(
            self.about_inner,
            text="© 2025 Techlaro Company",
            font=FONTS['log'],
            bg='gray',
            fg=COLORS['text_primary']
        ).pack(fill='x', pady=SIZES['padding'])

    def _load_profile_image(self, filename):
        """Load profile image with error handling."""
        try:
            path = os.path.join(PROFILE_PATH, filename)
            if os.path.exists(path):
                img = Image.open(path).resize((SIZES['photo_size'], SIZES['photo_size']))
                return ImageTk.PhotoImage(img)
            else:
                print(f"Profile image not found: {filename}")
        except Exception as e:
            print(f"Error loading profile {filename}: {e}")
        return None

    def _resize_about_canvas(self, event=None):
        """Resize about canvas on main_frame configure."""
        if hasattr(self, 'about_canvas') and self.about_canvas.winfo_exists():
            canvas_width = self.main_frame.winfo_width() - self.about_scrollbar.winfo_width()
            self.about_canvas.itemconfig(self.about_canvas.find_withtag("window"), width=canvas_width)

    def toggle_log(self):
        """Toggle the log display."""
        self.log_visible = not self.log_visible
        if self.log_visible:
            self.log_text.pack(fill='both', expand=True, padx=SIZES['padding'], pady=SIZES['padding'])
            self.update_log_display()
        else:
            self.log_text.pack_forget()

    def add_log(self, entry):
        """Add a log entry with timestamp."""
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_entries.append(f"{timestamp} | default_user | {entry}")
        if self.log_visible:
            self.update_log_display()

    def update_log_display(self):
        """Update the log text widget with current entries."""
        self.log_text.config(state='normal')
        self.log_text.delete(1.0, tk.END)
        for entry in self.log_entries[-10:]:  # Show last 10 for brevity
            self.log_text.insert(tk.END, f"{entry}\n")
        self.log_text.config(state='disabled')

    def toggle_motor(self):
        """Toggle the motor status."""
        message = "Motor: Toggled on/off"
        self._component_action(message)

    def _component_action(self, message):
        """Helper for component actions: update stats and log."""
        self.stats_log.config(text=message)
        self.add_log(message)

    def confirm_exit(self):
        """Show confirmation dialog before exiting."""
        # Create a larger confirmation dialog
        top = tk.Toplevel(self.root)
        top.title("Confirm Exit")
        width = 400
        height = 200
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2
        top.geometry(f"{width}x{height}+{x}+{y}")
        top.transient(self.root)
        top.grab_set()
        tk.Label(
            top,
            text="Are you sure you want to exit?",
            font=FONTS['confirm'],
            wraplength=350,
            justify='center'
        ).pack(expand=True, pady=20)
        button_frame = tk.Frame(top)
        button_frame.pack(pady=10)
        tk.Button(
            button_frame,
            text="Yes",
            command=lambda: [top.destroy(), self.shutdown_app()],
            font=FONTS['button'],
            bg=COLORS['btn_active'],
            fg=COLORS['text_primary'],
            width=10
        ).pack(side='left', padx=10)
        tk.Button(
            button_frame,
            text="No",
            command=lambda: [top.destroy(), self.set_active_menu(self.show_main_interface, self.menu_frames[0])],
            font=FONTS['button'],
            bg=COLORS['btn_inactive'],
            fg=COLORS['text_primary'],
            width=10
        ).pack(side='left', padx=10)

    def shutdown_app(self):
        """Close the application."""
        # Unbind keys
        self.root.unbind('<Key-a>')
        self.root.unbind('<Key-w>')
        self.root.unbind('<Key-d>')
        self.root.unbind('<Key-s>')
        self.root.unbind('<Key-c>')
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = ACSS_App(root)
    root.protocol("WM_DELETE_WINDOW", app.confirm_exit)
    root.mainloop()