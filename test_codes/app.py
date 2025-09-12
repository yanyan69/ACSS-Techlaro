import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
import cv2, datetime
from PIL import Image, ImageTk
import os, traceback
from constants import COLORS, FONTS, SIZES, IMAGE_PATH, PROFILE_PATH
from utils import load_icons, add_log, export_pdf, _component_action, _load_icon
from views import show_main_interface, show_statistics, show_component_status, show_about, toggle_log, toggle_motor
from hardware import HardwareInterface

class ACSS_App:
    def __init__(self, root):
        try:
            add_log(self, "Starting ACSS_App initialization")
            self.root = root
            self.root.title('Automated Copra Segregation System (PC Version)')
            self.root.geometry('1024x600')
            self.root.attributes('-fullscreen', True)
            add_log(self, "Binding keys")
            self.root.bind('<Key-f>', lambda e: self.toggle_fullscreen())
            self.root.bind('<Key-space>', lambda e: self.toggle_sorting())
            self.root.bind('<Key-Escape>', lambda e: self.show_component_status())
            self.root.bind('<Key-a>', lambda e: self.hardware.safe_send("SERVO_TEST_0") if self.hardware and self.hardware.arduino else None)
            self.root.bind('<Key-w>', lambda e: self.hardware.safe_send("SERVO_TEST_90") if self.hardware and self.hardware.arduino else None)
            self.root.bind('<Key-d>', lambda e: self.hardware.safe_send("SERVO_TEST_180") if self.hardware and self.hardware.arduino else None)
            self.root.bind('<Key-s>', lambda e: self.toggle_motor())
            self.root.bind('<Key-c>', lambda e: self.toggle_camera_view())
            self.camera_visible = True
            self.sorting_running = False
            self.log_visible = False
            self.active_menu = None
            self.log_entries = []
            self.stats_data = {
                'Raw': {'units': 0, 'moisture': []},
                'Standard': {'units': 0, 'moisture': []},
                'Overcooked': {'units': 0, 'moisture': []},
            }
            self.process_start_time = None
            self.current_user = "default_user"
            self.hardware = None
            self.start_icon = None
            self.small_start_icon = None
            self.stop_icon = None
            self.small_stop_icon = None
            self.camera_icon = None
            self.logo_icon = None

            # Load icons first
            add_log(self, "Loading icons")
            try:
                load_icons(self)
                add_log(self, f"Icon attributes: start_icon={hasattr(self, 'start_icon')}, "
                              f"small_start_icon={hasattr(self, 'small_start_icon')}, "
                              f"stop_icon={hasattr(self, 'stop_icon')}, "
                              f"small_stop_icon={hasattr(self, 'small_stop_icon')}, "
                              f"camera_icon={hasattr(self, 'camera_icon')}, "
                              f"logo_icon={hasattr(self, 'logo_icon')}")
            except Exception as e:
                messagebox.showwarning("Icon Loading Warning", f"Failed to load icons: {str(e)}. Using text fallbacks.")
                add_log(self, f"Icon loading failed: {str(e)}")

            # Ensure directories exist
            add_log(self, f"Creating directories: IMAGE_PATH={IMAGE_PATH}, PROFILE_PATH={PROFILE_PATH}")
            try:
                os.makedirs(IMAGE_PATH, exist_ok=True)
                os.makedirs(PROFILE_PATH, exist_ok=True)
            except Exception as e:
                messagebox.showwarning("Directory Creation Warning", f"Failed to create directories: {str(e)}")
                add_log(self, f"Directory creation failed: {str(e)}")

            # Initialize hardware interface
            add_log(self, "Initializing hardware")
            try:
                self.hardware = HardwareInterface(self)
                self.update_hardware_status()
            except Exception as e:
                messagebox.showwarning("Initialization Warning", f"Failed to initialize hardware: {str(e)}. Continuing without hardware.")
                add_log(self, f"Hardware initialization failed: {str(e)}")
                self.hardware = None

            # Initialize UI (after icons and hardware)
            add_log(self, "Setting up UI")
            self.setup_ui()

            # Start camera feed if available
            add_log(self, "Checking camera feed setup")
            try:
                if self.hardware and self.hardware.cap:
                    add_log(self, "Starting camera feed")
                    self.root.after(100, self.update_camera_feed)
                else:
                    add_log(self, "No camera available, skipping feed")
            except Exception as e:
                add_log(self, f"Camera feed setup error: {str(e)}")

            # Start Arduino listener if connected
            add_log(self, "Checking Arduino listener setup")
            try:
                if self.hardware and self.hardware.arduino:
                    add_log(self, "Starting Arduino listener")
                    self.root.after(100, self.listen_to_arduino)
                else:
                    add_log(self, "No Arduino connected, skipping listener")
            except Exception as e:
                add_log(self, f"Arduino listener setup error: {str(e)}")

            add_log(self, "ACSS_App initialization completed")
        except Exception as e:
            add_log(self, f"Initialization error: {str(e)}\n{traceback.format_exc()}")
            messagebox.showerror("Initialization Error", f"Application failed to initialize: {str(e)}")
            self.shutdown_app()
            return

    def update_hardware_status(self):
        """Update UI based on hardware status."""
        try:
            if hasattr(self, 'status_label') and self.status_label.winfo_exists():
                if not self.hardware:
                    self.status_label.config(text="Status: No hardware available")
                elif not self.hardware.arduino:
                    self.status_label.config(text="Status: No Arduino connected")
                else:
                    self.status_label.config(text=f"Status: Arduino connected on {self.hardware.arduino.port}")
            else:
                add_log(self, "Status label not available for update")
        except Exception as e:
            add_log(self, f"Update hardware status error: {str(e)}")

    def setup_root_grid(self):
        """Configure the root window's grid layout with adaptive sidebar."""
        try:
            add_log(self, "Setting up root grid")
            self.root.grid_rowconfigure(0, weight=1)
            self.root.grid_columnconfigure(0, weight=0)
            self.root.grid_columnconfigure(1, weight=0)
            self.root.grid_columnconfigure(2, weight=1)
        except Exception as e:
            add_log(self, f"Root grid setup error: {str(e)}")
            raise

    def setup_styles(self):
        """Configure global styles for widgets."""
        try:
            add_log(self, "Configuring styles")
            self.root.configure(bg=COLORS['bg_secondary'])
            style = ttk.Style()
            style.configure("Treeview.Heading", font=FONTS['table'])
            style.configure("Treeview", font=FONTS['table'], rowheight=30)
            style.configure("StartStop.TButton", font=FONTS['button'])
        except Exception as e:
            add_log(self, f"Style configuration error: {str(e)}")
            raise

    def setup_ui(self):
        """Set up the main UI components."""
        try:
            add_log(self, "Creating main frame")
            self.create_main_frame()
            add_log(self, "Creating separator")
            self.create_separator()
            add_log(self, "Creating sidebar")
            self.create_sidebar()
            add_log(self, "Setting active menu")
            self.set_active_menu(self.show_main_interface, self.menu_frames[0])
        except Exception as e:
            add_log(self, f"UI setup error: {str(e)}")
            raise

    def create_separator(self):
        """Create the vertical separator between sidebar and main frame."""
        try:
            self.separator = tk.Frame(self.root, bg='black', width=2)
            self.separator.grid(row=0, column=1)
        except Exception as e:
            add_log(self, f"Separator creation error: {str(e)}")
            raise

    def create_main_frame(self):
        """Create the main frame (no widgets here; created in views)."""
        try:
            self.main_frame = tk.Frame(self.root, bg=COLORS['bg_secondary'])
            self.main_frame.grid(row=0, column=2)
        except Exception as e:
            add_log(self, f"Main frame creation error: {str(e)}")
            raise

    def create_sidebar(self):
        """Create the sidebar with logo and toggle button in a header, menu items top-anchored, and Exit button bottom-anchored."""
        try:
            self.sidebar = tk.Frame(self.root, bg=COLORS['bg_primary'], width=SIZES['sidebar_width'])
            self.sidebar.grid(row=0, column=0)
            self.sidebar.grid_propagate(False)

            # Header frame for logo and toggle button
            self.sidebar_header = tk.Frame(self.sidebar, bg=COLORS['bg_primary'])
            self.sidebar_header.pack()

            # Logo placeholder [Image]
            self.logo_label = tk.Label(
                self.sidebar_header,
                image=self.logo_icon,
                text="[Logo]" if not self.logo_icon else "",
                bg=COLORS['bg_primary'],
                fg=COLORS['text_primary'],
                font=FONTS['heading']
            )
            self.logo_label.pack()
            self.logo_label.image = self.logo_icon

            # Sidebar toggle button [Button]
            self.sidebar_toggle_btn = tk.Button(
                self.sidebar_header,
                image=self.sidebar_icon if hasattr(self, 'sidebar_icon') else None,
                text="Toggle" if not hasattr(self, 'sidebar_icon') else "",
                command=self.toggle_sidebar,
                bg=COLORS['bg_primary'],
                bd=0
            )
            self.sidebar_toggle_btn.pack()
            self.sidebar_toggle_btn.image = self.sidebar_icon if hasattr(self, 'sidebar_icon') else None

            # Inner frame for menu items
            self.sidebar_inner = tk.Frame(self.sidebar, bg=COLORS['bg_primary'])
            self.sidebar_inner.pack()

            # Menu items
            menu_items = [
                ("Main Interface", self.show_main_interface),
                ("Statistics", self.show_statistics),
                ("Component Status", self.show_component_status),
                ("About", self.show_about),
            ]
            self.menu_frames = []
            for text, func in menu_items:
                frame = tk.Frame(self.sidebar_inner, bg=COLORS['bg_primary'])
                frame.pack()
                self.menu_frames.append(frame)
                btn = tk.Button(
                    frame,
                    text=text,
                    command=lambda f=func, fr=frame: self.set_active_menu(f, fr),
                    bg=COLORS['bg_primary'],
                    fg=COLORS['text_primary'],
                    font=FONTS['button'],
                    bd=0
                )
                btn.pack()

            # Exit button
            exit_frame = tk.Frame(self.sidebar, bg=COLORS['bg_primary'])
            exit_frame.pack()
            tk.Button(
                exit_frame,
                text="Exit",
                command=self.confirm_exit,
                bg=COLORS['btn_inactive'],
                fg=COLORS['text_primary'],
                font=FONTS['button'],
                bd=0
            ).pack()
        except Exception as e:
            add_log(self, f"Sidebar creation error: {str(e)}")
            raise

    def toggle_sidebar(self):
        """Toggle sidebar visibility."""
        try:
            self.sidebar_expanded = not self.sidebar_expanded
            width = SIZES['sidebar_width'] if self.sidebar_expanded else SIZES['sidebar_width'] // 2
            self.sidebar.config(width=width)
            add_log(self, "Toggled sidebar")
        except Exception as e:
            add_log(self, f"Toggle sidebar error: {str(e)}")

    def set_active_menu(self, func, frame):
        """Set the active menu and highlight it."""
        try:
            if self.active_menu:
                self.active_menu_frame.config(bg=COLORS['bg_primary'])
            self.active_menu = func
            self.active_menu_frame = frame
            self.active_menu_frame.config(bg=COLORS['highlight'])
            func()
        except Exception as e:
            add_log(self, f"Set active menu error: {str(e)}")
            raise

    def clear_main_frame(self):
        """Clear the main frame by destroying its children."""
        try:
            for widget in self.main_frame.winfo_children():
                widget.destroy()
        except Exception as e:
            add_log(self, f"Clear main frame error: {str(e)}")

    def toggle_fullscreen(self):
        """Toggle fullscreen mode."""
        try:
            self.root.attributes('-fullscreen', not self.root.attributes('-fullscreen'))
            add_log(self, "Toggled fullscreen")
        except Exception as e:
            add_log(self, f"Toggle fullscreen error: {str(e)}")

    def toggle_sorting(self):
        """Toggle the sorting process and update UI."""
        try:
            if not self.hardware or not self.hardware.arduino:
                messagebox.showwarning("Hardware Error", "Cannot start sorting: Arduino not connected")
                add_log(self, "Sorting blocked: Arduino not connected")
                return
            self.sorting_running = not self.sorting_running
            if self.sorting_running:
                self.hardware.start_detection()
                self.process_start_time = datetime.datetime.now()
                add_log(self, "Sorting started")
            else:
                self.hardware.stop_detection()
                add_log(self, "Sorting stopped")
            objects = self.hardware.processed_image_count if self.hardware else 0
            status = 'Sorting' if self.sorting_running else 'Idle'
            if hasattr(self, 'stats_log') and self.stats_log.winfo_exists():
                self.stats_log.config(text=f"Objects: {objects} | Status: {status}")
            if self.active_menu:
                self.active_menu()
        except Exception as e:
            messagebox.showerror("Sorting Error", f"Failed to toggle sorting: {str(e)}")
            add_log(self, f"Sorting error: {str(e)}")

    def toggle_camera_view(self):
        """Toggle the camera view and update UI."""
        try:
            if not self.hardware or not self.hardware.cap:
                messagebox.showwarning("Hardware Error", "Cannot toggle camera: No camera available")
                add_log(self, "Camera toggle blocked: No camera available")
                return
            self.camera_visible = not self.camera_visible
            add_log(self, f"Camera view {'enabled' if self.camera_visible else 'disabled'}")
            if self.active_menu:
                self.active_menu()
        except Exception as e:
            messagebox.showerror("Camera View Error", f"Failed to toggle camera view: {str(e)}")
            add_log(self, f"Camera view error: {str(e)}")

    def update_camera_feed(self):
        """Update the camera feed in the video_label."""
        try:
            if self.camera_visible and self.hardware and self.hardware.cap and hasattr(self, 'video_label') and self.video_label.winfo_exists():
                ret, frame = self.hardware.cap.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(frame).resize((self.main_frame.winfo_width() // 2, SIZES['camera_height']))
                    photo = ImageTk.PhotoImage(img)
                    self.video_label.config(image=photo)
                    self.video_label.image = photo
                self.root.after(100, self.update_camera_feed)
            else:
                add_log(self, "Camera feed skipped: Camera not visible or hardware unavailable")
                self.root.after(100, self.update_camera_feed)
        except Exception as e:
            add_log(self, f"Camera feed error: {str(e)}")
            self.camera_visible = False
            if self.active_menu:
                self.active_menu()

    def listen_to_arduino(self):
        """Listen for Arduino messages and process copra if detected."""
        try:
            if self.sorting_running and self.hardware and self.hardware.arduino:
                response = self.hardware.read_serial()
                if response == "DETECTED":
                    category = self.hardware.process_copra()
                    self.hardware.send_command(f"SERVO_{category.upper()}")
                    objects = self.hardware.processed_image_count
                    if hasattr(self, 'stats_log') and self.stats_log.winfo_exists():
                        self.stats_log.config(text=f"Objects: {objects} | Status: Sorting")
                    add_log(self, f"Processed copra: {category}")
            self.root.after(100, self.listen_to_arduino)
        except Exception as e:
            add_log(self, f"Arduino listener error: {str(e)}")
            self.root.after(100, self.listen_to_arduino)

    def show_main_interface(self):
        try:
            add_log(self, "Showing main interface")
            show_main_interface(self)
        except Exception as e:
            messagebox.showerror("Main Interface Error", f"Failed to show main interface: {str(e)}")
            add_log(self, f"Main interface error: {str(e)}")

    def show_statistics(self):
        try:
            show_statistics(self)
        except Exception as e:
            messagebox.showerror("Statistics Error", f"Failed to show statistics: {str(e)}")
            add_log(self, f"Statistics error: {str(e)}")

    def show_component_status(self):
        try:
            show_component_status(self)
        except Exception as e:
            messagebox.showerror("Component Status Error", f"Failed to show component status: {str(e)}")
            add_log(self, f"Component status error: {str(e)}")

    def show_about(self):
        try:
            show_about(self)
        except Exception as e:
            messagebox.showerror("About Error", f"Failed to show about page: {str(e)}")
            add_log(self, f"About page error: {str(e)}")

    def toggle_log(self):
        try:
            toggle_log(self)
        except Exception as e:
            messagebox.showerror("Log Error", f"Failed to toggle log: {str(e)}")
            add_log(self, f"Log toggle error: {str(e)}")

    def toggle_motor(self):
        try:
            if self.hardware and self.hardware.arduino:
                self.hardware.toggle_motor()
            else:
                messagebox.showwarning("Hardware Error", "Cannot toggle motor: Arduino not connected")
                add_log(self, "Motor toggle blocked: Arduino not connected")
        except Exception as e:
            messagebox.showerror("Motor Error", f"Failed to toggle motor: {str(e)}")
            add_log(self, f"Motor toggle error: {str(e)}")

    def export_pdf(self):
        try:
            export_pdf(self)
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export PDF: {str(e)}")
            add_log(self, f"Export PDF error: {str(e)}")

    def _component_action(self, message):
        try:
            _component_action(self, message)
        except Exception as e:
            add_log(self, f"Component action error: {str(e)}")

    def confirm_exit(self):
        """Show confirmation dialog before exiting."""
        try:
            top = tk.Toplevel(self.root)
            top.title("Confirm Exit")
            tk.Label(
                top,
                text="Are you sure you want to exit?",
                font=FONTS['confirm']
            ).pack()
            button_frame = tk.Frame(top)
            button_frame.pack()
            tk.Button(
                button_frame,
                text="Yes",
                command=lambda: [top.destroy(), self.shutdown_app()],
                font=FONTS['button'],
                bg=COLORS['btn_active'],
                fg=COLORS['text_primary']
            ).pack()
            tk.Button(
                button_frame,
                text="No",
                command=lambda: [top.destroy(), self.set_active_menu(self.active_menu, self.active_menu_frame)],
                font=FONTS['button'],
                bg=COLORS['btn_inactive'],
                fg=COLORS['text_primary']
            ).pack()
        except Exception as e:
            add_log(self, f"Confirm exit error: {str(e)}")

    def shutdown_app(self):
        """Close the application."""
        try:
            add_log(self, "Shutting down application")
            if self.hardware:
                self.hardware.shutdown()
            self.root.unbind('<Key-a>')
            self.root.unbind('<Key-w>')
            self.root.unbind('<Key-d>')
            self.root.unbind('<Key-s>')
            self.root.unbind('<Key-c>')
            self.root.destroy()
        except Exception as e:
            add_log(self, f"Shutdown error: {str(e)}")

if __name__ == '__main__':
    root = tk.Tk()
    try:
        app = ACSS_App(root)
        try:
            root.protocol("WM_DELETE_WINDOW", app.confirm_exit)
        except Exception as e:
            add_log(app, f"Window protocol error: {str(e)}")
        root.mainloop()
    except Exception as e:
        if 'app' in locals():
            add_log(app, f"Application crashed: {str(e)}\n{traceback.format_exc()}")
            messagebox.showerror("Application Error", f"Application crashed: {str(e)}\n{traceback.format_exc()}")
        try:
            root.destroy()
        except:
            pass