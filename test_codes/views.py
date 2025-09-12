import tkinter as tk
from tkinter import ttk, scrolledtext
from constants import COLORS, FONTS, SIZES
from utils import export_pdf, _component_action, _load_profile_image, add_log

def show_main_interface(app):
    """Display the main interface with camera feed and controls, matching ACSS-gui-test.py layout."""
    try:
        add_log(app, "Displaying main interface")
        app.clear_main_frame()

        # Main container for vertical layout
        main_container = tk.Frame(app.main_frame, bg=COLORS['bg_secondary'])
        main_container.pack(fill='both', expand=True)

        # Start/Stop button
        toggle_button_config = {
            'command': app.toggle_sorting,
            'style': "StartStop.TButton",
            'text': "Start" if not app.sorting_running else "Stop",
            'compound': 'center'
        }
        if hasattr(app, 'start_icon') and app.start_icon and not app.sorting_running:
            toggle_button_config['image'] = app.start_icon
        elif hasattr(app, 'stop_icon') and app.stop_icon and app.sorting_running:
            toggle_button_config['image'] = app.stop_icon
        app.toggle_button = ttk.Button(main_container, **toggle_button_config)
        app.toggle_button.pack(pady=SIZES['padding'], anchor='center')

        # Camera toggle or cancel button
        if app.camera_visible:
            app.video_label = tk.Label(
                main_container,
                text="[Camera Feed Placeholder]",
                bg='gray',
                height=SIZES['camera_height']
            )
            app.video_label.pack(fill='x', pady=SIZES['padding'], padx=SIZES['padding'])

            app.cancel_camera_button = tk.Button(
                main_container,
                text="Cancel Camera",
                command=app.toggle_camera_view,
                bg=COLORS['btn_inactive'],
                fg=COLORS['text_primary'],
                font=FONTS['button'],
                width=15,
                height=2
            )
            app.cancel_camera_button.pack(pady=SIZES['padding'], anchor='center')
        else:
            camera_toggle_config = {
                'command': app.toggle_camera_view,
                'bg': COLORS['bg_secondary'],
                'bd': 0
            }
            if hasattr(app, 'camera_icon') and app.camera_icon:
                camera_toggle_config['image'] = app.camera_icon
            else:
                camera_toggle_config['text'] = "Camera"
            app.camera_toggle = tk.Button(main_container, **camera_toggle_config)
            app.camera_toggle.pack(pady=SIZES['padding'], anchor='center')

        # Button frame for Log and Motor
        button_frame = tk.Frame(main_container, bg=COLORS['bg_secondary'])
        button_frame.pack(pady=SIZES['padding'], anchor='center')
        tk.Button(
            button_frame,
            text="Toggle Log",
            command=app.toggle_log,
            bg=COLORS['accent'],
            fg=COLORS['text_primary'],
            font=FONTS['button']
        ).pack(side='left', padx=5)
        tk.Button(
            button_frame,
            text="Toggle Motor",
            command=app.toggle_motor,
            bg=COLORS['btn_active'],
            fg=COLORS['text_primary'],
            font=FONTS['button']
        ).pack(side='left', padx=5)

        # Status log
        objects = app.hardware.processed_image_count if app.hardware else 0
        status = 'Sorting' if app.sorting_running else 'Idle'
        app.stats_log = tk.Label(
            main_container,
            text=f"Objects: {objects} | Status: {status}",
            font=FONTS['paragraph'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_secondary']
        )
        app.stats_log.pack(pady=SIZES['padding'], anchor='center')

        # Log display
        if app.log_visible:
            app.log_text = scrolledtext.ScrolledText(
                main_container,
                height=10,
                font=FONTS['log'],
                state='disabled'
            )
            app.log_text.pack(fill='x', padx=SIZES['padding'], pady=SIZES['padding'])
            for entry in app.log_entries:
                app.log_text.insert(tk.END, entry + '\n')
            app.log_text.see(tk.END)
    except Exception as e:
        add_log(app, f"Error displaying main interface: {str(e)}")
        raise

def show_statistics(app):
    """Display the statistics interface."""
    try:
        add_log(app, "Displaying statistics")
        app.clear_main_frame()
        main_container = tk.Frame(app.main_frame, bg=COLORS['bg_secondary'])
        main_container.pack(fill='both', expand=True)

        stats_frame = tk.Frame(main_container, bg=COLORS['bg_secondary'])
        stats_frame.pack(fill='both', expand=True, padx=SIZES['padding'], pady=SIZES['padding'])

        columns = ('Category', 'Units', 'Avg Moisture')
        tree = ttk.Treeview(stats_frame, columns=columns, show='headings')
        tree.heading('Category', text='Category')
        tree.heading('Units', text='Units')
        tree.heading('Avg Moisture', text='Avg Moisture (%)')
        tree.pack(fill='both', expand=True)

        if app.hardware:
            stats = app.hardware.calculate_stats()
            for row in stats:
                tree.insert('', 'end', values=row)

        export_button = tk.Button(
            stats_frame,
            text="Export to PDF",
            command=lambda: export_pdf(app),
            font=FONTS['button'],
            bg=COLORS['btn_active'],
            fg=COLORS['text_primary']
        )
        export_button.pack(pady=SIZES['padding'])
    except Exception as e:
        add_log(app, f"Error displaying statistics: {str(e)}")
        raise

def show_component_status(app):
    """Display the component testing interface."""
    try:
        add_log(app, "Displaying component status")
        app.clear_main_frame()
        main_container = tk.Frame(app.main_frame, bg=COLORS['bg_secondary'])
        main_container.pack(fill='both', expand=True)

        test_frame = tk.Frame(main_container, bg=COLORS['bg_secondary'])
        test_frame.pack(fill='both', expand=True, padx=SIZES['padding'], pady=SIZES['padding'])

        buttons = [
            ('Test Servo', 'SERVO_TEST'),
            ('Test Motor', 'MOTOR_TEST'),
            ('Test NIR Sensor', 'NIR_TEST'),
            ('Test Proximity Sensor', 'PROX_TEST'),
            ('Verify All Components', 'VERIFY_ALL'),
        ]
        for text, cmd in buttons:
            btn = tk.Button(
                test_frame,
                text=text,
                command=lambda c=cmd: _component_action(app, c),
                font=FONTS['button'],
                bg=COLORS['btn_active'],
                fg=COLORS['text_primary'],
                width=20
            )
            btn.pack(pady=SIZES['padding'])

        app.status_label = tk.Label(test_frame, text="Status: Idle", font=FONTS['paragraph'])
        app.status_label.pack(pady=SIZES['padding'])
    except Exception as e:
        add_log(app, f"Error displaying component status: {str(e)}")
        raise

def show_about(app):
    """Display the about interface."""
    try:
        add_log(app, "Displaying about page")
        app.clear_main_frame()
        main_container = tk.Frame(app.main_frame, bg=COLORS['bg_secondary'])
        main_container.pack(fill='both', expand=True)

        about_frame = tk.Frame(main_container, bg=COLORS['bg_secondary'])
        about_frame.pack(fill='both', expand=True, padx=SIZES['padding'], pady=SIZES['padding'])

        profile_image = _load_profile_image(app, "profile.png", (100, 100))
        profile_label = tk.Label(
            about_frame,
            image=profile_image if profile_image else None,
            text="[Profile Image]" if not profile_image else "",
            bg=COLORS['bg_secondary']
        )
        profile_label.image = profile_image
        profile_label.pack(pady=SIZES['padding'])

        tk.Label(
            about_frame,
            text="Automated Copra Segregation System\nVersion 1.0\nDeveloped by Techlaro",
            font=FONTS['paragraph'],
            bg=COLORS['bg_secondary'],
            fg=COLORS['text_primary'],
            justify='center'
        ).pack(pady=SIZES['padding'])
    except Exception as e:
        add_log(app, f"Error displaying about page: {str(e)}")
        raise

def toggle_log(app):
    """Toggle the visibility of the log display."""
    try:
        app.log_visible = not app.log_visible
        add_log(app, f"Log display {'enabled' if app.log_visible else 'disabled'}")
        if app.active_menu:
            app.active_menu()
    except Exception as e:
        add_log(app, f"Error toggling log: {str(e)}")
        raise

def toggle_motor(app):
    """Toggle the motor state."""
    try:
        app.toggle_motor()
    except Exception as e:
        add_log(app, f"Error toggling motor: {str(e)}")
        raise