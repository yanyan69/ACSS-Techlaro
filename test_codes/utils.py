import tkinter as tk
import os
from PIL import Image, ImageTk
from constants import IMAGE_PATH, PROFILE_PATH, SIZES, COLORS
from datetime import datetime
from fpdf import FPDF
import sqlite3

def _load_icon(app, filename, size):
    """Load an icon from the resources/icons directory with the specified size."""
    try:
        icon_path = os.path.join(IMAGE_PATH, filename)
        print(f"Loading icon: {icon_path}")
        if not os.path.exists(icon_path):
            print(f"Icon not found: {icon_path}")
            return None
        img = Image.open(icon_path).resize(size, Image.LANCZOS)
        return ImageTk.PhotoImage(img)
    except Exception as e:
        print(f"Error loading icon {filename}: {str(e)}")
        return None

def _load_profile_image(app, filename, size):
    """Load a profile image from the profile directory with the specified size."""
    try:
        profile_path = os.path.join(PROFILE_PATH, filename)
        print(f"Loading profile image: {profile_path}")
        if not os.path.exists(profile_path):
            print(f"Profile image not found: {profile_path}")
            return None
        img = Image.open(profile_path).resize(size, Image.LANCZOS)
        return ImageTk.PhotoImage(img)
    except Exception as e:
        print(f"Error loading profile image {filename}: {str(e)}")
        return None

def load_icons(app):
    """Load all icons for the application."""
    try:
        print("Starting icon loading")
        app.start_icon = _load_icon(app, "start_icon.png", (SIZES['icon_size'], SIZES['icon_size']))
        print(f"start_icon loaded: {'Success' if app.start_icon else 'Failed'}")
        app.small_start_icon = _load_icon(app, "start_icon.png", (SIZES['icon_size'] // 2, SIZES['icon_size'] // 2))
        print(f"small_start_icon loaded: {'Success' if app.small_start_icon else 'Failed'}")
        app.stop_icon = _load_icon(app, "stop_icon.png", (SIZES['icon_size'], SIZES['icon_size']))
        print(f"stop_icon loaded: {'Success' if app.stop_icon else 'Failed'}")
        app.small_stop_icon = _load_icon(app, "stop_icon.png", (SIZES['icon_size'] // 2, SIZES['icon_size'] // 2))
        print(f"small_stop_icon loaded: {'Success' if app.small_stop_icon else 'Failed'}")
        app.camera_icon = _load_icon(app, "camera_icon.png", (SIZES['icon_size'], SIZES['icon_size']))
        print(f"camera_icon loaded: {'Success' if app.camera_icon else 'Failed'}")
        app.logo_icon = _load_icon(app, "logo.png", (SIZES['icon_size'], SIZES['icon_size']))
        print(f"logo_icon loaded: {'Success' if app.logo_icon else 'Failed'}")
    except Exception as e:
        print(f"Error in load_icons: {str(e)}")
        raise

def add_log(app, message):
    """Add a log entry to the log_text widget and log_entries."""
    try:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        app.log_entries.append(log_entry)
        if hasattr(app, 'log_text') and app.log_text.winfo_exists():
            app.log_text.configure(state='normal')
            app.log_text.insert(tk.END, log_entry + '\n')
            app.log_text.configure(state='disabled')
            app.log_text.see(tk.END)
    except Exception as e:
        print(f"Error adding log: {str(e)}")

def update_log_display(app):
    """Update the log display with current log entries."""
    try:
        if hasattr(app, 'log_text') and app.log_text.winfo_exists():
            app.log_text.configure(state='normal')
            app.log_text.delete(1.0, tk.END)
            for entry in app.log_entries:
                app.log_text.insert(tk.END, entry + '\n')
            app.log_text.configure(state='disabled')
            app.log_text.see(tk.END)
    except Exception as e:
        print(f"Error updating log display: {str(e)}")

def export_pdf(app):
    """Export statistics to a PDF file."""
    try:
        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=12)
        pdf.cell(200, 10, txt="ACSS Statistics Report", ln=True, align='C')
        pdf.ln(10)
        if app.hardware:
            stats = app.hardware.calculate_stats()
            for row in stats:
                pdf.cell(200, 10, txt=f"{row[0]}: {row[1]} units, Avg Moisture: {row[2]}%", ln=True)
        else:
            pdf.cell(200, 10, txt="No hardware data available", ln=True)
        output_path = os.path.join(IMAGE_PATH, f"stats_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pdf")
        pdf.output(output_path)
        add_log(app, f"Exported PDF to {output_path}")
    except Exception as e:
        add_log(app, f"Error exporting PDF: {str(e)}")
        raise

def _component_action(app, message):
    """Handle component test actions."""
    try:
        if not app.hardware or not app.hardware.arduino:
            app.status_label.config(text="Error: No Arduino connected")
            return
        if message == "SERVO_TEST":
            app.hardware.safe_send("SERVO_TEST_90")
            app.status_label.config(text="Servo: Moved to neutral")
        elif message == "MOTOR_TEST":
            app.hardware.safe_send("MOTOR_ON")
            app.hardware.safe_send("MOTOR_OFF")
            app.status_label.config(text="Motor: Toggled on/off")
        elif message == "NIR_TEST":
            app.hardware.read_and_display_nir()
        elif message == "PROX_TEST":
            app.hardware.check_proximity()
        elif message == "VERIFY_ALL":
            app.hardware.verify_all_components()
    except Exception as e:
        app.status_label.config(text=f"Error: {str(e)}")
        add_log(app, f"Component action error: {str(e)}")