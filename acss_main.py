import tkinter as tk
from tkinter import ttk, messagebox
import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
from datetime import datetime
import json
import logging
import random
import numpy as np
from ultralytics import YOLO
import time
from picamera2 import Picamera2
import cv2  # Kept for YOLO and drawing

# Set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Initialize main window and font
root = tk.Tk()
root.title("Automated Copra Segregation System")
root.geometry("1024x600")  # Initial size, will go fullscreen later
custom_font = tkFont.Font(family="Arial", size=10)

# Global variables
camera_on = False
camera_view_on = False
picam = None
sorting_active = False
sorting_start_time = None
sorting_end_time = None
sorting_data = []
username = ""
batch_id = 0

# Fixed sizes for 1024x600
sidebar_width = 180
icon_size = 29
button_icon_size = 64
camera_size = (512, 300)
logo_size = 50
small_start_size = 12
profile_size = 100
padx = 5
pady = 5

# Global variables for camera updates
last_detections = []
last_update_time = None
last_log_time = None

# Initialize YOLO model
model_path = "my_model/train/weights/best.pt"
if not os.path.exists(model_path):
    print("ERROR: Model path is invalid or model was not found. Please update model_path.")
    exit(0)
model = YOLO(model_path, task='detect')
labels = model.names
bbox_colors = [(164, 120, 87), (68, 148, 228), (93, 97, 209), (178, 182, 133), (88, 159, 106),
               (96, 202, 231), (159, 124, 168), (169, 162, 241), (98, 118, 150), (172, 176, 184)]

# Class name mapping
class_name_map = {
    "raw-copra": "Raw Copra",
    "standard-copra": "Standard Copra",
    "overcooked-copra": "Overcooked Copra"
}

# Function definitions
def load_image(path, size=None, default='resources/icons/default_icon.png'):
    full_path = 'resources/icons/' + path
    if not os.path.exists(full_path):
        logging.warning(f"Image not found at {full_path}, using default: {default}")
        full_path = default
    try:
        img = Image.open(full_path)
        if size:
            img = img.resize(size, Image.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        logging.debug(f"Loaded image {path} successfully")
        return photo
    except Exception as e:
        logging.error(f"Failed to load image {full_path}: {e}")
        return None

def load_profile_image(path, size=None):
    full_path = 'resources/profiles/' + path
    default = 'resources/icons/default_icon.png'
    try:
        img = Image.open(full_path)
        if size:
            img = img.resize(size, Image.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        logging.debug(f"Loaded profile image {path} successfully")
        return photo
    except Exception as e:
        logging.error(f"Failed to load profile image {full_path}: {e}")
        return load_image('default_icon.png', size)

def center_popup(window, width, height):
    window.update_idletasks()
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2
    window.geometry(f"{width}x{height}+{x}+{y}")

def load_user_batch():
    global batch_id, sorting_data, sorting_start_time, sorting_end_time
    if os.path.exists('data/statistics.json'):
        with open('data/statistics.json', 'r') as f:
            try:
                all_data = json.load(f)
            except json.JSONDecodeError:
                logging.error("Invalid JSON in statistics.json, resetting to empty")
                all_data = {}
                with open('data/statistics.json', 'w') as f:
                    json.dump(all_data, f)
        latest_batch = None
        latest_date = None
        for date, batches in sorted(all_data.items(), reverse=True):
            if not isinstance(batches, list):
                logging.warning(f"Invalid data format for date {date} in statistics.json")
                continue
            for batch in batches:
                if not isinstance(batch, dict):
                    continue
                if batch.get('username') == username and (latest_batch is None or date > latest_date):
                    latest_batch = batch
                    latest_date = date
        if latest_batch:
            batch_id = latest_batch['batch_id']
            sorting_data = latest_batch['entries']
            sorting_start_time = datetime.strptime(latest_batch['start_time'], "%Y-%m-%d %H:%M:%S")
            sorting_end_time = datetime.strptime(latest_batch['end_time'], "%Y-%m-%d %H:%M:%S") if latest_batch['end_time'] else None
            logging.debug(f"Loaded batch {batch_id} for user {username}")
    update_right_frame()

def start_sorting(new_batch=True):
    global sorting_active, sorting_start_time, sorting_data, camera_on, picam, batch_id, last_log_time
    if new_batch:
        batch_id += 1
        sorting_data = []
        sorting_start_time = datetime.now()
        last_log_time = None
        logging.debug(f"Starting new batch: {batch_id}")
    else:
        logging.debug(f"Continuing batch: {batch_id}")
    sorting_active = True
    start_btn_frame.pack_forget()
    stop_btn_frame.pack(in_=small_button_frame, padx=5, pady=5, anchor='center')
    if not camera_on:
        camera_on = True
        picam = Picamera2()
        config = picam.create_video_configuration(main={"size": (640, 480)}, lores={"size": (320, 240)}, encode="yuv420")
        picam.configure(config)
        picam.start()
        if not camera_view_on:
            toggle_camera_view()
    update_right_frame()

def batch_prompt():
    if sorting_data and not sorting_active:
        batch_window = tk.Toplevel(root)
        batch_window.title("Batch Selection")
        center_popup(batch_window, 358, 210)
        batch_window.resizable(False, False)
        tk.Label(batch_window, text="Start a new batch or continue existing batch?", font=(custom_font.cget("family"), 10)).pack(pady=20)
        button_frame = tk.Frame(batch_window)
        button_frame.pack(pady=10)
        new_batch_btn = tk.Button(button_frame, text="New Batch", command=lambda: [batch_window.destroy(), start_sorting(True)], font=(custom_font.cget("family"), 10), width=8, height=2)
        new_batch_btn.pack(side='left', padx=5, pady=5, ipadx=10, ipady=5)
        continue_btn = tk.Button(button_frame, text="Continue", command=lambda: [batch_window.destroy(), start_sorting(False)], font=(custom_font.cget("family"), 10), width=8, height=2)
        continue_btn.pack(side='left', padx=5, pady=5, ipadx=10, ipady=5)
        batch_window.grab_set()
    else:
        start_sorting(True)

def stop_sorting():
    global sorting_active, sorting_end_time, camera_on, picam
    sorting_active = False
    sorting_end_time = datetime.now()
    logging.debug("Stopping sorting, delaying update for 2 seconds")
    stop_btn_frame.pack_forget()
    start_btn_frame.pack(in_=small_button_frame, padx=5, pady=5, anchor='center')
    if camera_on:
        camera_on = False
        if picam:
            picam.stop()
            picam.close()
            picam = None
        if camera_view_on:
            toggle_camera_view()
    if sorting_data:
        time.sleep(2)
        data_entry = {
            'batch_id': batch_id,
            'start_time': sorting_start_time.strftime("%Y-%m-%d %H:%M:%S"),
            'end_time': sorting_end_time.strftime("%Y-%m-%d %H:%M:%S"),
            'entries': sorting_data,
            'username': username
        }
        if os.path.exists('data/statistics.json'):
            with open('data/statistics.json', 'r') as f:
                try:
                    all_data = json.load(f)
                except json.JSONDecodeError:
                    logging.error("Invalid JSON in statistics.json, resetting to empty")
                    all_data = {}
                    with open('data/statistics.json', 'w') as f:
                        json.dump(all_data, f)
        else:
            all_data = {}
        date_key = sorting_start_time.strftime("%Y-%m-%d")
        if date_key not in all_data or not isinstance(all_data[date_key], list):
            all_data[date_key] = []
        for i, batch in enumerate(all_data[date_key]):
            if batch['batch_id'] == batch_id and batch['username'] == username:
                all_data[date_key][i] = data_entry
                break
        else:
            all_data[date_key].append(data_entry)
        with open('data/statistics.json', 'w') as f:
            json.dump(all_data, f)
    update_statistics()
    update_right_frame()

def toggle_camera_view():
    global camera_view_on
    camera_view_on = not camera_view_on
    logging.debug(f"Camera view toggled to: {camera_view_on}")
    if camera_view_on:
        camera_toggle_frame.pack_forget()
        cancel_camera_frame.pack(in_=camera_frame, padx=5, pady=5, anchor='center')
        update_camera()
    else:
        cancel_camera_frame.pack_forget()
        camera_toggle_frame.pack(in_=camera_frame, padx=5, pady=5, anchor='center')
        camera_label.config(image=placeholder_img)
        camera_label.image = placeholder_img

def update_camera():
    global camera_image, last_detections, last_log_time
    if camera_view_on and camera_on and picam:
        frame = picam.capture_array("main")  # Capture from main stream
        if frame is not None:
            # Convert YUV to RGB for YOLO (picamera2 outputs YUV by default)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_YUV420p2RGB)
            # Run YOLO inference
            results = model(frame_rgb, verbose=False)
            detections = results[0].boxes
            current_detections = []

            # Process detections
            for i in range(len(detections)):
                xyxy_tensor = detections[i].xyxy.cpu()
                xyxy = xyxy_tensor.numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy.astype(int)
                classidx = int(detections[i].cls.item())
                original_classname = labels[classidx]
                classname = class_name_map.get(original_classname, original_classname.replace('_', ' ').title())
                conf = detections[i].conf.item()
                logging.debug(f"Detected class: {original_classname} -> {classname}")

                if conf > 0.5:
                    color = bbox_colors[classidx % 10]
                    cv2.rectangle(frame_rgb, (xmin, ymin), (xmax, ymax), color, 2)
                    label = f'{classname} ({int(conf*100)}%)'
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    label_ymin = max(ymin, label_size[1] + 10)
                    cv2.rectangle(frame_rgb, (xmin, label_ymin - label_size[1] - 10), (xmin + label_size[0], label_ymin + 10), color, cv2.FILLED)
                    cv2.putText(frame_rgb, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    current_detections.append((classname, conf, (xmin, ymin, xmax, ymax)))

            # Check for new detections with debouncing
            current_time = datetime.now()
            new_detections = [d for d in current_detections if d not in last_detections]
            if new_detections and sorting_active and (last_log_time is None or (current_time - last_log_time).total_seconds() >= 2):
                for classname, conf, bbox in new_detections:
                    unit = 1.0
                    moisture = 0.0
                    description = classname
                    sorting_data.append({'unit': unit, 'description': description, 'moisture': moisture, 'detected_class': classname, 'confidence': conf})
                    logging.debug(f"Added to sorting_data: {classname}, unit: {unit}")
                    last_log_time = current_time
                update_right_frame()

            last_detections = current_detections.copy()

            # Convert frame for display
            camera_image = cv2.resize(frame_rgb, camera_size)
            img = Image.fromarray(camera_image)
            photo = ImageTk.PhotoImage(image=img)
            camera_label.config(image=photo)
            camera_label.image = photo
        root.after(100, update_camera)

def update_right_frame():
    global last_update_time
    current_time = datetime.now()
    if last_update_time is None or (current_time - last_update_time).total_seconds() >= 2:
        for widget in right_scrollable_frame.winfo_children():
            widget.destroy()
        tk.Label(right_scrollable_frame, text=f"User: {username} | Batch {batch_id}", font=(custom_font.cget("family"), 10)).pack(anchor='w')
        if sorting_data:
            if sorting_active:
                tk.Label(right_scrollable_frame, text="Sorting Started " + sorting_start_time.strftime("%Y-%m-%d %H:%M:%S"), font=(custom_font.cget("family"), 10)).pack(anchor='w')
            else:
                tk.Label(right_scrollable_frame, text="Batch Last Updated " + (sorting_end_time or sorting_start_time).strftime("%Y-%m-%d %H:%M:%S"), font=(custom_font.cget("family"), 10)).pack(anchor='w')
            for data in sorting_data:
                label_text = f"{data['unit']} unit; {data['description']} ({int(data['confidence']*100)}%); moisture: [{data['moisture']:.2f}%]"
                tk.Label(right_scrollable_frame, text=label_text, font=(custom_font.cget("family"), 10)).pack(anchor='w')
        right_canvas.update_idletasks()
        right_canvas.configure(scrollregion=right_canvas.bbox("all"))
        logging.debug("Updated right frame")
        last_update_time = current_time

def delete_batch(date, batch_id):
    if os.path.exists('data/statistics.json'):
        with open('data/statistics.json', 'r') as f:
            try:
                all_data = json.load(f)
            except json.JSONDecodeError:
                logging.error("Invalid JSON in statistics.json, resetting to empty")
                all_data = {}
                with open('data/statistics.json', 'w') as f:
                    json.dump(all_data, f)
        if date in all_data and isinstance(all_data[date], list):
            all_data[date] = [batch for batch in all_data[date] if batch['batch_id'] != batch_id]
            if not all_data[date]:
                del all_data[date]
            with open('data/statistics.json', 'w') as f:
                json.dump(all_data, f)
    update_statistics()

def delete_batch_prompt(date, batch_id):
    delete_window = tk.Toplevel(root)
    delete_window.title("Delete Batch")
    center_popup(delete_window, 358, 210)
    delete_window.resizable(False, False)
    tk.Label(delete_window, text="Are you sure you want to delete this batch?", font=(custom_font.cget("family"), 10)).pack(pady=20)
    button_frame = tk.Frame(delete_window)
    button_frame.pack(pady=10)
    yes_btn = tk.Button(button_frame, text="Yes", command=lambda: [delete_window.destroy(), delete_batch(date, batch_id)], font=(custom_font.cget("family"), 10), width=8, height=2)
    yes_btn.pack(side='left', padx=5, pady=5, ipadx=10, ipady=5)
    no_btn = tk.Button(button_frame, text="No", command=delete_window.destroy, font=(custom_font.cget("family"), 10), width=8, height=2)
    no_btn.pack(side='left', padx=5, pady=5, ipadx=10, ipady=5)
    delete_window.grab_set()

def update_statistics():
    global frame_order
    for widget in stats_scrollable_frame.winfo_children():
        widget.destroy()
    
    title_frame = wrap_widget(stats_scrollable_frame, "Title Frame", 'pack', fill='x', padx=20, pady=10)
    tk.Label(title_frame, text="Statistics", font=(custom_font.cget("family"), 16, 'bold')).pack(anchor='w', padx=padx, pady=pady)
    
    if os.path.exists('data/statistics.json'):
        with open('data/statistics.json', 'r') as f:
            try:
                all_data = json.load(f)
            except json.JSONDecodeError:
                logging.error("Invalid JSON in statistics.json, resetting to empty")
                all_data = {}
                with open('data/statistics.json', 'w') as f:
                    json.dump(all_data, f)
    else:
        all_data = {}
    
    frame_order = []
    
    if not all_data:
        table_frame_container = wrap_widget(stats_scrollable_frame, "Empty Table Frame Container", 'pack', fill='both', expand=True, padx=20, pady=10)
        table_frame = wrap_widget(table_frame_container, "Empty Table Frame", 'pack', fill='both', expand=True, padx=20, pady=10)
        headers = ["Copra Classification", "Units", "Average Moisture", "Total Time"]
        for col, header in enumerate(headers):
            tk.Label(table_frame, text=header, font=(custom_font.cget("family"), 14, 'bold'), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=0, column=col, sticky='nsew')
        table_frame.columnconfigure((0,1,2,3), weight=1)
    else:
        found_data = False
        for date, batches in sorted(all_data.items(), reverse=True):
            if not isinstance(batches, list):
                logging.warning(f"Invalid data format for date {date} in statistics.json")
                continue
            for batch in batches:
                if not isinstance(batch, dict) or batch.get('username') != username:
                    continue
                found_data = True
                table_frame_container = wrap_widget(stats_scrollable_frame, f"Table {batch['batch_id']:03d} Frame", 'pack', fill='both', expand=True, padx=20, pady=10)
                subheading_frame = wrap_widget(table_frame_container, f"Subheading {date} Batch {batch['batch_id']}", 'pack', fill='x', padx=20, pady=10)
                date_frame = wrap_widget(table_frame_container, f"Date Frame {date} Batch {batch['batch_id']}", 'pack', fill='both', expand=True, padx=20, pady=10)
                collapse_var = tk.BooleanVar(value=False)
                
                collapse_btn = tk.Button(subheading_frame, text="−" if collapse_var.get() else "+", font=(custom_font.cget("family"), 10))
                collapse_btn.pack(side='left', padx=padx)
                collapse_btn.configure(command=lambda df=date_frame, cv=collapse_var, cb=collapse_btn, sf=subheading_frame, d=date, bid=batch['batch_id']: toggle_collapse(df, cv, cb, sf, d, bid))
                tk.Label(subheading_frame, text=f"Name: {batch['username']} | Batch {batch['batch_id']}", font=(custom_font.cget("family"), 14, 'bold'), bg='#D3D3D3').pack(side='left', padx=padx)
                tk.Label(subheading_frame, text=date, font=(custom_font.cget("family"), 14, 'bold'), bg='#D3D3D3').pack(side='left', padx=padx)
                tk.Button(subheading_frame, text="Delete", command=lambda d=date, bid=batch['batch_id']: delete_batch_prompt(d, bid), font=(custom_font.cget("family"), 10)).pack(side='right', padx=padx)
                
                frame_order.append((table_frame_container, date_frame, date, batch['batch_id']))
                
                if collapse_var.get():
                    date_frame.pack_forget()
                
                raw_copra = [e for e in batch['entries'] if e['detected_class'] == "Raw Copra"]
                standard_copra = [e for e in batch['entries'] if e['detected_class'] == "Standard Copra"]
                overcooked_copra = [e for e in batch['entries'] if e['detected_class'] == "Overcooked Copra"]
                
                raw_units = sum(e['unit'] for e in raw_copra) if raw_copra else 0
                standard_units = sum(e['unit'] for e in standard_copra) if standard_copra else 0
                overcooked_units = sum(e['unit'] for e in overcooked_copra) if overcooked_copra else 0
                
                raw_avg_moisture = sum(e['moisture'] for e in raw_copra) / len(raw_copra) if raw_copra else 0
                standard_avg_moisture = sum(e['moisture'] for e in standard_copra) / len(standard_copra) if standard_copra else 0
                overcooked_avg_moisture = sum(e['moisture'] for e in overcooked_copra) / len(overcooked_copra) if overcooked_copra else 0
                
                total_units = raw_units + standard_units + overcooked_units
                total_moisture = sum(e['moisture'] for e in batch['entries'])
                overall_avg_moisture = total_moisture / len(batch['entries']) if batch['entries'] else 0
                total_time = datetime.strptime(batch['end_time'], "%Y-%m-%d %H:%M:%S") - datetime.strptime(batch['start_time'], "%Y-%m-%d %H:%M:%S")
                total_time_str = str(total_time).split('.')[0]
                
                table_frame = wrap_widget(date_frame, f"Table Frame {date} Batch {batch['batch_id']}", 'pack', fill='both', expand=True, padx=20, pady=10)
                headers = ["Copra Classification", "Units", "Average Moisture", "Total Time"]
                for col, header in enumerate(headers):
                    tk.Label(table_frame, text=header, font=(custom_font.cget("family"), 14, 'bold'), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=0, column=col, sticky='nsew')
                
                tk.Label(table_frame, text="Raw Copra (>7%)", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=1, column=0, sticky='nsew')
                tk.Label(table_frame, text=f"{raw_units:.1f}", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=1, column=1, sticky='nsew')
                tk.Label(table_frame, text=f"{raw_avg_moisture:.2f}%" if raw_copra else "0.00%", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=1, column=2, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=1, column=3, sticky='nsew')
                
                tk.Label(table_frame, text="Standard Copra (6-7%)", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=2, column=0, sticky='nsew')
                tk.Label(table_frame, text=f"{standard_units:.1f}", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=2, column=1, sticky='nsew')
                tk.Label(table_frame, text=f"{standard_avg_moisture:.2f}%" if standard_copra else "0.00%", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=2, column=2, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=2, column=3, sticky='nsew')
                
                tk.Label(table_frame, text="Overcooked Copra (<6%)", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=3, column=0, sticky='nsew')
                tk.Label(table_frame, text=f"{overcooked_units:.1f}", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=3, column=1, sticky='nsew')
                tk.Label(table_frame, text=f"{overcooked_avg_moisture:.2f}%" if overcooked_copra else "0.00%", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=3, column=2, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=3, column=3, sticky='nsew')
                
                tk.Label(table_frame, text="Total Units", font=(custom_font.cget("family"), 14, 'bold'), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=4, column=0, sticky='nsew')
                tk.Label(table_frame, text=f"{total_units:.1f}", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=4, column=1, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=4, column=2, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=4, column=3, sticky='nsew')
                
                tk.Label(table_frame, text="Overall Average Moisture", font=(custom_font.cget("family"), 14, 'bold'), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=5, column=0, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=5, column=1, sticky='nsew')
                tk.Label(table_frame, text=f"{overall_avg_moisture:.2f}%", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=5, column=2, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=5, column=3, sticky='nsew')
                
                tk.Label(table_frame, text="Total Time", font=(custom_font.cget("family"), 14, 'bold'), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=6, column=0, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=6, column=1, sticky='nsew')
                tk.Label(table_frame, text="", font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=6, column=2, sticky='nsew')
                tk.Label(table_frame, text=total_time_str, font=(custom_font.cget("family"), 10), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=6, column=3, sticky='nsew')
                
                table_frame.columnconfigure((0,1,2,3), weight=1)
        
        if not found_data:
            table_frame_container = wrap_widget(stats_scrollable_frame, "Empty Table Frame Container", 'pack', fill='both', expand=True, padx=20, pady=10)
            table_frame = wrap_widget(table_frame_container, "Empty Table Frame", 'pack', fill='both', expand=True, padx=20, pady=10)
            headers = ["Copra Classification", "Units", "Average Moisture", "Total Time"]
            for col, header in enumerate(headers):
                tk.Label(table_frame, text=header, font=(custom_font.cget("family"), 14, 'bold'), borderwidth=1, relief='solid', padx=10, pady=10).grid(row=0, column=col, sticky='nsew')
            table_frame.columnconfigure((0,1,2,3), weight=1)
    
    stats_canvas.update_idletasks()
    stats_canvas.configure(scrollregion=stats_canvas.bbox("all"))

def toggle_collapse(date_frame, collapse_var, collapse_btn, subheading_frame, date, batch_id):
    if collapse_var.get():
        collapse_btn.config(text="+")
        date_frame.pack_forget()
    else:
        collapse_btn.config(text="−")
        date_frame.pack(fill='both', expand=True)
    collapse_var.set(not collapse_var.get())
    update_statistics()

def update_debug():
    for frame in debug_frames:
        frame.config(borderwidth=1 if debug_var.get() else 0, relief='solid' if debug_var.get() else 'flat')
    for label in debug_labels:
        label.config(bg='yellow' if debug_var.get() else '')
        label.place(x=2, y=2) if debug_var.get() else label.place_forget()
    logging.debug("Debug mode updated")

def exit_prompt():
    exit_window = tk.Toplevel(root)
    exit_window.title("Exit Confirmation")
    center_popup(exit_window, 358, 210)
    exit_window.resizable(False, False)
    tk.Label(exit_window, text="Are you sure you want to exit?", font=(custom_font.cget("family"), 10)).pack(pady=20)
    button_frame = tk.Frame(exit_window)
    button_frame.pack(pady=10)
    yes_btn = tk.Button(button_frame, text="Yes", command=lambda: [exit_window.destroy(), root.quit()], font=(custom_font.cget("family"), 10), width=8, height=2)
    yes_btn.pack(side='left', padx=5, pady=5, ipadx=10, ipady=5)
    no_btn = tk.Button(button_frame, text="No", command=exit_window.destroy, font=(custom_font.cget("family"), 10), width=8, height=2)
    no_btn.pack(side='left', padx=5, pady=5, ipadx=10, ipady=5)
    exit_window.grab_set()

# Load images
start_img_small = load_image('start_icon.png', size=(button_icon_size, button_icon_size))
stop_img = load_image('stop_icon.png', size=(button_icon_size, button_icon_size))
camera_img = load_image('camera_icon.png', size=(button_icon_size, button_icon_size))
cancel_camera_img = load_image('cancel_camera_icon.png', size=(button_icon_size, button_icon_size))
small_start_img = load_image('start_icon.png', size=(small_start_size, small_start_size))
placeholder_img = load_image('default_icon.png', size=camera_size)

# Verify image loading
for img, name in [(start_img_small, 'start_img_small'), (stop_img, 'stop_img'),
                  (camera_img, 'camera_img'), (cancel_camera_img, 'cancel_camera_img'),
                  (small_start_img, 'small_start_img'), (placeholder_img, 'placeholder_img')]:
    if img is None:
        logging.error(f"Image {name} failed to load")

debug_var = tk.BooleanVar(value=False)
debug_frames = []
debug_labels = []
frame_order = []

def wrap_widget(parent, name, layout_manager='pack', width=None, **layout_kwargs):
    frame = tk.Frame(parent, borderwidth=1 if debug_var.get() else 0, relief='solid', bg=layout_kwargs.pop('bg', 'white'))
    if width is not None:
        frame.config(width=width)
    debug_frames.append(frame)
    label = tk.Label(frame, text=name, font=(custom_font.cget("family"), 5), bg='yellow')
    debug_labels.append(label)
    if debug_var.get():
        label.place(x=2, y=2)
        label.lift()
    if layout_manager == 'pack':
        frame.pack(**layout_kwargs)
    elif layout_manager == 'grid':
        frame.grid(**layout_kwargs)
    return frame

sidebar = tk.Frame(root, borderwidth=1, relief='solid', bg='white', width=sidebar_width)
sidebar.pack(side='left', padx=padx, pady=pady, fill='y')

top_frame = wrap_widget(sidebar, "Top Frame", 'pack', side='top', fill='x', pady=pady, padx=padx)
logo_img = load_image('logo.png', size=(logo_size, logo_size))
if logo_img:
    tk.Label(top_frame, image=logo_img, bg='white').pack(side='left', padx=padx)
tk.Label(top_frame, text="ACSS", font=(custom_font.cget("family"), 14, 'bold'), bg='white').pack(side='left')

divider_frame = wrap_widget(sidebar, "Divider", 'pack', fill='x', pady=pady)
tk.Frame(divider_frame, height=2, bg='gray').pack(fill='x', padx=padx)

pages = ["Main Interface", "Statistics", "Component Settings", "About Us", "Exit"]
icons = ["main_interface_icon.png", "statistics_icon.png", "component_settings_icon.png", "about_us_icon.png", "exit_icon.png"]
button_dict = {}
current_page = None

def switch_page(page):
    global current_page
    if page == "Exit":
        exit_prompt()
        return
    for child in main_frame.winfo_children():
        child.pack_forget()
    if page == "Main Interface":
        main_interface_frame.pack(expand=True, fill='both')
    elif page == "Statistics":
        stats_frame.pack(expand=True, fill='both')
        update_statistics()
    elif page == "Component Settings":
        settings_frame.pack(expand=True, fill='both')
    elif page == "About Us":
        about_frame.pack(expand=True, fill='both')
    if current_page:
        button_dict[current_page]['button'].config(bg='white')
    button_dict[page]['button'].config(bg='lightgray')
    current_page = page

for i, (page, icon) in enumerate(zip(pages, icons)):
    page_wrap = wrap_widget(sidebar, page, 'pack', fill='x', padx=padx, pady=pady)
    icon_img = load_image(icon, size=(icon_size, icon_size))
    btn = tk.Button(page_wrap, image=icon_img, text=page, compound='top', font=(custom_font.cget("family"), 10), bg='white', relief='flat', padx=padx, pady=pady, command=lambda p=page: switch_page(p))
    btn.image = icon_img
    btn.pack(fill='x', padx=padx, pady=pady)
    button_dict[page] = {'button': btn, 'image': icon_img}
    if i < len(pages) - 1:
        divider_frame = wrap_widget(sidebar, f"Divider {i+1}", 'pack', fill='x', pady=pady)
        tk.Frame(divider_frame, height=2, bg='gray').pack(fill='x', padx=padx)

main_frame = wrap_widget(root, "Main Frame", 'pack', side='left', expand=True, fill='both', padx=padx, pady=pady)

stats_frame = wrap_widget(main_frame, "Statistics Frame", 'pack', expand=True, fill='both')
stats_frame.config(width=800)
margin_frame = wrap_widget(stats_frame, "Margin Frame", 'pack', padx=40, pady=20, fill='both', expand=True)

center_frame = tk.Frame(margin_frame)
center_frame.pack(expand=True, fill='both', anchor='center')

stats_canvas = tk.Canvas(center_frame)
stats_scrollbar = tk.Scrollbar(center_frame, orient="vertical", command=stats_canvas.yview)
stats_scrollable_frame = wrap_widget(stats_canvas, "Stats Scrollable Frame", 'pack', padx=20, pady=10, fill='both', expand=True)
stats_scrollable_frame.bind("<Configure>", lambda e: stats_canvas.configure(scrollregion=stats_canvas.bbox("all")))
stats_canvas.create_window((0, 0), window=stats_scrollable_frame, anchor="nw")
stats_canvas.configure(yscrollcommand=stats_scrollbar.set, yscrollincrement=10)
stats_canvas.pack(side="left", fill="both", expand=True)
stats_scrollbar.pack(side="right", fill="y")

about_frame = wrap_widget(main_frame, "About Frame", 'pack', expand=True, fill='both', padx=padx, pady=pady)
canvas = tk.Canvas(about_frame)
scrollbar = tk.Scrollbar(about_frame, orient="vertical", command=canvas.yview)
scrollable_frame = wrap_widget(canvas, "About Scrollable Frame", 'pack')
scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
canvas.configure(yscrollcommand=scrollbar.set, yscrollincrement=10)
canvas.pack(side="left", fill="both", expand=True)
scrollbar.pack(side="right", fill="y")

logo_frame = wrap_widget(scrollable_frame, "Logo Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
logo_img_about = load_image('logo.png', size=(logo_size, logo_size))
if logo_img_about:
    tk.Label(logo_frame, image=logo_img_about).pack(pady=10)

about_us_frame = wrap_widget(scrollable_frame, "About Us Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
tk.Label(about_us_frame, text="About Us – Techlaro", font=(custom_font.cget("family"), 14, 'bold')).pack(anchor='w', pady=10)
tk.Label(about_us_frame, text="We are Techlaro, a student-led engineering team dedicated to creating innovative solutions for real-world problems. Our flagship project, the Automated Copra Segregation System (ACSS), is designed to improve the accuracy and efficiency of copra quality assessment.", font=(custom_font.cget("family"), 10), wraplength=800).pack(anchor='w', pady=10)

system_frame = wrap_widget(scrollable_frame, "System Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
tk.Label(system_frame, text="Our System", font=(custom_font.cget("family"), 14, 'bold')).pack(anchor='w', pady=10)
tk.Label(system_frame, text="The Automated Copra Segregation System (ACSS) uses image processing, NIR (Near-Infrared) sensors, and smart algorithms to classify copra based on quality standards. By reducing human error and inconsistency in manual sorting, our system helps farmers and traders save time, cut costs, and improve product reliability.", font=(custom_font.cget("family"), 10), wraplength=800).pack(anchor='w', pady=10)

vision_frame = wrap_widget(scrollable_frame, "Vision Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
tk.Label(vision_frame, text="Our Vision", font=(custom_font.cget("family"), 14, 'bold')).pack(anchor='w', pady=10)
tk.Label(vision_frame, text="At Techlaro, we aim to bridge engineering and agriculture by applying modern technology to age-old farming practices. Our mission is to create affordable, practical, and efficient systems that support local farmers while promoting sustainability and innovation in the industry.", font=(custom_font.cget("family"), 10), wraplength=800).pack(anchor='w', pady=10)

team_frame = wrap_widget(scrollable_frame, "Team Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
tk.Label(team_frame, text="Meet the Team", font=(custom_font.cget("family"), 14, 'bold')).pack(anchor='w', pady=10)
team_grid = wrap_widget(team_frame, "Team Grid", 'pack', fill='both', expand=True)
team_images = [
    load_profile_image('jerald.png', size=(profile_size, profile_size)),
    load_profile_image('johnpaul.png', size=(profile_size, profile_size)),
    load_profile_image('christian.png', size=(profile_size, profile_size)),
    load_profile_image('marielle.png', size=(profile_size, profile_size))
]
team_members = [
    ("Engr. Jerald James D. Preclaro", "Team Lead / Lead Developer"),
    ("Engr. John Paul F. Armenta", "Hardware Engineer"),
    ("Engr. Christian L. Narvaez", "Software Engineer"),
    ("Engr. Marielle B. Maming", "Database Engineer")
]
for i, (name, role) in enumerate(team_members):
    member_frame = wrap_widget(team_grid, f"Member Frame {i+1}", 'grid', row=i//2, column=i%2, padx=padx, pady=pady, sticky='nsew')
    tk.Label(member_frame, image=team_images[i]).pack(padx=padx, pady=5)
    tk.Label(member_frame, text=name, font=(custom_font.cget("family"), 10, 'bold')).pack(pady=5)
    tk.Label(member_frame, text=role, font=(custom_font.cget("family"), 10)).pack(pady=5)
team_grid.columnconfigure((0, 1), weight=1)

contact_frame = wrap_widget(scrollable_frame, "Contact Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
tk.Label(contact_frame, text="Contact Us", font=(custom_font.cget("family"), 14, 'bold')).pack(anchor='w', pady=10)
tk.Label(contact_frame, text="Email: techlaro.team@gmail.com", font=(custom_font.cget("family"), 10)).pack(anchor='w', pady=5)
tk.Label(contact_frame, text="Website: www.techlaro.com", font=(custom_font.cget("family"), 10)).pack(anchor='w', pady=5)

footer_frame = wrap_widget(scrollable_frame, "Footer Frame", 'pack', fill='both', expand=True, padx=20, pady=20)
tk.Label(footer_frame, text="© 2025 Techlaro – Automated Copra Segregation System | All Rights Reserved", font=(custom_font.cget("family"), 8)).pack(anchor='w', pady=10)

settings_frame = wrap_widget(main_frame, "Settings Frame", 'pack', expand=True, fill='both', padx=padx, pady=pady)
settings_wrap = wrap_widget(settings_frame, "Settings", 'pack', expand=True, fill='both', pady=pady, padx=padx)
tk.Checkbutton(settings_wrap, text="Show debug labels and borders", variable=debug_var, command=update_debug, font=(custom_font.cget("family"), 10)).pack(pady=pady, padx=padx, anchor='w')

main_interface_frame = wrap_widget(main_frame, "Main Interface Frame", 'pack', expand=True, fill='both', padx=padx, pady=pady)

heading_frame = wrap_widget(main_interface_frame, "Heading Frame", 'pack', fill='x', pady=pady, padx=padx)
tk.Label(heading_frame, text="Automated Copra Segregation System", font=(custom_font.cget("family"), 16, 'bold')).pack(padx=padx, pady=pady)

text_frame = wrap_widget(main_interface_frame, "Instruction Text Frame", 'pack', fill='x', pady=pady, padx=padx)
instr_left = tk.Label(text_frame, text="Press the ", font=(custom_font.cget("family"), 10))
instr_left.pack(side='left', padx=padx)
icon_label = tk.Label(text_frame, image=small_start_img)
icon_label.image = small_start_img
icon_label.pack(side='left')
instr_right = tk.Label(text_frame, text=" to start sorting", font=(custom_font.cget("family"), 10))
instr_right.pack(side='left', padx=padx)

camera_container = wrap_widget(main_interface_frame, "Camera Container", 'pack', fill='x', padx=padx, pady=pady)
camera_view_frame = tk.Frame(camera_container, width=camera_size[0], height=camera_size[1])
camera_view_frame.pack(side='left', padx=padx, pady=pady)
camera_view_frame.pack_propagate(False)
camera_label = tk.Label(camera_view_frame, image=placeholder_img)
camera_label.image = placeholder_img
camera_label.pack(padx=padx, pady=pady)

right_frame = wrap_widget(camera_container, "Right Frame", 'pack', side='right', expand=True, fill='both', padx=padx, pady=pady)
right_canvas = tk.Canvas(right_frame)
right_scrollbar = tk.Scrollbar(right_frame, orient="vertical", command=right_canvas.yview)
right_scrollable_frame = tk.Frame(right_canvas)
right_scrollable_frame.bind("<Configure>", lambda e: right_canvas.configure(scrollregion=right_canvas.bbox("all")))
right_canvas.create_window((0, 0), window=right_scrollable_frame, anchor="nw")
right_canvas.configure(yscrollcommand=right_scrollbar.set, yscrollincrement=10)
right_canvas.pack(side="left", fill="both", expand=True)
right_scrollbar.pack(side="right", fill="y")
tk.Label(right_scrollable_frame, text=f"User: {username}", font=(custom_font.cget("family"), 10)).pack(anchor='w')

controls_frame = wrap_widget(main_interface_frame, "Controls Frame", 'pack', fill='x', padx=padx, pady=pady)
small_button_frame = wrap_widget(controls_frame, "Start Button Frame", 'pack', side='left', expand=True, padx=padx, pady=pady)
start_btn_frame = tk.Frame(small_button_frame, borderwidth=1, relief='solid')
start_btn_frame.pack(padx=padx, pady=pady, anchor='center')
start_btn = tk.Button(start_btn_frame, image=start_img_small, relief='flat', activebackground='lightgray', command=batch_prompt)
start_btn.image = start_img_small
start_btn.pack(padx=padx, pady=pady, anchor='center')

stop_btn_frame = tk.Frame(small_button_frame, borderwidth=1, relief='solid')
stop_btn = tk.Button(stop_btn_frame, image=stop_img, relief='flat', activebackground='lightgray', command=stop_sorting)
stop_btn.image = stop_img
stop_btn.pack(padx=padx, pady=pady, anchor='center')

camera_frame = wrap_widget(controls_frame, "Camera Toggle Frame", 'pack', side='left', expand=True, padx=padx, pady=pady)
camera_toggle_frame = tk.Frame(camera_frame, borderwidth=1, relief='solid')
camera_toggle_frame.pack(padx=padx, pady=pady, anchor='center')
camera_toggle = tk.Button(camera_toggle_frame, image=camera_img, relief='flat', activebackground='lightgray', command=toggle_camera_view)
camera_toggle.image = camera_img
camera_toggle.pack(padx=padx, pady=pady, anchor='center')

cancel_camera_frame = tk.Frame(camera_frame, borderwidth=1, relief='solid')
cancel_camera_btn = tk.Button(cancel_camera_frame, image=cancel_camera_img, relief='flat', activebackground='lightgray', command=toggle_camera_view)
cancel_camera_btn.image = cancel_camera_img
cancel_camera_btn.pack(padx=padx, pady=pady, anchor='center')

def get_username():
    global username
    username_window = tk.Toplevel(root)
    username_window.title("Enter Username")
    center_popup(username_window, 358, 210)
    username_window.resizable(False, False)
    username_window.attributes('-topmost', True)
    username_window.update()

    if not os.path.exists('data'):
        os.makedirs('data')

    usernames_file = 'data/usernames.json'
    if os.path.exists(usernames_file):
        with open(usernames_file, 'r') as f:
            try:
                past_usernames = json.load(f)
            except json.JSONDecodeError:
                past_usernames = []
    else:
        past_usernames = []

    tk.Label(username_window, text="Username (max 16 characters):", font=(custom_font.cget("family"), 10)).pack(pady=10)
    username_var = tk.StringVar()
    username_combobox = ttk.Combobox(username_window, textvariable=username_var, values=past_usernames, font=(custom_font.cget("family"), 10))
    username_combobox.pack(pady=10)

    def submit_username():
        global username
        entered_username = username_var.get().strip()
        if not entered_username:
            messagebox.showerror("Error", "Username cannot be blank.")
            return
        if len(entered_username) > 16:
            messagebox.showerror("Error", "Username cannot exceed 16 characters.")
            return
        username = entered_username
        if username and username not in past_usernames:
            past_usernames.append(username)
            with open('data/usernames.json', 'w') as f:
                json.dump(past_usernames, f)
        username_window.destroy()
        root.attributes('-fullscreen', True)
        load_user_batch()
        update_right_frame()

    submit_btn = tk.Button(username_window, text="Submit", command=submit_username, font=(custom_font.cget("family"), 10), width=8, height=2)
    submit_btn.pack(pady=10, padx=5, ipadx=10, ipady=5)
    username_window.grab_set()

if __name__ == "__main__":
    get_username()
    switch_page("Main Interface")
    root.mainloop()