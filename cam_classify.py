import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
logging.getLogger('tensorflow').setLevel
warnings.filterwarnings('ignore', category=UserWarning, module='google.protobuf')

import tensorflow as tf
import numpy as np
from PIL import Image
import RPi.GPIO as GPIO
import time
import picamera

# === GPIO SETUP ===
GPIO.setmode(GPIO.BCM)

# LED Pins
LED_PINS = {
    'overcooked': 17,  # Red
    'raw': 27,         # Blue
    'standard': 22     # Green
}

# IR LED Pins
IR_LED_PINS = [23, 24]

# Setup all pins
for pin in list(LED_PINS.values()) + IR_LED_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# === LOAD MODEL ===
model_path = 'copra_classifier/models/copra_model.keras'
model = tf.keras.models.load_model(model_path)
class_names = ['overcooked', 'raw', 'standard']
print(f'\nModel loaded from: {model_path}')

# === CAPTURE IMAGE ===
img_path = 'copra_classifier/sample/captured.jpg'

# Turn ON IR LEDs
for pin in IR_LED_PINS:
    GPIO.output(pin, GPIO.HIGH)

print("Capturing image...")
with picamera.PiCamera() as cam:
    cam.resolution = (640, 480)
    time.sleep(2)  # Let camera adjust
    cam.capture(img_path)
print(f"Image saved to: {img_path}")

# Turn OFF IR LEDs
for pin in IR_LED_PINS:
    GPIO.output(pin, GPIO.LOW)

# === PREPARE IMAGE ===
img = Image.open(img_path).resize((180, 180))
img_array = tf.keras.utils.img_to_array(img)
img_array = tf.expand_dims(img_array, 0)

# === PREDICT ===
predictions = model.predict(img_array)
predicted_class = class_names[np.argmax(predictions)]
confidence = 100 * np.max(predictions)

# === LIGHT UP RESULT LED ===
for cls, pin in LED_PINS.items():
    GPIO.output(pin, GPIO.HIGH if cls == predicted_class else GPIO.LOW)

print(f"""
Prediction Result:
Class: {predicted_class}
Confidence: {confidence:.2f}%
""")

# === STORE TO TEMP DATASET ===
import shutil
import datetime

# File destination
dest_folder = f'copra_classifier/dataset-temp/{predicted_class}/'
os.makedirs(dest_folder, exist_ok=True)

# Generate unique name using timestamp
filename = f'{predicted_class}_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.jpg'
shutil.copy(img_path, os.path.join(dest_folder, filename))
print(f"Saved to temp dataset folder: {dest_folder}{filename}")

# === CLEANUP ===
GPIO.cleanup()