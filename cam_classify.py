import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings('ignore', category=UserWarning, module='google.protobuf')

import tflite_runtime.interpreter as tflite
import shutil, datetime
import numpy as np
from PIL import Image
from time import sleep
import time
from picamera2 import Picamera2
from gpiozero import LED

# === LED SETUP ===
LED_PINS = {
    'overcooked': LED(17),  # Red
    'raw': LED(27),         # Yellow
    'standard': LED(22)     # Green
}

# IR LEDs
ir_leds = [LED(23), LED(24)]

# === LOAD MODEL ===
model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
class_names = ['overcooked', 'raw', 'standard']
print(f'\nModel loaded from: {model_path}')

# === CAPTURE IMAGE ===
img_path = 'copra_classifier/sample/captured.jpg'

# Turn ON IR LEDs
for led in ir_leds:
    led.on()

print("Capturing image...")
picam = Picamera2()
picam.configure(picam.create_still_configuration())
picam.start()
time.sleep(2)
picam.capture_file(img_path)
picam.stop()
print(f"Image saved to: {img_path}")

# Turn OFF IR LEDs
for led in ir_leds:
    led.off()

# === PREDICT ===
img = Image.open(img_path).resize((180, 180))
img_array = np.array(img, dtype=np.float32)
img_array = np.expand_dims(img_array, axis=0)

interpreter.set_tensor(input_details[0]['index'], img_array)
interpreter.invoke()
predictions = interpreter.get_tensor(output_details[0]['index'])

predicted_class = class_names[np.argmax(predictions)]
confidence = 100 * np.max(predictions)

# === LIGHT UP RESULT LED ===
for cls, led in LED_PINS.items():
    led.on() if cls == predicted_class else led.off()

print(f"""
Prediction Result:
Class: {predicted_class}
Confidence: {confidence:.2f}%
""")

# === SAVE TO TEMP DATASET ===
dest_folder = f'copra_classifier/dataset-temp/{predicted_class}/'
os.makedirs(dest_folder, exist_ok=True)
filename = f'{predicted_class}_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
shutil.copy(img_path, os.path.join(dest_folder, filename))
print(f"Saved to temp dataset folder: {dest_folder}{filename}")
