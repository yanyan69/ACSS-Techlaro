import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings('ignore')

import tflite_runtime.interpreter as tflite
import shutil
import datetime
import numpy as np
from PIL import Image
import RPi.GPIO as GPIO
import time
import picamera

GPIO.setmode(GPIO.BCM)

LED_PINS = {
    'overcooked': 17,  # Red
    'raw': 27,         # yellow
    'standard': 22     # Green
}

IR_LED_PINS = [23, 24]

for pin in list(LED_PINS.values()) + IR_LED_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

class_names = ['overcooked', 'raw', 'standard']
print(f'\nModel loaded from: {model_path}')

img_path = 'copra_classifier/sample/captured.jpg'

for pin in IR_LED_PINS:
    GPIO.output(pin, GPIO.HIGH)

print("Capturing image...")
with picamera.PiCamera() as cam:
    cam.resolution = (640, 480)
    time.sleep(2)
    cam.capture(img_path)
print(f"Image saved to: {img_path}")

for pin in IR_LED_PINS:
    GPIO.output(pin, GPIO.LOW)

img = Image.open(img_path).resize((180, 180))
img_array = np.array(img, dtype=np.float32)
img_array = np.expand_dims(img_array, axis=0)

interpreter.set_tensor(input_details[0]['index'], img_array)
interpreter.invoke()
predictions = interpreter.get_tensor(output_details[0]['index'])

predicted_class = class_names[np.argmax(predictions)]
confidence = 100 * np.max(predictions)

for cls, pin in LED_PINS.items():
    GPIO.output(pin, GPIO.HIGH if cls == predicted_class else GPIO.LOW)

print(f"""
Prediction Result:
Class: {predicted_class}
Confidence: {confidence:.2f}%
""")

dest_folder = f'copra_classifier/dataset-temp/{predicted_class}/'
os.makedirs(dest_folder, exist_ok=True)

filename = f'{predicted_class}_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.jpg'
shutil.copy(img_path, os.path.join(dest_folder, filename))
print(f"Saved to temp dataset folder: {dest_folder}{filename}")

GPIO.cleanup()