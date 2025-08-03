import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings('ignore', category=UserWarning, module='google.protobuf')

import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
from time import sleep, time
from picamera2 import Picamera2
from gpiozero import LED
from collections import Counter

# === SETUP ===
LED_PINS = {
    'overcooked': LED(17),
    'raw': LED(27),
    'standard': LED(22)
}
ir_leds = [LED(23), LED(24)]

class_names = ['overcooked', 'raw', 'standard']
model_path = 'copra_classifier/models/copra_model.tflite'

# Load TFLite model
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print(f'\nModel loaded from: {model_path}')

# Turn ON IR LEDs
for led in ir_leds:
    led.on()

# Start Camera
picam = Picamera2()
picam.configure(picam.create_still_configuration(main={"size": (640, 480)}))  # 720p or similar
picam.start()
sleep(2)

print("Starting live classification...\n")

# === LIVE CLASSIFICATION LOOP ===
try:
    result_window = []  # store predictions for averaging
    last_report_time = time()

    while True:
        # Capture frame to memory (PIL image)
        frame = picam.capture_array()
        img = Image.fromarray(frame).resize((180, 180))
        img_array = np.array(img, dtype=np.float32)
        img_array = np.expand_dims(img_array, axis=0)

        # Run inference
        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        predictions = interpreter.get_tensor(output_details[0]['index'])

        predicted_class = class_names[np.argmax(predictions)]
        result_window.append(predicted_class)

        # After 10 seconds, compute most common prediction
        if time() - last_report_time >= 10:
            most_common = Counter(result_window).most_common(1)[0]
            label, count = most_common
            confidence = (count / len(result_window)) * 100

            # LED feedback
            for cls, led in LED_PINS.items():
                led.on() if cls == label else led.off()

            print(f"""
[10s Summary]
Class: {label}
Confidence: {confidence:.2f}%
            """)
            result_window.clear()
            last_report_time = time()

        sleep(1)  # Capture every 1 second

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    picam.stop()
    picam.close()
    for led in ir_leds + list(LED_PINS.values()):
        led.off()
