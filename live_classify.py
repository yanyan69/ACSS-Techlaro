import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings('ignore', category=UserWarning, module='google.protobuf')

import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
from time import sleep, time
from gpiozero import LED
from picamera2 import Picamera2
import cv2

# === LED SETUP ===
LED_PINS = {
    'overcooked': LED(17),  # Red
    'raw': LED(27),         # Yellow
    'standard': LED(22)     # Green
}

# === LOAD MODEL ===
class_names = ['overcooked', 'raw', 'standard']
model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
print(f'\nModel loaded from: {model_path}')

# === SETUP CAMERA ===
picam = Picamera2()
picam.configure(picam.create_video_configuration(main={"size": (640, 480)}))
picam.start()
sleep(2)
print("Live classification started. Press Ctrl+C to stop.\n")

# === TRACKING ===
start_time = time()
window_preds = []
window_confidences = []

try:
    while True:
        # Capture frame and show preview using OpenCV
        frame = picam.capture_array()
        cv2.imshow("Live Preview", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Prepare image for classification
        img = Image.fromarray(frame).convert("RGB").resize((180, 180))
        img_array = np.array(img, dtype=np.float32)
        img_array = np.expand_dims(img_array, axis=0)

        # Predict
        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        prediction = interpreter.get_tensor(output_details[0]['index'])[0]

        predicted_index = np.argmax(prediction)
        predicted_class = class_names[predicted_index]
        confidence = prediction[predicted_index] * 100

        window_preds.append(predicted_class)
        window_confidences.append(confidence)

        # === Every 10 seconds, summarize result ===
        if time() - start_time >= 10:
            # Get most common prediction and average confidence
            from collections import Counter
            most_common_class = Counter(window_preds).most_common(1)[0][0]
            avg_conf = np.mean([c for i, c in enumerate(window_confidences) if window_preds[i] == most_common_class])

            print(f"""
[10s Summary]
  Class: {most_common_class}
  Avg Confidence: {avg_conf:.2f}%
""")

            # Light up corresponding LED
            for cls, led in LED_PINS.items():
                led.on() if cls == most_common_class else led.off()

            # Reset tracking
            window_preds.clear()
            window_confidences.clear()
            start_time = time()

        sleep(1)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    picam.stop()
    picam.close()
    cv2.destroyAllWindows()
    for led in LED_PINS.values():
        led.off()
