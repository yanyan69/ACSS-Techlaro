import time
import cv2
import numpy as np
from PIL import Image
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
from gpiozero import LED

# === Setup 3 LED indicators ===
overcooked_led = LED(17)
raw_led = LED(27)
standard_led = LED(22)

def turn_on_led(predicted_class):
    overcooked_led.off()
    raw_led.off()
    standard_led.off()

    if predicted_class == "overcooked":
        overcooked_led.on()
    elif predicted_class == "raw":
        raw_led.on()
    elif predicted_class == "standard":
        standard_led.on()

# === Load the TFLite model ===
model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

class_names = ['overcooked', 'raw', 'standard']
print(f"Model loaded from: {model_path}")

# === Setup camera ===
picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
picam.start()
time.sleep(2)

print("Live classification started. Press Ctrl+C to stop.")

# === Classification loop ===
frame_rate = 1
frame_count = 0
class_interval = 10  # seconds
accum_conf = []
accum_class = []

try:
    while True:
        frame = picam.capture_array()
        preview = frame.copy()

        # Resize and preprocess for model
        img = Image.fromarray(frame).convert("RGB").resize((180, 180))
        img_array = np.array(img, dtype=np.float32) / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        # Inference
        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        pred_index = int(np.argmax(output))
        pred_class = class_names[pred_index]
        conf = float(np.max(output)) * 100

        accum_conf.append(conf)
        accum_class.append(pred_class)

        # Show live camera window
        cv2.imshow("Live Classification", preview)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1
        if frame_count >= class_interval:
            final_class = max(set(accum_class), key=accum_class.count)
            avg_conf = sum(accum_conf) / len(accum_conf)

            print(f"\n[10s Summary]")
            print(f"  Class: {final_class}")
            print(f"  Avg Confidence: {avg_conf:.2f}%")

            turn_on_led(final_class)

            # Reset counter
            frame_count = 0
            accum_class.clear()
            accum_conf.clear()

        time.sleep(1 / frame_rate)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    overcooked_led.off()
    raw_led.off()
    standard_led.off()
    picam.stop()
    picam.close()
    cv2.destroyAllWindows()
