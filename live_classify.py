import time
import numpy as np
from PIL import Image
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
from gpiozero import LED

# === Setup IR LEDs ===
ir_led_pins = [17, 27, 22]  # BCM GPIO numbers
ir_leds = [LED(pin) for pin in ir_led_pins]

# === Load TFLite Model ===
model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

class_names = ['overcooked', 'raw', 'standard']
print(f"Model loaded from: {model_path}")

# === Setup Camera ===
picam = Picamera2()
picam.configure(picam.create_still_configuration(main={"size": (640, 480)}))
picam.start()
time.sleep(2)

# === Start Classification Loop ===
print("Live classification started. Press Ctrl+C to stop.")
frame_interval = 10  # seconds per summary
frame_rate = 1  # 1 frame per second
frame_count = 0
accum_confidences = []
accum_labels = []

# === Turn on IR LEDs ===
for led in ir_leds:
    led.on()

try:
    while True:
        frame_count += 1

        # Capture and preprocess image
        img = picam.capture_array()
        img = Image.fromarray(img).convert("RGB").resize((180, 180))
        img_array = np.array(img, dtype=np.float32) / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        # Predict
        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        predicted_index = int(np.argmax(output))
        predicted_class = class_names[predicted_index]
        confidence = float(np.max(output)) * 100

        accum_confidences.append(confidence)
        accum_labels.append(predicted_class)

        time.sleep(1 / frame_rate)

        # Show summary every 10 seconds
        if frame_count >= frame_interval:
            # Get most frequent prediction
            most_common = max(set(accum_labels), key=accum_labels.count)
            avg_conf = sum(accum_confidences) / len(accum_confidences)

            print(f"\n[10s Summary]")
            print(f"  Class: {most_common}")
            print(f"  Avg Confidence: {avg_conf:.2f}%\n")

            # Reset accumulators
            frame_count = 0
            accum_confidences.clear()
            accum_labels.clear()

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    for led in ir_leds:
        led.off()
    picam.stop()
    picam.close()
