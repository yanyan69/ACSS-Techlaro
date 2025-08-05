import time
import cv2
import numpy as np
from PIL import Image
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
from gpiozero import LED

# === Setup LEDs ===
led_overcooked = LED(17)
led_raw = LED(27)
led_standard = LED(22)

def activate_led(label):
    led_overcooked.off()
    led_raw.off()
    led_standard.off()
    if label == "overcooked":
        led_overcooked.on()
    elif label == "raw":
        led_raw.on()
    elif label == "standard":
        led_standard.on()

# === Load class names ===
with open('copra_classifier/models/class_names.txt', 'r') as f:
    class_names = [line.strip() for line in f.readlines()]

# === Load TFLite model ===
model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_size = input_details[0]['shape'][1:3]  # (height, width)

print(f"✅ Model loaded from: {model_path}")
print(f"✅ Class labels: {class_names}")

# === Setup PiCamera ===
picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
picam.start()
time.sleep(2)

print("📡 Live Classification Running... Press Ctrl+C to stop.")

frame_rate = 1
frame_count = 0
class_interval = 10
accum_conf = []
accum_class = []

try:
    while True:
        frame = picam.capture_array()
        preview = frame.copy()

        # === Preprocess: Resize and normalize ===
        img = Image.fromarray(preview).convert("RGB").resize(input_size)
        input_tensor = np.expand_dims(np.array(img, dtype=np.float32) / 255.0, axis=0)

        # === Run inference ===
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        pred_index = int(np.argmax(output))
        label = class_names[pred_index]
        confidence = float(np.max(output)) * 100

        accum_conf.append(confidence)
        accum_class.append(label)

        # === Visual output ===
        cv2.putText(preview, f'{label} ({confidence:.1f}%)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Live Classification", preview)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1
        if frame_count >= class_interval:
            final_class = max(set(accum_class), key=accum_class.count)
            avg_conf = sum(accum_conf) / len(accum_conf)

            print(f"\n📊 [10s Summary]")
            print(f"   Class: {final_class}")
            print(f"   Confidence: {avg_conf:.2f}%")

            activate_led(final_class)

            # Reset counters
            frame_count = 0
            accum_class.clear()
            accum_conf.clear()

        time.sleep(1 / frame_rate)

except KeyboardInterrupt:
    print("🛑 Stopped by user.")

finally:
    led_raw.off()
    led_standard.off()
    led_overcooked.off()
    picam.stop()
    picam.close()
    cv2.destroyAllWindows()
