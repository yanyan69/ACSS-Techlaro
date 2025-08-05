import time
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
from gpiozero import LED
from PIL import Image

# === LED Setup ===
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

# === Model and Labels ===
model_path = 'copra_classifier/models/copra_model.tflite'
class_names = ['overcooked', 'raw', 'standard']

interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_size = input_details[0]['shape'][:2]  # [180, 180]

print(f"✅ Model loaded from: {model_path}")
print(f"✅ Class labels: {class_names}")

# === Camera Setup ===
picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
picam.set_controls({"AwbMode": 1})  # Lock white balance
picam.start()
time.sleep(2)

print("📡 Live Classification Running (press Ctrl+C to stop)...")

# === Parameters ===
box_size = 180
conf_threshold = 65.0

try:
    while True:
        start = time.time()
        frame = picam.capture_array()
        preview = frame.copy()

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        half = box_size // 2

        # === Crop center box
        crop = frame[cy-half:cy+half, cx-half:cx+half]
        img = Image.fromarray(crop).convert("RGB").resize(input_size)
        input_tensor = np.array(img, dtype=np.float32) / 255.0  # Shape: [180,180,3]

        # === Run Inference
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        pred_index = int(np.argmax(output))
        pred_class = class_names[pred_index]
        confidence = float(np.max(output)) * 100

        if confidence >= conf_threshold:
            activate_led(pred_class)
            label_text = f"{pred_class} ({confidence:.1f}%)"
        else:
            label_text = "..."
            led_overcooked.off()
            led_raw.off()
            led_standard.off()

        # === Display result
        cv2.rectangle(preview, (cx-half, cy-half), (cx+half, cy+half), (0, 255, 0), 2)
        cv2.putText(preview, label_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("Live Copra Classification", preview)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        fps = 1 / (time.time() - start)
        print(f"⚡ FPS: {fps:.2f}    Prediction: {label_text}", end='\r')
        print("📐 Expected input shape:", input_details[0]['shape'])  # Should be [1, 180, 180, 3]


except KeyboardInterrupt:
    print("\n🛑 Stopped by user.")

finally:
    led_overcooked.off()
    led_raw.off()
    led_standard.off()
    picam.stop()
    picam.close()
    cv2.destroyAllWindows()