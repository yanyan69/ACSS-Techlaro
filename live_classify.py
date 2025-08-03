import os
import time
import numpy as np
from PIL import Image
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter

# === Load TFLite Model ===
model_path = 'copra_classifier/models/copra_model.tflite'
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()
print(f"\nModel loaded from: {model_path}")

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

class_names = ['overcooked', 'raw', 'standard']

# === Setup Camera ===
picam = Picamera2()
picam.configure(picam.create_still_configuration(main={"size": (640, 480)}))
picam.start()
time.sleep(2)  # Allow warm-up

print("\nLive classification started. Press Ctrl+C to stop.\n")

try:
    confidence_buffer = []
    class_buffer = []
    start_time = time.time()

    while True:
        # Capture frame
        frame = picam.capture_array()
        img = Image.fromarray(frame).convert("RGB").resize((180, 180))

        # Preprocess (normalize like in training)
        img_array = np.array(img, dtype=np.float32) / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        # Run inference
        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0]

        predicted_class = class_names[np.argmax(output)]
        confidence = 100 * np.max(output)

        confidence_buffer.append(confidence)
        class_buffer.append(predicted_class)

        # Every 10 seconds, show summary
        if time.time() - start_time > 10:
            avg_confidence = np.mean(confidence_buffer)
            final_class = max(set(class_buffer), key=class_buffer.count)

            print(f"\n[10s Summary]")
            print(f"  Class: {final_class}")
            print(f"  Avg Confidence: {avg_confidence:.2f}%\n")

            # Reset buffer
            confidence_buffer = []
            class_buffer = []
            start_time = time.time()

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    picam.stop()
    picam.close()
