import cv2
import time
import numpy as np
from PIL import Image
from gpiozero import LED
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2

# === LED Setup ===
led_raw = LED(27)
led_standard = LED(22)
led_overcooked = LED(17)

def activate_led(label):
    led_raw.off()
    led_standard.off()
    led_overcooked.off()
    if label == 'raw':
        led_raw.on()
    elif label == 'standard':
        led_standard.on()
    elif label == 'overcooked':
        led_overcooked.on()

# === Load TFLite Model ===
interpreter = Interpreter(model_path='copra_classifier/models/copra_model.tflite')
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

class_names = ['overcooked', 'raw', 'standard']
input_size = (180, 180)

# === PiCamera2 Setup ===
picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
picam.start()
time.sleep(2)

print("🔍 Live Object Classifier Running...")

try:
    while True:
        frame = picam.capture_array()
        preview = frame.copy()

        # === Detect copra (basic brown color segmentation) ===
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_brown = np.array([10, 60, 20])
        upper_brown = np.array([30, 255, 200])
        mask = cv2.inRange(hsv, lower_brown, upper_brown)

        # Debug view
        cv2.imshow("HSV Mask", mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            print(f"[Detection] Bounding box: (x={x}, y={y}, w={w}, h={h}) - Area: {area}")

            if area > 1000:
                roi = frame[y:y+h, x:x+w]
                cv2.rectangle(preview, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Check HSV at center of ROI
                center_hsv = hsv[y + h//2, x + w//2]
                print(f"[HSV @ center] H: {center_hsv[0]}, S: {center_hsv[1]}, V: {center_hsv[2]}")

                # Classify cropped ROI
                img = Image.fromarray(roi).convert('RGB').resize(input_size)
                input_tensor = np.expand_dims(np.array(img, dtype=np.float32) / 255.0, axis=0)

                interpreter.set_tensor(input_details[0]['index'], input_tensor)
                interpreter.invoke()
                output = interpreter.get_tensor(output_details[0]['index'])[0]

                pred_index = int(np.argmax(output))
                label = class_names[pred_index]
                confidence = float(np.max(output)) * 100
                print(f"[Classification] {label} ({confidence:.2f}%)")

                activate_led(label)

                # Show label on image
                cv2.putText(preview, f'{label} ({confidence:.1f}%)', (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                print(f"[Skipped] Detected contour too small: Area = {area}")
        else:
            print("[Detection] No contours found.")
            activate_led(None)



        # === Show the frame ===
        cv2.imshow("Copra Object Classifier", preview)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Stopped by user.")

finally:
    led_raw.off()
    led_standard.off()
    led_overcooked.off()
    picam.stop()
    picam.close()
    cv2.destroyAllWindows()
