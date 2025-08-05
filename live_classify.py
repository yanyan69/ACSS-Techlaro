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

        # Show mask window (debug)
        cv2.imshow("HSV Mask", mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)

            if w * h > 1000:
                # Draw detection box
                cv2.rectangle(preview, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Print HSV value at center of detected region
                center_x, center_y = x + w // 2, y + h // 2
                hsv_pixel = hsv[center_y, center_x]
                print(f"🎯 Detected area HSV @ ({center_x},{center_y}): {hsv_pixel}")

                # Crop ROI and classify
                roi = frame[y:y+h, x:x+w]
                img = Image.fromarray(roi).convert('RGB').resize(input_size)
                input_tensor = np.expand_dims(np.array(img, dtype=np.float32) / 255.0, axis=0)

                interpreter.set_tensor(input_details[0]['index'], input_tensor)
                interpreter.invoke()
                output = interpreter.get_tensor(output_details[0]['index'])[0]

                pred_index = int(np.argmax(output))
                label = class_names[pred_index]
                confidence = float(np.max(output)) * 100

                # LED + Label
                activate_led(label)
                cv2.putText(preview, f'{label} ({confidence:.1f}%)', (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                print(f"🧠 Class: {label} | Confidence: {confidence:.2f}%\n")

        else:
            print("⚠️  No object detected in current frame.")
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
