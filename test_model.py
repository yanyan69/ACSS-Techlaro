import cv2
import time
from ultralytics import YOLO

# === SETTINGS ===
MODEL_PATH = "my_model/my_model.pt"   # or your custom model path
CONF_THRESH = 0.5            # Minimum confidence threshold

# === LOAD MODEL ===
print("Loading YOLO model...")
model = YOLO(MODEL_PATH)
labels = model.names
print("Model loaded successfully!")

# === OPEN BUILT-IN CAMERA ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("ERROR: Cannot access the camera.")
    exit()

# Optional: Set resolution
cap.set(3, 640)
cap.set(4, 480)

print("Running YOLO live detection... Press 'Q' to quit.")

# === MAIN LOOP ===
while True:
    start_time = time.time()
    ret, frame = cap.read()
    if not ret:
        print("Camera frame unavailable.")
        break

    # Run YOLO inference
    results = model(frame, verbose=False)
    detections = results[0].boxes

    # Draw detections
    for det in detections:
        conf = float(det.conf)
        if conf >= CONF_THRESH:
            cls_id = int(det.cls)
            label = labels[cls_id]
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            # Draw bounding box
            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Add label background for readability
            label_text = f"{label} {conf*100:.1f}%"
            (w, h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            y_label = max(y1, h + 10)
            cv2.rectangle(frame, (x1, y_label - h - 10), (x1 + w, y_label), color, -1)
            cv2.putText(frame, label_text, (x1, y_label - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    # Display FPS
    fps = 1 / (time.time() - start_time)
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Show live video
    cv2.imshow("YOLO Live Detection (Laptop Cam)", frame)

    # Press 'Q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === CLEANUP ===
cap.release()
cv2.destroyAllWindows()
print("Stopped.")
