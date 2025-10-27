#!/usr/bin/env python3
"""
Standalone demo for YOLO classification and sorting application.
- Loads model, captures from Picamera2, runs tracking.
- Classifies and "sorts" leading objects by printing actions.
- Press 'q' to quit.
"""

import time
import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

# User settings (same as original)
YOLO_MODEL_PATH = "my_model/my_model.pt"
TRACKER_PATH = "bytetrack.yaml"  # Download from Ultralytics if needed
CLASS_TO_SORT = {0: 'L', 1: 'C', 2: 'R'}  # Class ID to sort direction

# Camera settings (exact match)
CAM_SIZE = (640, 480)

def main():
    # Load YOLO model
    try:
        yolo = YOLO(YOLO_MODEL_PATH)
        print(f"YOLO loaded: {YOLO_MODEL_PATH}")
    except Exception as e:
        print(f"YOLO load failed: {e}")
        return

    # Initialize camera
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": CAM_SIZE}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1)  # Warm-up
        print("Camera started with config: 640x480, XRGB8888")
    except Exception as e:
        print(f"Camera start failed: {e}")
        return

    # Tracking variables (same as original)
    process_running = True  # Simulate "process started"
    sort_cooldown = 0
    sorted_tracks = set()
    last_cleanup = time.time()
    frame_counter = 0
    fps_time = time.time()
    debug_tracking = True  # Same as main code

    print("Demo running: Detection active. Press 'q' to quit.")

    while True:
        try:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            # YOLO tracking (exact params)
            results = yolo.track(
                source=frame,
                persist=True,
                tracker=TRACKER_PATH,
                conf=0.25,
                max_det=5,
                verbose=False
            )
            annotated_frame = results[0].plot()  # Draw boxes + track IDs

            # Classification and application (sorting logic, exact match)
            current_time = time.time()
            if process_running and results[0].boxes is not None and len(results[0].boxes) > 0 and current_time > sort_cooldown:
                track_ids = results[0].boxes.id.cpu().numpy() if results[0].boxes.id is not None else np.array([])
                boxes = results[0].boxes.xyxy.cpu().numpy()
                cls_tensor = results[0].boxes.cls.cpu().numpy()
                conf_tensor = results[0].boxes.conf.cpu().numpy()

                candidates = []
                for i in range(len(cls_tensor)):
                    track_id = int(track_ids[i]) if len(track_ids) > i else -1
                    if track_id not in sorted_tracks and track_id != -1:
                        y_center = (boxes[i, 1] + boxes[i, 3]) / 2
                        cls = int(cls_tensor[i])
                        conf = conf_tensor[i]
                        if conf > 0.25:
                            candidates.append((track_id, cls, conf, y_center, i))

                if candidates:
                    candidates.sort(key=lambda x: x[3])  # Sort by y_center (ascending)
                    leading_track = candidates[0]
                    track_id, cls, conf, y_center, _ = leading_track
                    sort_char = CLASS_TO_SORT.get(cls, 'C')
                    print(f"Tracked ID {track_id} (y-center {y_center:.1f}): class {cls} conf {conf:.2f}, sorting {sort_char}")  # GUI log equivalent
                    if debug_tracking:
                        print(f"Leading track {track_id} at y={y_center:.1f} (class {cls}, conf {conf:.2f}) -> {sort_char}")  # Matches main code's console.comment
                    sorted_tracks.add(track_id)
                    sort_cooldown = current_time + 2.0  # Debounce
                elif debug_tracking:
                    print(f"No new unsorted tracks this frame (total detections: {len(cls_tensor)})")

                # Periodic cleanup
                if current_time - last_cleanup > 10.0:
                    if len(results[0].boxes) == 0:
                        sorted_tracks.clear()
                        if debug_tracking:
                            print("Cleared stale tracks (empty frame)")
                    last_cleanup = current_time

            # FPS overlay (exact)
            frame_counter += 1
            if frame_counter >= 10:
                fps = 10 / (time.time() - fps_time)
                fps_time = time.time()
                cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                frame_counter = 0

            # Display
            cv2.imshow("YOLO Demo", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Loop error: {e}")

    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
    print("Demo stopped.")

if __name__ == "__main__":
    main()