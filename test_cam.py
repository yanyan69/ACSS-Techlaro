from picamera2 import Picamera2, Preview
import time

try:
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(main={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start_preview(Preview.NULL)  # Use Preview.QTGL for monitor
    picam2.start()
    print("Camera started")
    time.sleep(5)  # Wait 5 seconds
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    picam2.capture_file(f"test_{timestamp}.jpg")
    print(f"Image captured: test_{timestamp}.jpg")
    picam2.stop()
except Exception as e:
    print(f"Camera error: {e}")