flow chart
START SYSTEM

# ================= INIT =================
1. Initialize Raspberry Pi
   - Load libraries: OpenCV, NumPy, PySerial, lgpio, Picamera2, Ultralytics (YOLO)
   - Setup serial connection to Arduino
   - Load trained YOLO model (v8 or v11) from file
   - Prepare CSV/log file for sensor + image data
   - Start camera (Picamera2) with fixed resolution & frame rate

2. Initialize Arduino (pins already mapped)
   - Motor driver pins (STBY, AIN1, AIN2, PWMA)
   - Servo pin (PWM)
   - AS7263 sensor (SDA, SCL, RST)
   - IR sensor pin

# ================= MAIN LOOP =================
WHILE system is running:

   # --- IR Detection ---
   IF IR sensor detects copra presence:
       SEND "IR" to RPi (or poll in RPi code)
       # This triggers image + spectral acquisition

       # --- Image Capture + Classification ---
       CAPTURE image from Pi Camera
       RUN YOLO model on image
       DETERMINE copra class (raw / standard / overcooked) based on visual features
       STORE detection confidence & bounding box if needed

       # --- AS7263 Spectral Reading ---
       SEND "REQ_AS" to Arduino
       WAIT for AS7263 readings (6 channels)
       COMPUTE moisture index (CH_R or regression)
       CLASSIFY moisture using thresholds:
           IF moisture <= 5.9% → overcooked
           ELSE IF 6.0% <= moisture <= 7.0% → standard
           ELSE IF moisture >= 7.1% → raw
           ELSE → unknown

       # --- Combine Results ---
       OPTIONALLY combine YOLO & moisture classification
           - If mismatch, decide priority (e.g., moisture takes precedence)

       # --- Servo Sorting ---
       DETERMINE servo position based on classification
           overcooked → left
           standard → center
           raw → right
       SEND "SORT,L/C/R" command to Arduino
       WAIT for ACK

       # --- Motor Control (optional) ---
       - Turn motor ON while copra is on conveyor
       - Optionally stop conveyor until servo completes sorting

       # --- Logging ---
       LOG timestamp, YOLO class, moisture value, final class, sensor readings

   ELSE:
       # Conveyor idle
       WAIT small delay (e.g., 100ms)

# ================= SHUTDOWN =================
ON keyboard interrupt or shutdown:
   - Close serial connection
   - Stop motor
   - Stop servo at neutral position
   - Close camera & release resources
   - Save log file
   - Exit



# ================= CHECKLIST =================
okay na 1. finish gathering datasets for image processing
2. then datasets for as7263, process it via code
3. label, 
4. train in google colab
5. sample run for only image processing
6. sample run for only moisture
7. combine along with everything else