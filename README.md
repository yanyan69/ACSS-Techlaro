Automated Copra Segregation System developed by Team Techlaro

Team Members: Preclaro, Jerald James D., Narvaez, Christian L., Armenta, John Paul F., Maming, Marielle B.

This system aims to follow the following objectives:
1. Sort copra based on (a) color via image processing/ object detection or classification and (b) moisture content via NIR sensor or light reflected.
2. Ensure sorting speed is faster in kg/hr and more accurate than manual sorting.
3. User acceptance across multiple copra buyer in Marinduque.

Prototype Hardware Requirements:
1. Raspberry Pi 5 (Specs: Bookworm 64-bit 4GB RAM)
2. Standard Raspberry Pi Camera 3
3. Arduino Uno
4. AS7263 NIR sensor
5. 42BYGH48 Stepper Motor with DRV8825 Motor Driver
6. 3 MG996R Servo Motor
7. 12V 5A, S-60-12 Switching Power Supply
8. 12V 3900RPM, Vibration Motor
9. LM2596 DC-DC Buckdown converter

Software Requirements:
1. Python: tkinter, opencv-python, numpy, pyserial, lgpio, ultralytics, picamera2, serial
2. VSCode
3. GitHub
4. Ultralytics: yolov11n model

yolov11n model classes: {0: 'overcooked-copra', 1: 'raw-copra', 2: 'standard-copra'}

General Flow of the system:
1. Input 5k copra at most in the hopper. Hopper should vibrate here and copra should flow out one by one with a servo stopper moving down, then up.
2. Detects copra at the start of conveyor (prolly will use proximity sensor at this point), Moves copra up to the middle of camera view. This process stacks in queue if next copra immediately detected at the starting point which came out of the hopper.
3. After copra is verified in the camera, it moves next to the NIR sensor to measure its moisture content via infrared. This process needed two parameters to be the basis for copra classification for solid output. This is crucial to achieve financial gains. less misclassification
4. After verification in NIR sensor, copra then moves next to flapping zone, where copra will either be pushed left (raw-copra) or right (overcooked-copra), fast enough to send it flying to its direction or either not flap it at all to send it directly forward (standard-copra). 
5. This process should employ FIFO queuing, ensure proper alignment in the conveyor system.
