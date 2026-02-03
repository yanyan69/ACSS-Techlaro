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
10. 12V 0.5A 3800 RPM Vibration Motor with TB6612FNG Motor Driver

Software Requirements:
1. Python: tkinter, opencv-python, numpy, pyserial, lgpio, ultralytics, picamera2, serial
2. VSCode
3. GitHub
4. Ultralytics: yolov11n model, 40 epoch, 480px, with 1000 dataset total (2600 total when preprocessed and augmented)

yolov11n model classes: {0: 'overcooked-copra', 1: 'raw-copra', 2: 'standard-copra'}

General Flow of the system:
1. Input 5k copra at most in the hopper. Hopper should vibrate here and copra should flow down one by one with a servo stopper to control flow at 45 degrees. Then copra should flow slowly until it gets out of the V-shaped chute to make sure copra flows one by one. the vibration motor vibrates until copra is down in the conveyor entry where a proximity sensor is placed.
2. Detects copra at the start of conveyor (prolly will use proximity sensor at this point), Moves copra up to the middle of camera view. This process stacks in queue if next copra immediately detected at the starting point which came out of the hopper.
3. After copra is verified in the camera, it moves next to the NIR sensor to measure its moisture content via infrared. This process needed two parameters to be the basis for copra classification for solid output. This is crucial to achieve financial gains. less misclassification
4. After verification in NIR sensor, copra then moves next to flapping zone, where copra will either be pushed left (raw-copra) or right (overcooked-copra), fast enough to send it flying to its direction or either not flap it at all to send it directly forward (standard-copra). 
5. This process should employ FIFO queuing, ensure proper alignment in the conveyor system.

For re-defense at October 20, 2025 Checklist:
1. Add a stopper at the hopper system using servo motor. Possibly Redesign hopper. With a controlled vibration motor.
2. Add ultrasonic proximity sensor at the starting point of the conveyor system to know if copra is there or not. (or put the servo stopper in line with the proximity sensor) to control copra distance until no copra is seen by the proximity sensor. like rotate stepper and then simultaneously move servo to control the gaps/ distance of copra.
3. Use Stepper motor for precise rotation.
4. Use servo-flappers to push copra left (raw-copra) or right (overcooked-copra) or ignore copra if standard-copra which goes straight forward.
5. Fix Bins positions, redesign flapping area. redesign stopper at edge of conveyor.