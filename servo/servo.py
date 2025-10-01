import serial
import time

# Open serial port (check /dev/ttyACM0 or /dev/ttyUSB0 on your Pi)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # wait for Arduino reset

print("Sending S command to Arduino...")
ser.write(b'S')   # send servo test command
time.sleep(0.5)

while ser.in_waiting:
    line = ser.readline().decode('utf-8').strip()
    print("Arduino says:", line)

ser.close()
