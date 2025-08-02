import RPi.GPIO as GPIO
from time import sleep

AIN1 = 17
AIN2 = 27
PWMA = 18
STBY = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup([AIN1, AIN2, PWMA, STBY], GPIO.OUT)

pwm = GPIO.PWM(PWMA, 1000)

pwm.start(0)

def motor_fw(speed=100):
    GPIO.output(STBY, GPIO.HIGH)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(AIN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)
    print('dc motor spinning forward')
    
def motor_bw(speed=100):
    GPIO.output(STBY, GPIO.HIGH)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)
    print('dc motor moving backwards')
    
def motor_stop():
    GPIO.output(STBY, GPIO.LOW)
    pwm.ChangeDutyCycle(0)
    print('motor stops')
    
motor_fw(80)
sleep(2)
motor_bw(80)
sleep(2)
motor_stop()
sleep(2)

GPIO.cleanup()