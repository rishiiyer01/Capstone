import RPi.GPIO as GPIO
import board
GPIO.setmode(GPIO.BCM)

GPIO.setup(2, GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
p=GPIO.PWM(12,1000)
while 1:
    GPIO.output(2,1)
    p.start(100)