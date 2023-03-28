import RPi.GPIO as GPIO
import board
GPIO.setmode(GPIO.BCM)

GPIO.setup(4, GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
p=GPIO.PWM(12,1000)
GPIO.setup(26,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
p2=GPIO.PWM(13,1000)
while 1:
    GPIO.output(4,1)
    GPIO.output(26,1)
    p.start(100)
    p2.start(100)