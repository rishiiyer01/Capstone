import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

GPIO.setup(3, GPIO.OUT)
GPIO.setup(32,GPIO.OUT)
p=GPIO.PWM(32,1)
while 1:
    GPIO.output(3,0)
    p.start(100)