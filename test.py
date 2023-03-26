import RPi.GPIO as GPIO

# set up GPIO mode
GPIO.setmode(GPIO.BOARD)

# set up GPIO pin
GPIO.setup(3, GPIO.OUT)
GPIO.setup(32,GPIO.OUT)
p=GPIO.PWM(32,1)
while 1:
    GPIO.output(3,0)
    p.start(100)


