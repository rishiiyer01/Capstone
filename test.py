import RPi.GPIO as GPIO


# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola
# License: Public Domain
import time

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
GPIO.setup(40,GPIO.OUT)
GPIO.output(40,1)
GPIO.setup(3, GPIO.OUT)
GPIO.setup(32,GPIO.OUT)
p=GPIO.PWM(32,1)
# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)

print("Raw ADC Value: ", chan.value)
print("ADC Voltage: " + str(chan.voltage) + "V")
while True:

    print('Raw ADC Value: ', chan.value)
    print('ADC Voltage: ' + str(chan.voltage) + 'V')
    time.sleep(0.5)
    

# set up GPIO pin
GPIO.setup(3, GPIO.OUT)
GPIO.setup(32,GPIO.OUT)
p=GPIO.PWM(32,1)
while 1:
    GPIO.output(3,0)
    p.start(100)


