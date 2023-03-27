import RPi.GPIO as GPIO



import time

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
GPIO.setmode(GPIO.BCM)

# create the spi bus
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
GPIO.setup(21,GPIO.OUT)
GPIO.output(21,1)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
p=GPIO.PWM(12,1000)
# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)

print("Raw ADC Value: ", chan.value)
print("ADC Voltage: " + str(chan.voltage) + "V")
while 1:
    GPIO.output(2,0)
    if chan.voltage>=1.5:
        p.stop()
    else:
        p.start(100)
    print(chan.voltage)
    



