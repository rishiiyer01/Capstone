import RPi.GPIO as GPIO



import time

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import adafruit_bno055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
GPIO.setmode(GPIO.BCM)

# create the spi bus
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
#actuator 1
GPIO.setup(4, GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
p=GPIO.PWM(12,1000)
#actuator 2
GPIO.setup(26,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
p2=GPIO.PWM(13,1000)


# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)
cs2=digitalio.DigitalInOut(board.D6)
# create the mcp object
mcp = MCP.MCP3008(spi, cs)
mcp2=MCP.MCP3008(spi,cs2)
# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)
chan2=AnalogIn(mcp2, MCP.P0)

print("Raw ADC Value: ", chan.value)
print("ADC Voltage: " + str(chan.voltage) + "V")
while 1:
    GPIO.output(4,0)
    GPIO.output(26,0)
    if chan.voltage>=1.5:
        p.stop()
    else:
        p.start(100)
        p2.start(100)
    print(chan.voltage,chan2.voltage)
    



