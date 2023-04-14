
import RPi.GPIO as GPIO
import numpy as np

#252 255 217 255 223 255 41 2 223 255 242 5 255 255 251 255 0 0 232 3 98 5
#234 255 226 255 228 255 239 1 48 255 239 7 0 0 251 255 1 0 232 3 248 5
import time
x=np.arange(3)
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import adafruit_bno055
GPIO.setmode(GPIO.BCM)

GPIO.setup(4, GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
p=GPIO.PWM(12,1000)
GPIO.setup(26,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
p2=GPIO.PWM(13,1000)
# create the spi bus
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
print(board.SCK,board.MOSI,board.MISO)
# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)
cs2=digitalio.DigitalInOut(board.D6)
# create the mcp object
mcp = MCP.MCP3008(spi, cs)
mcp2=MCP.MCP3008(spi,cs2)


chan1 = AnalogIn(mcp, MCP.P0)
chan2=AnalogIn(mcp2, MCP.P0)

try: 
    while 1:
        
        voltage_target=0.8215
        if chan1.voltage<voltage_target:
            GPIO.output(4,1)
            p.start(100)
            print('we shouldnt be here for long')
        else:
        
            p.stop()
            print('we in')
        if chan2.voltage>voltage_target:
            GPIO.output(26,0)
            p2.start(100)
            print('we shouldnt be here for long')
        else: 
            p2.stop()
           
            print('we in')
        
        print(chan1.voltage,chan2.voltage,chan2.voltage-voltage_target)

except KeyboardInterrupt:
    print('stopping')
    p.stop()
    p2.stop()
