#test3
import RPi.GPIO as GPIO


#252 255 217 255 223 255 41 2 223 255 242 5 255 255 251 255 0 0 232 3 98 5
#234 255 226 255 228 255 239 1 48 255 239 7 0 0 251 255 1 0 232 3 248 5
import time
from time import process_time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import adafruit_bno055
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(2, GPIO.OUT)
#GPIO.setup(3,GPIO.OUT)
#while 1:
    #GPIO.output(2,1)
    #GPIO.output(3,1)
#GPIO.setup()
#GPIO.output()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
