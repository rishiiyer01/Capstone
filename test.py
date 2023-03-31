import RPi.GPIO as GPIO


#252 255 217 255 223 255 41 2 223 255 242 5 255 255 251 255 0 0 232 3 98 5
#234 255 226 255 228 255 239 1 48 255 239 7 0 0 251 255 1 0 232 3 248 5
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
#sensor.mode=adafruit_bno055.CONFIG_MODE
print(sensor.mode)
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
print(sensor.calibration_status)
##def calibration
sensor.offsets_gyroscope=(0,-5,1)
sensor.offsets_accelerometer=(-22,-30,-28)
sensor.offsets_magnetometer=(389,-211,2032)
sensor.radius_accelerometer=1000
sensor.radius_magnetometer=1528

#sensor.mode=adafruit_bno055.CONFIG_MODE 
sensor._write_register(0x55,sensor._read_register(0x55))
print(sensor._read_register(0x55))
#sensor.mode=adafruit_bno055.NDOF_MODE
while 1:
    GPIO.output(4,1)
    GPIO.output(26,1)
    if chan.voltage>=0.55:
        p.stop()
    else:
        p.start(100)
        p2.start(100)
    #print(chan.voltage,chan2.voltage,sensor.euler)
    print(sensor.calibration_status,sensor.offsets_gyroscope, sensor.offsets_accelerometer,sensor.offsets_magnetometer,sensor.radius_accelerometer,sensor.radius_magnetometer)
    print(sensor._read_register(0x55),sensor._read_register(0x56),sensor._read_register(0x57),sensor._read_register(0x58),sensor._read_register(0x59),sensor._read_register(0x5A),sensor._read_register(0x5B),sensor._read_register(0x5C),sensor._read_register(0x5D),sensor._read_register(0x5E),sensor._read_register(0x5F),sensor._read_register(0x60),sensor._read_register(0x61),sensor._read_register(0x62),sensor._read_register(0x63),sensor._read_register(0x64),sensor._read_register(0x65),sensor._read_register(0x66),sensor._read_register(0x67),sensor._read_register(0x68),sensor._read_register(0x69),sensor._read_register(0x6A))
    if sensor.calibration_status==(3,3,3,3):
        sensor.mode=adafruit_bno055.CONFIG_MODE
        print(sensor._read_register(0x55),sensor._read_register(0x56),sensor._read_register(0x57),sensor._read_register(0x58),sensor._read_register(0x59),sensor._read_register(0x5A),sensor._read_register(0x5B),sensor._read_register(0x5C),sensor._read_register(0x5D),sensor._read_register(0x5E),sensor._read_register(0x5F),sensor._read_register(0x60),sensor._read_register(0x61),sensor._read_register(0x62),sensor._read_register(0x63),sensor._read_register(0x64),sensor._read_register(0x65),sensor._read_register(0x66),sensor._read_register(0x67),sensor._read_register(0x68),sensor._read_register(0x69),sensor._read_register(0x6A))
        sensor._write_register(0x55,sensor._read_register(0x55))
        sensor._write_register(0x56,sensor._read_register(0x56))
        sensor._write_register(0x57,sensor._read_register(0x57))
        sensor._write_register(0x58,sensor._read_register(0x58))
        sensor._write_register(0x59,sensor._read_register(0x59))
        sensor._write_register(0x5A,sensor._read_register(0x5A))
        sensor._write_register(0x5B,sensor._read_register(0x5B))
        sensor._write_register(0x5C,sensor._read_register(0x5C))
        sensor._write_register(0x5D,sensor._read_register(0x5D))
        sensor._write_register(0x5E,sensor._read_register(0x5E))
        sensor._write_register(0x5F,sensor._read_register(0x5F))
        sensor._write_register(0x60,sensor._read_register(0x60))
        sensor._write_register(0x61,sensor._read_register(0x61))
        sensor._write_register(0x62,sensor._read_register(0x62))
        sensor._write_register(0x63,sensor._read_register(0x63))
        sensor._write_register(0x64,sensor._read_register(0x64))
        sensor._write_register(0x65,sensor._read_register(0x65))
        sensor._write_register(0x66,sensor._read_register(0x66))
        sensor._write_register(0x67,sensor._read_register(0x67))
        sensor._write_register(0x68,sensor._read_register(0x68))
        sensor._write_register(0x69,sensor._read_register(0x69))
        sensor._write_register(0x6A,sensor._read_register(0x6A))
        break

