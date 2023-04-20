import numpy as np
import control.matlab as c
import time
import threading
import scipy.optimize
I = 4
leverArm = 0.1  # cp-cg, center of pressure-center of gravity
CNa = 37 * np.pi / 180
# putting aerodynamic plant+control in a system of form xdot=Ax+Bu, where
# x represents the state
density = 1
v = 150;  # roughly half the speed of sound, this is the freestream air speed
A = 0.25 * np.pi * (6 * 25.4) ** 2
# C1=-CNa*leverArm*(0.5*density*v**2)/I;
C1 = 14.7
#middle case
#C1=9.898
#final stable
#C1=0.1
C2 = -CNa * (leverArm ** 2) * (0.5 * density * v) / I
Thrust = 92
leverArmMotor = 0.25
C3 = Thrust * leverArmMotor / I
A=np.array([[0, 0, 1, 0,0,0],
            [0, 0, 0, 1,0,0],
            [C1, 0, C2, 0,C3,0],
            [0,C1, 0, C2,0,C3],
            [0,0, 0, 0,0,0],
            [0,0, 0, 0,0,0]]); 


B=[[0, 0], [0, 0], [0, 0],[0, 0],[1, 0],[0, 1]]
Q=np.array([[10000,0,0,0,0,0],[0,10000,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
R=np.eye(2)
K, S, E = c.lqr(A, B, Q, R)
K[0,4]=0
K[1,5]=0
K[0,0]=100
K[0,2]=1000
K[1,1]=100
K[1,3]=1000

print(K)

import RPi.GPIO as GPIO

# 252 255 217 255 223 255 41 2 223 255 242 5 255 255 251 255 0 0 232 3 98 5
# 234 255 226 255 228 255 239 1 48 255 239 7 0 0 251 255 1 0 232 3 248 5
import time
from time import process_time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
GPIO.setmode(GPIO.BCM)
# sensor.mode=adafruit_bno055.CONFIG_MODE

# create the spi bus
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
# actuator 1
GPIO.setup(4, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(12, 1000)
# actuator 2
GPIO.setup(26, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
p2 = GPIO.PWM(13, 1000)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)
cs2 = digitalio.DigitalInOut(board.D6)
# create the mcp object
mcp = MCP.MCP3008(spi, cs)
mcp2 = MCP.MCP3008(spi, cs2)
# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)
chan2 = AnalogIn(mcp2, MCP.P0)

##def calibration
sensor.offsets_gyroscope = (0, -5, 1)
sensor.offsets_accelerometer = (-22, -30, -28)
sensor.offsets_magnetometer = (389, -211, 2032)
sensor.radius_accelerometer = 1000
sensor.radius_magnetometer = 1528

# sensor.mode=adafruit_bno055.CONFIG_MODE
sensor._write_register(0x55, sensor._read_register(0x55))

# sensor.mode=adafruit_bno055.NDOF_MODE
# def angVelocities(sensor,time):
start = time.time()
# anglevelos=angle1velo1,angle1velo2,angle1velo3,angle1velo4,angle1velo5
angvelo1 = 0
angvelo2 = 0
prevtime = start
prev1 = 0
prev2 = 0


def f2(theta):
    theta1 = float(theta[0])
    theta2 = float(theta[1])
    theta1 = max(min(theta1, np.pi / 12), -np.pi / 12)
    theta2 = max(min(theta2, np.pi / 12), -np.pi / 12)

    thetad = theta1
    R = 4.303
    a = 2.065
    L = 3.1259
    x2 = -2.625
    y20 = 1.2385
    # s1

    s1 = (R * np.cos(thetad) + a * np.sin(thetad) - y20) - (
            (L ** 2) - (R * np.sin(thetad) - a * np.cos(thetad) - x2) ** 2) ** 0.5

    thetad2 = theta2
    # s2
    s2 = (R * np.cos(thetad2) + a * np.sin(thetad2) - y20) - (
            (L ** 2) - (R * np.sin(thetad2) - a * np.cos(thetad2) - x2) ** 2) ** 0.5
    return (s1, s2)
def f3(theta,*args):
    theta1 = float(theta[0])
    theta2 = float(theta[1])
    R = 4.303
    a = 2.065
    L = 3.1259
    x2 = -2.625
    y20 = 1.2385
    L1=-s1+(R * np.cos(theta1) + a * np.sin(theta1) - y20) - (
            (L ** 2) - (R * np.sin(theta1) - a * np.cos(theta1) - x2) ** 2) ** 0.5
    L2=-s2+(R * np.cos(theta2) + a * np.sin(theta2) - y20) - (
            (L ** 2) - (R * np.sin(theta2) - a * np.cos(theta2) - x2) ** 2) ** 0.5
    return(L1,L2)




def stroke_speed(theta, theta_speed):
    # Constant
    rad = 4.303
    a = 2.065
    length = 3.1259
    x2 = -2.625

    s_dot = theta_speed * (a * np.cos(theta) - rad * np.sin(theta) + ((rad * np.sin(theta) - a*np.cos(theta) - x2) * (
                                       rad * np.cos(theta) + a * np.sin(theta)) /
                           ((length ** 2 - (rad * np.sin(theta) - a * np.cos(theta) - x2) ** 2) ** .5)))
    return s_dot

def pwm_actuator(K, state, chan, chan2, time, *args, **kwargs):
    # Assuming that this is changed to read back theta velocities
    theta_v_target = 1 * np.matmul(K, state)

    theta_v_target1 = float(theta_v_target[0])
    theta_v_target2 = float(theta_v_target[1])

    # Assumes that theta 1 and theta 2 are at indexs 4 and 5 of the state matrix
    theta_1 = float(state[4])
    theta_2 = float(state[5])

    s_dot1 = stroke_speed(theta_1, theta_v_target1)
    s_dot2 = stroke_speed(theta_2, theta_v_target2)

    max_voltage = 1.35
    min_voltage = 0.18

    max_speed = 1.4

    voltage_scalar1 = min(100, 100 * abs(s_dot1 / max_speed))
    voltage_scalar2 = min(100, 100 * abs(s_dot2 / max_speed))

    # Actuator 1
    if chan.voltage < max_voltage and s_dot1 > 0:
        GPIO.output(4, 1)
        p.start(100)
    elif chan.voltage > min_voltage and s_dot1 < 0:
        GPIO.output(4, 0)
        p.start(100)
    else:
        p.stop()

    if chan2.voltage < max_voltage and s_dot2 > 0:
        GPIO.output(26, 1)
        p2.start(100)
    elif chan2.voltage > min_voltage and s_dot2 < 0:
        GPIO.output(26, 0)
        p2.start(100)
    else:
        p2.stop()

    return [theta_v_target1, theta_v_target2]

def write_to_file(file_obj, data_to_write):
    file_obj.write(data_to_write)


file = open("./DataLogging/" + time.strftime("%Y.%m.%d-%H.%M.%S") + ".csv", 'w')

file.write("Time,Euler1,Euler2,Gyro1,Gyro2m,target_theta1,target_theta2,target_stroke1,target_stroke2\n")

cached_data = []

while 1:

    try:

        if sensor.calibration_status[1] == 3:

            # print(chan.voltage,chan2.voltage,sensor.euler)
            # print(sensor.euler)
            s1=-0.9+(chan.voltage-0.18)/0.735
            s2=-0.9+(chan2.voltage-0.18)/0.735
            sol=scipy.optimize.root(f3,[0,0],args=(s1,s2))
            theta1,theta2=sol.x
            #theta1=0
            #theta2=0
            state = np.array([[(sensor.euler[1]-0.44)*np.pi/180], [(sensor.euler[2]-1.625)*np.pi/180], [sensor._gyro[1]], [sensor._gyro[2]],[theta1],[theta2]])
            #print(state)
            coordTransform = np.array(
                [[np.cos(-np.pi / 4), -np.sin(-np.pi / 4), 0, 0,0,0], [np.sin(-np.pi / 4), np.cos(-np.pi / 4), 0, 0,0,0],
                 [0, 0, np.cos(-np.pi / 4), -np.sin(-np.pi / 4),0,0], [0, 0, np.sin(-np.pi / 4), np.cos(-np.pi / 4),0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
            state = -np.matmul(coordTransform, state)
            state[0] = -state[0]
            state[2] = -state[2]
            #print(state)
            thetas = pwm_actuator(K, state, chan, chan2, time)
            # print(sensor._read_register(0x55),sensor._read_register(0x56),sensor._read_register(0x57),sensor._read_register(0x58),sensor._read_register(0x59),sensor._read_register(0x5A),sensor._read_register(0x5B),sensor._read_register(0x5C),sensor._read_register(0x5D),sensor._read_register(0x5E),sensor._read_register(0x5F),sensor._read_register(0x60),sensor._read_register(0x61),sensor._read_register(0x62),sensor._read_register(0x63),sensor._read_register(0x64),sensor._read_register(0x65),sensor._read_register(0x66),sensor._read_register(0x67),sensor._read_register(0x68),sensor._read_register(0x69),sensor._read_register(0x6A))
            print(state)

            cached_data.append(','.join([str(time.time()), str(state[0]), str(state[1]),
                                        str(state[2]), str(state[3]), str(thetas[0]), str(thetas[1]),
                                        str(chan.voltage), str(chan2.voltage)]) + '\n')

            if len(cached_data) > 200:
                t = threading.Thread(target=write_to_file, args=(file, ''.join(cached_data)))
                cached_data = []

            continue
        else:
            print(sensor.euler, sensor.calibration_status)
            continue
    except KeyboardInterrupt:
        print('stopping')
        p.stop()
        p2.stop()
        file.write(''.join(cached_data))
        break

    #except:
     #   print('yourmom')
      #  p.stop()
       # p2.stop()

p.stop()
p2.stop()
file.close()
