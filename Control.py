import numpy as np
import control.matlab as c
import smbus
I=4
leverArm=0.1 #cp-cg, center of pressure-center of gravity
CNa=37*np.pi/180
#putting aerodynamic plant+control in a system of form xdot=Ax+Bu, where
#x represents the state
density=1;
v=150; #roughly half the speed of sound, this is the freestream air speed
A=0.25*np.pi*(6*25.4)**2;
C1=-CNa*leverArm*(0.5*density*v**2)/I;
C2=-CNa*(leverArm**2)*(0.5*density*v)/I;
A=np.array([[0, 0, 1, 0], [0, 0, 0, 1], [C1, 0, C2, 0], [0,C1, 0, C2]]); 
leverArmMotor=1
Thrust=92
C3=Thrust*leverArmMotor/I
B=[[0, 0], [0, 0], [C3, 0],[0, C3]]
Q=np.eye(4)
R=np.eye(2)
K, S, E = c.lqr(A, B, Q, R)
print(K)

