import numpy as np
import control.matlab as c
I=4
leverArm=0.1 #cp-cg, center of pressure-center of gravity
CNa=37*pi/180
#putting aerodynamic plant+control in a system of form xdot=Ax+Bu, where
#x represents the state
density=1;
v=150; #roughly half the speed of sound, this is the freestream air speed
A=0.25*pi*(6*25.4)^2;
C1=-CNa*leverArm*(0.5*density*v^2)/I;
C2=-CNa*(leverArm^2)*(0.5*density*v)/I;
A=np.array([[0, 0, 1, 0], [0, 0, 0, 1], [C1, 0, C2, 0], [0,C1, 0, C2]]); 

