% %capstone SIM+CONTROL 
%to be used in raspb pi, this script is explicitly to compute the
%Controller matrix that expresses the dynamics
%everything will be in SI unit
clc; clear; close all;
%%FOR INVERTED PENDULUM 
%%FOR INVERTED PENDULUM
%%FOR INVERTED PENDULUM 
%%FOR INVERTED PENDULUM

I=4;
leverArm=0.1; %%cp-cg, center of pressure-center of gravity
CNa=37*pi/180;
%%putting aerodynamic plant+control in a system of form xdot=Ax+Bu, where
%%x represents the state
density=1;
v=150; %%roughly half the speed of sound, this is the freestream air speed
A=0.25*pi*(6*25.4)^2;
%C1=-CNa*leverArm*(0.5*density*v^2)/I;
C1=14.61;
C2=-CNa*(leverArm^2)*(0.5*density*v)/I;
A=[0 0 1 0; 0 0 0 1; C1 0 C2 0; 0 C1 0 C2]; 
%%note that damping is basically miniscule compared to spring constant
%%calling it spring constant because this is basically just a mass spring
%%differential equation
%%our u input here is the angle, we will have to change this to represent
%%the actual actuation of the piston, AJ i can explain this in person
leverArmMotor=0.25;
Thrust=98;%N
%torque is then Thrust*LeverArmMotor*theta, theta is in our state
%recall that our state vector is [theta1,theta2,thetaDot1,thetaDot2]
C3=Thrust*leverArmMotor/I;
%B matrix in the dynamical system
%B=[0 0; 0 0; C3 0; 0 C3];
%B with velocities
td=1.56/1.44/2;
B=[0 0 0 0; 0 0 0 0; C3 0 td 0; 0 C3 0 td];
%%
Bchopped=[C3 0 0 0; 0 C3 0 0];
%%%from rishi Jan 16 2022, the B matrix that determines the optimal control
%%tf had to be edited such that it was clear that only the angular output
%%could be changed into angular acceleration via the motor's thrust
%%defining the Q and R matrices, making them default for now, effectively
%%these are the quantities we play with for tuning, where R defines the
%%weight of our actuators on the optimization function, and Q defines the
%%weight of how close we have to be to the reference (0 angle, 0 angular
%%velocity) We can talk more on this
Q=[1 0 0 0; 0 1 0 0; 0 0 10 0; 0 0 0 10];
R=Q;
%R=Q because i was lazy

C=eye(4,4);
D=zeros(4,4);
E=zeros(4,4);
%DelayT=struct('delay',td,'a',E,'b',B,'c',E,'d',D);


%sys=delayss(A,D,C,D,DelayT);
%NU = [1];
%NY = [1];
%NINT = [1];
%sysPade=pade(sys,NU,NY,NINT);

[K1,S1,P1] = lqr(A,B,Q,R);


sys=ss(A-B*K1,B,C,D);
step(sys)

