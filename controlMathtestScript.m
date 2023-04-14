%test script for control analysis
clc; clear; close all;
I=4;
leverArm=0.1; %%cp-cg, center of pressure-center of gravity
CNa=37*pi/180;

density=1;
v=150; 
A=0.25*pi*(6*25.4)^2;

C1=14;
C2=-CNa*(leverArm^2)*(0.5*density*v)/I;
A=[0 0 1 0; 0 0 0 1; C1 0 C2 0; 0 C1 0 C2]; 
K=[0.6 0 0 0; 0 0.6 0 0; 0 0 10 0; 0 0 10 0]; 
E=[0 0 0 0; 0 0 0 0; 25 0 0 0; 0 25 0 0]; 

leverArmMotor=0.25;
Thrust=98;%N


C3=Thrust*leverArmMotor;

td=1.56/1.44/2;
%B=[0 0 0 0; 0 0 0 0; C3 0 td 0; 0 C3 0 td];
C=eye(4,4);
D=zeros(4,4);
B=eye(4,4);
sensorNoise=[0.01;0.01;0.05;0.05];
sensorNoiseMatrix=sensorNoise*transpose(sensorNoise);



sys=ss(A-E*K,B,C,D);
pole(sys)
step(sys)
