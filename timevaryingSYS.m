%%time-variant state space model of the LQR alg derived from LTI dynamics
clc; clear; close all;
% %def constants rn these are all wrong and probably not even right order
% of mag wise
I=1.5;
leverArm=1.778; %%cp-cg, center of pressure-center of gravity
CNa=1.71;
%%putting aerodynamic plant+control in a system of form xdot=Ax+Bu, where
%%x represents the state
density=1;
v=20; %%roughly half the speed of sound, this is the freestream air speed
Area=0.00791;
C1=CNa*leverArm*(0.5*density*v^2)/I;
C2=-((CNa*(leverArm^2)*(0.5*density*v)/I)-(0.055*0.1*0.1));
A=[0 0 1 0; 0 0 0 1; C1 0 C2 0; 0 C1 0 C2]; 
%%note that damping is basically miniscule compared to spring constant
%%calling it spring constant because this is basically just a mass spring
%%differential equation
%%our u input here is the angle, we will have to change this to represent
%%the actual actuation of the piston, AJ i can explain this in person
leverArmMotor=0.96;
Thrust=94;%N
%torque is then Thrust*LeverArmMotor*theta, theta is in our state
%recall that our state vector is [theta1,theta2,thetaDot1,thetaDot2]
C3=Thrust*leverArmMotor/I;
%B matrix in the dynamical system
B=[0 0; 0 0; C3 0; 0 C3];
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
Q=[10000 0 0 0; 0 10000 0 0; 0 0 10 0; 0 0 0 10];
R=[1 0; 0 1];
%R=Q because i was lazy
[K1,S1,P1] = lqr(A,B,Q,R);
C=eye(4,4);
D=zeros(4,2);
sys=ss(A-B*K1,B,C,D);
%step(sys)
%%pole placement experiment
K2=place(A,B,[-4,-4,-400,-400]);
sys2=ss(A-B*K2,B,C,D);
step(sys)
%%defining a function that alters the matrices A,B
%K1 is fixed
table=xlsread('rocketXL.xlsx');
velocity=table(:,2);
time=table(:,1);

thrust=table(:,3);
dt=0.001;
xstore=zeros(length(time),4);
x0=[0.25;0.25;0;0];
for i=1:length(time)
    v0=velocity(i);
    T=thrust(i);
    C1=CNa*leverArm*(0.5*density*v0^2)/I;
    C2=-((CNa*(leverArm^2)*(0.5*density*v0)/I)-(0.055*0.1*0.1));
    A=[0 0 1 0; 0 0 0 1; C1 0 C2 0; 0 C1 0 C2];
    C3=T*leverArmMotor/I;
    %B matrix in the dynamical system
    B=[0 0; 0 0; C3 0; 0 C3];
    
    %timestep
    xstore(i+1,:)=transpose(x0)+5*transpose((A-B*K1)*x0*dt);
    x0=transpose(xstore(i+1,:));
end
figure (2)
plot(time/4,xstore(2:end,1))
hold on 
x0=[0.25;0.25;0;0];
for i=1:length(time)
    
    C1=CNa*leverArm*(0.5*density*v^2)/I;
    C2=-((CNa*(leverArm^2)*(0.5*density*v)/I)-(0.055*0.1*0.1));
    A=[0 0 1 0; 0 0 0 1; C1 0 C2 0; 0 C1 0 C2];
    C3=Thrust*leverArmMotor/I;
    %B matrix in the dynamical system
    B=[0 0; 0 0; C3 0; 0 C3];
    
    %timestep
    xstore(i+1,:)=transpose(x0)+8*transpose((A-B*K1)*x0*dt);
    x0=transpose(xstore(i+1,:));
end
plot(time/4,xstore(2:end,1))
title('Simulated Time Response with Initial Conditions, LTI system vs Time Varying Nonlinear System')
xlabel('time (s)') 
ylabel('angle of attack (rad)')
legend({'time-varying nonlinear','linear time invariant'},'Location','southwest')



