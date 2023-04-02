##python script for capstone
import numpy as np
import scipy
R=4.303
a=2.065
L=3.1259
x2=2.625
y20=1.2385


def f1(s,theta):
    s1,s2=s
    theta1,theta2=theta
    thetad=theta1*np.pi/180
    R=4.303
    a=2.065
    L=3.1259
    x2=-2.625
    y20=1.2385
    #s1
    s1part1=(R*np.sin(thetad) -a*np.cos(thetad)-x2)**2
    s1part2=(R*np.cos(thetad)+a*np.sin(thetad)-y20-s1)**2
    s1part3=-L**2
    z1=s1part1+s1part2+s1part3
    thetad2=theta2*np.pi/180
    #s2
    s2part1=(R*np.sin(thetad2) -a*np.cos(thetad2)-x2)**2
    s2part2=(R*np.cos(thetad2)+a*np.sin(thetad2)-y20-s2)**2
    s2part3=-L**2
    z2=s2part1+s2part2+s2part3
    return(z1,z2)
theta=np.array([-15,15])
sol=scipy.optimize.root(f1,[0.25,0.25],args=theta)

print(sol.x)
def f2(theta):
    
    theta1,theta2=theta
    thetad=theta1*np.pi/180
    R=4.303
    a=2.065
    L=3.1259
    x2=-2.625
    y20=1.2385
    #s1
    
    s1=(R*np.cos(thetad)+a*np.sin(thetad)-y20)-((L**2)-(R*np.sin(thetad) -a*np.cos(thetad)-x2)**2)**0.5
    
    thetad2=theta2*np.pi/180
    #s2
    s2=(R*np.cos(thetad2)+a*np.sin(thetad2)-y20)-((L**2)-(R*np.sin(thetad2) -a*np.cos(thetad2)-x2)**2)**0.5
    return(s1,s2)
theta=np.array([-15,15])
sol=scipy.optimize.root(f1,[0.25,0.25],args=theta)

print(sol.x,f2(theta))