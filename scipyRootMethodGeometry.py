##python script for capstone
import numpy as np
import scipy
R=4.303
a=2.065
L=3.1259
x2=2.625
y20=1.2385
def f1(R,a,L,x2,y20,theta,s1):
    thetad=theta*np.pi/180
    #s1
    s1part1=(((R**2+a**2)**0.5)*np.sin(thetad) -x2)**2
    s1part2=(((R**2+a**2)**0.5)*np.cos(thetad) -y20+s1)**2
    s1part3=L**2
    return(s1part1+s1part2+s1part3)
def f2(R,a,L,x2,y20,theta2,s2):
    thetad=theta2*np.pi/180
    #s2
    s2part1=(((R**2+a**2)**0.5)*np.sin(thetad) -x2)**2
    s2part2=(((R**2+a**2)**0.5)*np.cos(thetad) -y20+s2)**2
    s2part3=L**2
    return(s2part1+s2part2+s2part3)

