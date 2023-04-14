import numpy as np
import control.matlab as c
import keyboard
import scipy.optimize
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
s1=0.8
s2=0.6
sol=scipy.optimize.root(f3,[0,0],args=(s1,s2))
theta1,theta2=sol.x
print(theta1)