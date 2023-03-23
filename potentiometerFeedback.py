##potentiometerFeedback
##design of potentiometer feedback controller
##simple pid
##potentiometer
import numpy as np
import matplotlib.pyplot as plt
import control.matlab as c
#2 in/s speed
#control speed to converge on position
t=0
pos=1
dt=0.01 #represents clock speed
while 0:
    t=t+dt
    ref=np.sin(5*t)
    speed=1
    #proportional control
    s=(pos-ref)
    pos=pos-(s*dt)


# Initialize time, position, and step size
t = 0
pos = 1
dt = 0.01

# Create empty lists to store data
time = []
position = []
reference = []
error = []

# Set up plot
fig, ax = plt.subplots(1, 1)

# Main loop
while True:
    # Increment time and calculate reference signal
    t += dt
    ref = np.sin(5 * t)

    # Calculate error and update position
    error.append(pos - ref)
    pos = pos - error[-1] * dt

    # Store data
    time.append(t)
    position.append(pos)
    reference.append(ref)

    # Clear plot and draw updated data
    ax.clear()
    ax.plot(time, position, label='Position')
    ax.plot(time, reference, label='Reference')
    ax.plot(time, error, label='Error')
    ax.legend()
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.set_title('Position, Reference, and Error vs Time')
    plt.pause(0.001)

    

