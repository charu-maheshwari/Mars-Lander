# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt


# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)


def EulerIntegration():
    
    x = 0
    v = 1

    # initialise empty lists to record trajectories
    x_list = []
    v_list = []
    # Euler integration
    for t in t_array:
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)
        # calculate new position and velocity
        a = -k * x / m
        x = x + dt * v
        v = v + dt * a
    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    return x_array, v_array



def VerletIntegration():

    x_list = []
    v_list = []
    
    v = 1
    x_dtt = - v * dt
    x = 0
    
    for t in t_array:
        
        x_tdt = 2 * x - x_dtt + dt**2 * (-k * x / m) 
        v = ( x_tdt - x_dtt ) / ( 2 * dt )

        x_list.append(x)
        v_list.append(v)

        x_dtt = x
        x = x_tdt

        
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    return x_array, v_array





x_array_EI , v_array_EI = EulerIntegration()
x_array_VI , v_array_VI = VerletIntegration()


# plot the position-time graph
fig, axes = plt.subplots(2, 1)
axes[0].set_title('Euler integration')
axes[0].plot(t_array, x_array_EI, label='x (m)')
axes[0].plot(t_array, v_array_EI, label='v (m/s)')
axes[0].set_xlabel('time (s)')
axes[0].grid()
axes[0].legend()

axes[1].set_title('Verlet integration')
axes[1].plot(t_array, x_array_VI, label='x (m)')
axes[1].plot(t_array, v_array_VI, label='v (m/s)')
axes[1].set_xlabel('time (s)')
axes[1].grid()
axes[1].legend()

plt.tight_layout(h_pad = 0.05)
plt.show()