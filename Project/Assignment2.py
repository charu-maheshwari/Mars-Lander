# uncomment the next line if running in a notebook
# %matplotlib inline

# To change to drawing parabola or circle simply replace v_ellipse
# with the corresponding variable at the beigining of Euler and Verlet functions

import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
G = 6.674 * 1e-11
M = 6.42 * 1e23

# simulation time, timestep and time
t_max = 35
dt = 0.001
t_array = np.arange(0, t_max, dt)


# Euler integration becomes stable for t - 0.01

def Euler_integration( )  :

    v_circle = 20699.536
    v_para = 32000
    v_ellipse = 10000
    x = np.array([100000, 0 , 0])
    v = np.array([0.001, v_ellipse, 0])
    
    # initialise empty lists to record trajectories
    x_list = []
    v_list = []
    
    # Euler integration
    for t in t_array:
        
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)
        # calculate new position and velocity
        
        mag_a = - G * M / ((np.sum(np.square(x))) ** 1.5 )
        a = x * mag_a
        x = x + dt * v
        v = v + dt * a

    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    return x_array, v_array

def Verlet_integration( ):
    
    v_circle = 20699.536
    v_para = 32000
    v_ellipse = 10000 
    
    
    x_list = []
    v_list = []
    
    v = np.array([0.001, v_ellipse, 0])    
    x = np.array([100000, 0 , 0])
    x_dtt = x - v * dt
    
    for t in t_array:
        
        mag_a = - G * M / ((np.sum(np.square(x))) ** 1.5 )
        a = x * mag_a
        x_tdt = 2 * x - x_dtt + dt**2 * a 
        v = ( x_tdt - x_dtt ) / ( 2 * dt )

        x_list.append(x)
        v_list.append(v)

        x_dtt = x
        x = x_tdt

        
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    return x_array, v_array
    

x_array_EI, v_array_EI = Euler_integration()  
fig, axes = plt.subplots(2, 1)
axes[0].set_title('Euler integration')
axes[0].plot( x_array_EI[:, 0], x_array_EI[ : ,  1] , label='x (m)')
axes[0].grid()
axes[0].legend()

x_array_VI, v_array_VI = Verlet_integration()
axes[1].set_title('Verlet integration')
axes[1].plot( x_array_VI[:, 0], x_array_VI[ : ,  1] , label='x (m)')
axes[1].grid()
axes[1].legend()
plt.show()
                                                               



