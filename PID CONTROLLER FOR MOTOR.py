# -*- coding: utf-8 -*-
"""
Created on Sun Mar  3 23:14:35 2024

@author: asama
"""

import os                        # provides resources to interact with OS.
import control                   # provides resorces to perform CS applications.
from control.matlab import *     # provides flavour of matlab in python while working in CS applications.control.matlab is the package
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

time = 0
integral = 0
time_prev = -1e-6
e_prev = 0

def PID(Kp, Ki, Kd, setpoint, measurement):
    global time, integral, time_prev, e_prev
    # Value of offset - when the error is equal zero
    offset = 320
    # PID calculations
    e = setpoint - measurement
    P = Kp*e
    integral = integral + Ki*e*(time - time_prev)
    D = Kd*(e - e_prev)/(time - time_prev)
    # calculate manipulated variable - MV 
    MV = offset + P + integral + D
    # update stored data for next iteration
    e_prev = e
    time_prev = time
    return MV

R=0.1
L=0.005
K=1
J=0.0025
B=0.00065

# Define the system of ODEs
def system(w, t, V):
    i, w = w
    didt = -(R/L)*i - (K/L)*w + V/L # Example equations, replace with your own
    dwdt = (K/J)*i - (B/L)*w
    return [didt, dwdt]

# Initial conditions
y0 = [1, 8]

# Time points to solve the ODEs at
t = np.linspace(0, 10, 100)
V=12
# Solve the ODEs
sol = odeint(system, y0, t, args=(V,))
print(sol[-1][1])

# Plot the solutions
plt.plot(t, sol[:, 0], label='i(t)')
plt.plot(t, sol[:, 1], label='w(t)')
plt.xlabel('Time')
plt.ylabel('Speed')
plt.legend()
plt.grid(True)
plt.title('Solution of Two Variable ODEs')
plt.show()

a = PID(0.6,0.9,0.1, 10, 8),
print(a)

# number of steps
n = 250
time_prev = 0
y0 = [1,8]
deltat = 0.1
y_sol = [y0[-1]]
t_sol = [time_prev]
# V is chosen as a manipulated variable
V = 12,
q_sol = [V[0]]
setpoint = 10
integral = 0
for i in range(1, n):
    time = i * deltat
    tspan = np.linspace(time_prev, time, 10)
    V = PID(0.6, 0.9, 0.1, setpoint, y_sol[-1]),
    yi = odeint(system, y0, tspan, args=(V,))
    t_sol.append(time)
    y_sol.append(yi[-1][1])
    q_sol.append(V[0])
    time_prev = time
    plt.plot(t_sol, y_sol)
    plt.xlabel('Time')
    plt.ylabel('Temperature')