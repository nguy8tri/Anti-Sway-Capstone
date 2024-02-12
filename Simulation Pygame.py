# -*- coding: utf-8 -*-
"""
Created on Mon Jan 29 23:14:51 2024

@author: malac
"""

import numpy as np
import matplotlib.pyplot as plt
import pygame
import sys
#import scipy as sc
import Fetch as FE

def pygame_pendulum_animation(time, theta, string_length, x, f):
    #pixel to length ratio:
    ptlr = 25
    pygame.init()
    string_length *= -1*ptlr

    # Parameters
    width, height = 800, 600
    apex_color = (0, 0, 0)  # Black color for the apex
    pendulum_color = (255, 0, 0)  # Red color for the pendulum

    # Pygame setup
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    xyz = (width // 2, height // 4)  # Apex position in the middle of the screen

    # Main loop
    for t, angle, x0, F0 in zip(time, theta, x, f):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        apex_position = ((width // 2)+int(x0*ptlr), height // 4)  # Apex position in the middle of the screen
        vector_end = ((width // 2)+int(x0*ptlr)+F0*ptlr/(M1), height // 4)  # Apex position in the middle of the screen

        # Convert polar coordinates to Cartesian coordinates for the pendulum
        pendulum_x = int(apex_position[0] + string_length * np.sin(-1*angle))
        pendulum_y = int(apex_position[1] - string_length * np.cos(-1*angle))


        screen.fill((255, 255, 255))  # White background
        pygame.draw.circle(screen, (255,0,255), xyz, 5)  # Draw the apex as a small black dot
        pygame.draw.circle(screen, apex_color, apex_position, 5)  # Draw the apex as a small black dot
        pygame.draw.circle(screen, pendulum_color, (pendulum_x, pendulum_y), 15)  # Draw the pendulum as a red circle
        pygame.draw.line(screen, 'black', (pendulum_x, pendulum_y),(apex_position),1)
        pygame.draw.line(screen, 'green', (apex_position),(vector_end),2)


        pygame.display.flip()
        clock.tick(2*(simulation_speed/5)/dt)  # Adjust the frame rate
    pygame.quit()

def get_force():
    m = 1
    T = 2*np.pi*np.sqrt(l/g)
    if DISTANCE:
        A = theta_max*np.pi*g/(2*180)
        ta = track_destination/(m*A*T)
        tc = m*T-ta
    else:
        A = 2*velocity_set/T
        ta = T/4
        tc = T/4
    print(f"ta: {ta}")
    print(f"tc: {tc}")
    print(f"T: {T}")
    print(f"A: {A}")
    force_out = []
    for i in range(len(time)):
        if i*dt < ta:
            force_out.append(A*M1)
        elif i*dt - ta < tc:
            force_out.append(0)
        elif i*dt - tc - ta < ta:
            if DISTANCE:
                force_out.append(-1*A*M1)
            else:
                force_out.append(A*M1)
        else:
            force_out.append(0)
    return(force_out)

def get_vset(Vset, dt, ramp):
    #piecewise to see response
    for i in range(len(Vset)):
        length = len(Vset)
        divide = int(length/5)
        if i < divide:
            Vset[i] = i*dt*ramp
        elif i < divide*2:
            Vset[i] = divide*dt*ramp
        elif i < divide*3:
            Vset[i] = -1*divide*dt*ramp
        elif i < divide*4:
            Vset[i] = -1*divide*dt*ramp + (i-3*divide)*dt*ramp
        else:
            Vset[i] = ramp*np.sin(i*dt)
    return(Vset)

        
        
####   

#parameters, physical

M0 = 10 #kg, mass of human
M1 = 100000 #kg, mass of motor
B0 = 10
B1 = 0
g = 9.81 #m/s^2
l = 4 #meter
dt = 0.01 #time step
Xi = 0# initial position of the support
Ti = 0 # initial angle
percentage = 0.4 #amount of weight to carry of persons total weight
time = np.arange(0,20,dt)

#force parameters:
DISTANCE = False #specify the distance and get there with 0 sway. else velocity mode
controller = True
Kp = 5
Ki = 2
Vset = np.zeros_like(time)
Vset = get_vset(Vset, dt, ramp=2)
anti_sway = True

theta_max = 40 #degrees, maximum sway angle
track_destination = 10 #meters, for distance mode
velocity_set = 3 #for velocity mode

#simulation parameters:
simulation_speed = 2 #1 to 10 ish

######

#F = M1*np.cos(time)
F = get_force()
F_app = np.zeros_like(time)

stored = [F, time, dt, M0, M1, B0, B1, g, l, Xi, Ti, F_app, percentage, controller, velocity_set, Kp, Ki, Vset, anti_sway]

# Run simulation
Theta, X, dX, F = FE.fetch_pendulum_mode(stored)
#Theta2, Xp, Xm = FE.fetch_tracking_mode(stored)

plt.figure()
plt.plot(time, X, label='X')
plt.plot(time, dX, label='Vel')
plt.plot(time, Vset, label='VelSet')
plt.plot(time, 3*np.array(Theta), label = '3*Theta')
plt.plot(time, np.array(F)/M1, label='F/M Motor')
plt.legend()
plt.grid()
plt.show()


pygame_pendulum_animation(time, Theta, l, X, F)








