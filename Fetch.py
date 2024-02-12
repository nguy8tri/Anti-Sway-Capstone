# -*- coding: utf-8 -*-
"""
Created on Tue Jan 30 10:27:39 2024

@author: malac
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Jan 26 12:31:41 2024

@author: malac
"""
import matplotlib.pyplot as plt
import numpy as np

class PIController:
    def __init__(self, kp, ki, M, dt):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.dt = dt  # Time step
        self.M = M
        self.integral = 0

    def compute(self, error):
        # Proportional term
        proportional_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        integral_term = self.ki * self.integral

        # Calculate the control signal
        control_signal = (proportional_term + integral_term)*self.M

        return control_signal

#SIMULATE LOOP
def fetch_pendulum_mode(stored):
    F, time, dt, M0, M1, B0, B1, g, l, Xi, Ti, F_app, percentage, controller, velocity_set, Kp, Ki, Vset, anti_sway = stored
    if controller:
        PI_control = PIController(Kp,Ki, M1, dt)
        F = [0]
    
    ddX = [0]; dX = [0]; X = [Xi]
    ddTheta = [0]; dTheta = [0]; Theta = [Ti]
    
    def step_X(i, F, dt):
        ddX_n = (F-(M0*l*ddTheta[i])-(B0*dX[i]))/(M0+M1)
        dX_n = ddX_n*dt + dX[i]
        X_n = dX_n*dt + X[i]
        ddX.append(ddX_n); dX.append(dX_n); X.append(X_n)

    def step_Theta(i, dt):
        ddTheta_n = (-ddX[i]-g*Theta[i]-B1*dTheta[i])/l
        dTheta_n = ddTheta_n*dt + dTheta[i]
        Theta_n = dTheta_n*dt + Theta[i]
        ddTheta.append(ddTheta_n); dTheta.append(dTheta_n); Theta.append(Theta_n)
        
        
    for i in range(len(time)):
        if not i == len(time) - 1:
            if controller:
                if anti_sway:
                    K = 2*np.sqrt(l/g)
                    F.append(PI_control.compute((Vset[i]+(K*g*Theta[i]))-dX[i]))
                else:
                    F.append(PI_control.compute(Vset[i]-dX[i]))
            step_X(i, F[i], dt)
            step_Theta(i, dt)
            
            
    return Theta, X, dX, F


#SIMULATE LOOP
def fetch_tracking_mode(stored):
    F, time, dt, M0, M1, B0, B1, g, l, Xi, Ti, F_app, percentage = stored
    
    Xmi = Xi
    Xpi = (l*np.cos(Ti)*Ti)+Xmi
    F_supp =  g*M1*percentage
    
    ddXp = [0]; dXp = [0]; Xp = [Xpi] #FILL LATER
    ddXm = [0]; dXm = [0]; Xm = [Xmi] #FILL LATER
    dTheta = [0]; Theta = [Ti]
    
    F_supp = 0
    
    def step_Xp(i):
        ddX_n = -Theta[i]*F_supp + F_app[i]
        dX_n = ddX_n*dt + dXp[i]
        X_n = dX_n*dt + Xp[i]
        ddXp.append(ddX_n); dXp.append(dX_n); Xp.append(X_n)
        
    def step_Theta(i):
        dTheta_n = (dXp[i]-dXm[i])/(l*np.cos(Theta[i]))
        Theta_n = dTheta_n*dt + Theta[i]
        dTheta.append(dTheta_n); Theta.append(Theta_n)
        
    def step_Xm(i):
        ddX_n = (-B0*dXm[i] + Theta[i]*F_supp + F[i])/M0 #neglecting friction for now
        dX_n = ddX_n*dt + dXm[i]
        X_n = dX_n*dt + Xm[i]
        ddXm.append(ddX_n); dXm.append(dX_n); Xm.append(X_n)
        
        
    for i in range(len(time)):
        if not i == len(time) - 1:
            step_Xp(i)
            step_Xm(i)
            step_Theta(i)
            
            
    return Theta, Xp, Xm
    

