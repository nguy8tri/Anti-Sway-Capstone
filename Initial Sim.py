# -*- coding: utf-8 -*-
"""
Created on Fri Jan 26 12:31:41 2024

@author: malac
"""
import matplotlib.pyplot as plt
import numpy as np
#simulation params
global dt
dt = 0.001 #delta t for approx
time = np.arange(0,10,dt)

#parameters, physical
Mm = 2 #kg, mass of motor
Mp = 60 #kg, mass of human
B = 0.01 #Ns/m, probably small
g = 9.81 #m/s^2
uf = 0.1 #probably small
l = 1 #meter

#derived params (constant)
Fsupp = Mp*g

#inputs
Fapp = np.sin(time) #walking force input
Fm = np.zeros_like(time) #motor response force (controller will determine eventually)

#outputs, with initial conditions!
vm = [0]
theta = [0]
vp = [0]

#derived params (nonconstant)
def Ff(Fsupp, uf):
    return(Fsupp * uf)

#GOVERNING EQUATIONS
#1: Mm * vm' = -B*vm + 0*Fsupp + Fm - (Fsupp + Mm*g)*uf
#want to get vm' from known inputs. returns vm one dt in the future
def step_vm(vm, Mm, B, theta, Fsupp, Fm, g, uf):
    if (Fsupp + Mm*g)*uf > (-B*vm) + (theta*Fsupp) + Fm:
        d_vm = 0
    else:
        d_vm = (-B*vm) + (theta*Fsupp) + Fm - (Fsupp + Mm*g)*uf
    d_vm /= Mm
    return (vm + d_vm*dt)

#2: Mp * vp' = -0*Fsupp + Fapp
#returns vp one dt in the future
def step_vp(vp, Mp, theta, Fsupp, Fapp):
    d_vp = -theta*Fsupp + Fapp
    d_vp /= Mp
    return (vp + d_vp*dt)

#3: l*0'= vp - vm
#returns theta one dt in the future
def step_theta(theta, vp, vm, l):
    d_theta = (vp-vm)/l
    return(theta*d_theta*dt)

#SIMULATE LOOP
for i, t in enumerate(time):
    if not i == len(time) - 1:
        vm.append(step_vm(vm[i], Mm, B, theta[i], Fsupp, Fm[i], g, uf))
        vp.append(step_vp(vp[i], Mp, theta[i], Fsupp, Fapp[i]))
        theta.append(step_theta(theta[i],vp[i],vm[i],l))

plt.figure()
plt.plot(time, vm, label='vm')
plt.plot(time, vp, label = 'vp')
plt.legend()
plt.show()

plt.figure()
plt.plot(time, theta, label = 'theta')
plt.show()


