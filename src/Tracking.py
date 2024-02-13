import numpy as np
import control
import matplotlib.pyplot as plt
import scipy.integrate as si

# System Constants
g = 9.81

# System Parameters
M_m = 10
l_perp = 2
B_m = 30
frac_supp = 0.5
M_p = 60
F_supp = M_p * g * frac_supp

## Begin Compensation Design 1: Proportional Controller with Theta_ref

# Open-Loop System
open_num_theta = [-1]
open_den = [M_m*l_perp, B_m*l_perp, F_supp]

gh : control.TransferFunction = control.tf(open_num_theta, open_den)
control.root_locus(gh)
plt.show()

# Pick a nice gain
K = 294

# Get Closed Loop System
closed_num_theta = [K*l_perp, 0]
closed_num_vel = [F_supp-K]

closed_den = list(open_den)
closed_den[-1] -= K

theta_tf = control.tf(closed_num_theta, closed_den)
vel_tf = control.tf(closed_num_vel, closed_den)

#Time Step the closed loop system
samples = 500
T = 1000
vel_p = 5

t = np.linspace(0, T, samples)
X0 = 10 * 1 / 294
theta_ref = np.zeros(samples)
vel_ref = np.ones(samples) * vel_p

theta_response : control.TimeResponseData = control.forced_response(theta_tf, t, theta_ref, X0)

vel_response : control.TimeResponseData = control.forced_response(vel_tf, t, vel_ref, X0)

plt.figure()
plt.ylabel("Motor Velocity")
plt.xlabel("Time (s)")

plt.plot(theta_response.t, theta_response.y[0], label="Theta Response")
plt.plot(vel_response.t, vel_response.y[0], label="Velocity Response")

combined_vel = vel_response.y[0] + theta_response.y[0]

plt.plot(vel_response.t, combined_vel, label="Combined Response")

plt.plot(vel_response.t, vel_response.u[0], label="Person's Velocity")


plt.legend()

combined_x = si.cumulative_trapezoid(combined_vel, t, initial=0)
ref_x = si.cumulative_trapezoid(vel_ref, t, initial=0)
angle = (ref_x - combined_x) / l_perp

# plt.figure()
# plt.xlabel("Time (s)")
# plt.ylabel("Position (m)")
# plt.plot(t, combined_x, label="Combined Response")
# plt.plot(t, ref_x, label="Reference Position")
plt.plot(t, (ref_x - combined_x) / l_perp / 50, label="Rope Angle (rad / 50)")
plt.legend()

## Begin Compensation Design 2: Proportional Controller with Person's Velocity as Input

# Open-Loop System
new_open_num = [1]
new_open_den = [M_m, B_m, F_supp/l_perp]

new_gh : control.TransferFunction = control.tf(new_open_num, new_open_den)

plt.figure()
control.root_locus(new_gh)

# Pick Some Gain
K_1 = 0.001

# Define Closed-Loop System
new_num = [K_1 + F_supp/l_perp]
new_den = [M_m, B_m, F_supp/l_perp + K_1]

new_sys : control.TransferFunction = control.tf(new_num, new_den)


# Time Step Response
samples = 500
T = 10
t = np.linspace(0, T, samples)

ref_v = 5 * np.ones(samples)
ref_x = si.cumulative_trapezoid(ref_v, t, initial=0)

new_response : control.TimeResponseData = control.forced_response(new_sys, t, ref_x)

plt.figure()
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.plot(new_response.t, new_response.y[0], label="Motor Response")
plt.plot(new_response.t, new_response.u[0], label="Person Input")
plt.legend()

theta = (new_response.y[0] - new_response.u[0]) / l_perp

v_m = np.diff(new_response.y[0]) / np.diff(new_response.t)
v_p = ref_v

v_m = np.insert(v_m, 0, 0)

plt.figure()
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.plot(new_response.t, v_m, label="Motor Response")
plt.plot(new_response.t, v_p, label="Person Input")
plt.plot(new_response.t, theta, label="Rope Angle (rad)")
plt.legend()
plt.show()

