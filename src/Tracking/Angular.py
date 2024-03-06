import control
import numpy as np
import matplotlib.pyplot as plt
import scipy as sc
from SystemParams import SystemParams
from Positional import vel_cntr


# System Constants
g = 9.81

# System Parameters
M_m = 2 # about 9 lbs
l_perp = 0.5 # 1.5 ft
frac_supp = 0.3
M_p = 1
F_supp = M_p * g * frac_supp

def optimize(settling_time, overshoot):
    '''
    Optimizes Parameters for System

    :param settling_time: The desired settling time
    :param overshoot: The desired overshoot
    :return: The damping, inner, and outer loop gains
    for the system
    '''
    # The poles are at 0 and -B_m/M_m
    # The meeting point is at -B_m/(2*M_m),
    # so we must control that to control the
    # settling time. We can then introduce
    # a gain to then cause the system to have
    # a particular overshoot.
    # open_num = [1]
    # open_den = [l_perp * M_m, l_perp * B_m, 0]
    # open_sys = control.tf(open_num, open_den)

    # B_m/M_m = 2 * zeta * omega = 2 * 4 / settling_time
    B_m = 8 * M_m / settling_time

    zeta = -np.log(overshoot) / (np.pi ** 2 + np.log(overshoot) ** 2) ** 0.5

    # Desired Gain
    K_inner = (B_m / (2 * M_m * zeta)) ** 2 * M_m
    K_outer = K_inner * l_perp

    print(K_inner)
    print(K_outer)

    open_num = [1]
    open_den = [M_m, B_m, 0]
    open_sys = control.tf(open_num, open_den)
    control.root_locus(open_sys)
    plt.show()

    K_1 = 1
    K_2 = (K_1 * l_perp - K_outer + F_supp) / K_1

    return B_m, K_1, K_2

def gen_ramp_disturbance(samples, segments, X0=0):
    t = np.linspace(0, segments[-1][-1], samples)
    disturbance = [X0]
    for segment in segments:
        initial = disturbance[-1]
        i = 0
        while i + len(disturbance) != len(t) and t[i + len(disturbance)] <= segment[-1]:
            i += 1
        size = len(disturbance)
        slope = (segment[0] - initial) / i
        for j in range(size, size + i):
            disturbance.append(slope + disturbance[-1])

    assert len(t) == len(disturbance), str(len(t)) + " " + str(len(disturbance))

    return t, np.array(disturbance)


B_m, K_1, K_2 = optimize(0.5, 0.6)

print(B_m, K_1, K_2)

closed_num_reg = [-K_1*K_2]
closed_num_dist = [M_m, B_m, K_1]
closed_den = [l_perp * M_m, l_perp * B_m, K_1 * l_perp - K_1*K_2 + F_supp]

sys_reg : control.TransferFunction = control.tf(closed_num_reg, closed_den)
sys_dist : control.TransferFunction = control.tf(closed_num_dist, closed_den)

print(sys_reg.poles())

T = 5
samples = 100
X0 = 0.0
x_lim = (0.5588 ** 2 + 0.6096 ** 2) ** 0.5
v_lim = 1.5

# Theta ref of three degrees lead
theta_ref = -np.deg2rad(0) * np.ones(samples)
t, x_ref = gen_ramp_disturbance(samples, [[x_lim, x_lim/v_lim], [x_lim, 5], [0, 5.0 + x_lim/v_lim]])

data_reg : control.TimeResponseData = control.forced_response(sys_reg, t, theta_ref, X0)
data_dist : control.TimeResponseData = control.forced_response(sys_dist, t, x_ref, X0)

plt.figure()
plt.title("Regulator Response")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.plot(data_reg.t, np.rad2deg(data_reg.u[0]), label="Regulator")
plt.plot(data_reg.t, np.rad2deg(data_reg.y[0]), label="Regulator Response")
plt.legend()

# plt.figure()
plt.title("Disturbance Response")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.plot(data_dist.t, data_dist.u[0], label="Disturbance (m)")
plt.plot(data_dist.t, np.rad2deg(data_dist.y[0]), label="Disturbance Response")
plt.legend()

# plt.figure()
plt.title("Combined Response")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.plot(data_reg.t, np.rad2deg(data_reg.y[0] + data_dist.y[0]), label="Combined Response")
plt.legend()


theta_real = data_reg.y[0] + data_dist.y[0]
vel_real : np.ndarray = np.diff(np.array(x_ref) - theta_real * l_perp) / np.diff(t)

vel_real = np.append(vel_real, [0.0])

theta_diff = theta_ref - theta_real
pos_diff = theta_diff * K_2 - theta_real * l_perp
F_m = K_1 * pos_diff - B_m * vel_real

plt.figure()
plt.title("Force Response")
plt.xlabel("Time (s)")
plt.ylabel("Relative Force (N/kg)")
plt.plot(data_reg.t[:-1], F_m[:-1] / M_m, label="Combined Response")
plt.legend()
plt.show()

print(np.rad2deg(theta_real[-1]))
