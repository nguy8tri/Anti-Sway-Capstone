import control
import numpy as np
import matplotlib.pyplot as plt
import scipy as sc
import scipy.integrate
from typing import List, Tuple


# System Constants
g = 9.81

# System Parameters
M_m = 2 # about 9 lbs
l_perp = 0.5 # 1.5 ft
frac_supp = 0.3
M_p = 1
F_supp = M_p * g * frac_supp

# Computer-World Interface Parameters
r = 0.003
K_m = 0.11
K_a = 0.41
K = K_m * K_a

def optimize(settling_time : float, overshoot : float) -> Tuple[float, float, float]:
    '''
    Optimizes Parameters for System

    :param settling_time: The desired settling time
    :param overshoot: The desired overshoot, between 0.0
    and 1.0
    :return: The damping, inner, and outer loop gains
    for the system
    '''
    # The poles are at 0 and -B_m/M_m
    # The meeting point is at -B_m/(2*M_m),
    # so we must control that to control the
    # settling time. We can then introduce
    # a gain to then cause the system to have
    # a particular overshoot.
    B_m = 8 * M_m / settling_time

    zeta = -np.log(overshoot) / (np.pi ** 2 + np.log(overshoot) ** 2) ** 0.5

    # Desired Gains
    K_inner = (B_m / (2 * M_m * zeta)) ** 2 * M_m
    K_outer = K_inner * l_perp

    K_1 = 1
    K_2 = (K_1 * l_perp - K_outer + F_supp) / K_1

    return B_m, K_1, K_2

def gen_ramp_disturbance(samples : int, segments : List[Tuple[float, float]], X0=0) -> Tuple[List[float], List[float]]:
    '''
    Generates a full disturbance according to the
    parameters
    :param samples: The number of time samples to generate
    :param segments: A list of Tuples with
        1) The final position
        2) The time to reach that position
    :param X0: [Optional] Initial Condition, which is by default 0
    :return: A tuple with the timestamp and the position of the disturbance
    '''
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

def gen_norm_vel_disturbance(velocity : float, max_dist : float, samples : int, vel_option = False) -> Tuple[List[float], List[float], List[float]]:
    '''
    Generates a normal velocity disturbance (based on Lab Report)
    that supports step and ramp velocities
    :param velocity: The maximum velocity
    :param max_dist: The maximum distance
    :param samples: The number of samples to get
    :param vel_option: [Optional] False for step velocities, true for ramp velocities
    :return: A tuple with
        1) Timestamp
        2) Disturbance Velocity
        3) Disturbance Position (Intial Position is 0)
    where if vel_option is true, a step velocity disturbance occurs in the beginning and end such that
    the disturbance travels a distance of max_dist during both those times. If vel_option if false,
    then a ramp velocity disturbance occurs with the same characterstic as just mentioned.
    '''
    ramp_time = max_dist / velocity
    if vel_option:
        ramp_time *= 2

    total_time = 3 * ramp_time

    t = np.linspace(0, total_time, samples)

    vel_ramp = [0 if vel_option else velocity]
    vel_unramp = []

    i = 0
    while t[i] < ramp_time:
        if vel_option:
            vel_ramp.append(velocity / ramp_time * t[i])
            vel_unramp.append(-vel_ramp[-1])
        else:
            vel_ramp.append(velocity)
            vel_unramp.append(-velocity)
        i += 1

    vel = vel_ramp + [0 for i in range(len(t) - 2 * i - 1)] + vel_unramp

    assert len(vel) == len(t), str(len(vel)) + " " + str(len(t))

    return t, vel, sc.integrate.cumulative_trapezoid(vel, t, initial=0)

if __name__ == "__main__":
    B_m, K_1, K_2 = optimize(0.1, 0.05)

    closed_num_reg = [-K_1*K_2]
    closed_num_dist = [M_m, B_m, K_1]
    closed_den = [l_perp * M_m, l_perp * B_m, K_1 * l_perp - K_1*K_2 + F_supp]

    sys_reg : control.TransferFunction = control.tf(closed_num_reg, closed_den)
    sys_dist : control.TransferFunction = control.tf(closed_num_dist, closed_den)

    # To test the code, change:
    # v_lim, and
    # "t, v_ref, x_ref = gen_norm_vel_disturbance(..., vel_option=<Boolean>)"

    T = 5
    samples = 100000
    X0 = 0.0
    x_lim = (0.5588 ** 2 + 0.6096 ** 2) ** 0.5
    v_lim = 0.5 # Change this to produce a different max velocity

    # Get references
    theta_ref = -np.deg2rad(0) * np.ones(samples)
    t, v_ref, x_ref = gen_norm_vel_disturbance(v_lim, x_lim, samples, vel_option=True) # Change last parameter for step or ramp

    data_reg : control.TimeResponseData = control.forced_response(sys_reg, t, theta_ref, X0)
    data_dist : control.TimeResponseData = control.forced_response(sys_dist, t, x_ref, X0)

    ## Calculate Other Primary Outputs
    # Calculate Theta and trolley position response
    theta_real = data_reg.y[0] + data_dist.y[0]
    x_real = x_ref - theta_real * l_perp
    # Calculate Trolley Velocity
    vel_real : np.ndarray = np.diff(np.array(x_ref) - theta_real * l_perp) / np.diff(t)
    vel_real = np.append(vel_real, [0.0])
    # Calculate Motor Force
    theta_diff = theta_ref - theta_real
    pos_diff = theta_diff * K_2 - theta_real * l_perp
    F_m = K_1 * pos_diff - B_m * vel_real

    ## Calculate Auxiliary Outputs
    F_real = F_m[:-1]  # Real Motor Force
    T_real = F_m[:-1]*r  # Motor Torque
    rpm_real = (60*vel_real/(2 * np.pi * r))[:-1]  # Motor Speed (rpm)
    A_real = T_real / K_m  # Motor Current
    V_real = A_real / K_a  # Motor Velocity

    # Plot Primary Outputs
    fig, axs = plt.subplots(5, figsize=(8,8))
    fig.suptitle("Tracking Mode Input/Output Responses", size=20)

    axs[0].set_ylabel("Distance (m)")
    axs[0].plot(data_dist.t, data_dist.u[0], label="Disturbance")
    axs[0].legend(loc="upper right")

    axs[1].set_ylabel("Distance (m)")
    axs[1].plot(data_dist.t, x_real, label="Trolley Position")
    axs[1].legend(loc="upper right")

    axs[2].set_ylabel("Velocity (m/s)")
    axs[2].plot(data_dist.t, v_ref, label="Disturbance")
    axs[2].legend(loc="upper right")

    axs[3].set_ylabel("Velocity (m/s)")
    axs[3].plot(data_dist.t[:-1], vel_real[:-1], label="Trolley Position")
    axs[3].legend(loc="upper right")

    axs[4].set_ylabel("Angle (deg)")
    axs[4].plot(data_reg.t, np.rad2deg(theta_real), label="Rope Angle")
    axs[4].legend(loc="upper right")

    fig, axs = plt.subplots(5, figsize=(8,8))
    fig.suptitle("Tracking Mode Auxillary Outputs", size=20)

    axs[0].set_ylabel("Force (N)")
    axs[0].plot(data_reg.t[:-1], F_real, label="Motor Force")
    axs[0].legend(loc="upper right")

    axs[1].set_ylabel("Torque (N-m)")
    axs[1].plot(data_reg.t[:-1], T_real, label="Motor Torque")
    axs[1].legend(loc="upper right")

    axs[2].set_ylabel("Angular Velocity\n(rpm)")
    axs[2].plot(data_reg.t[:-1], rpm_real, label="Motor Velocity")
    axs[2].legend(loc="upper right")

    axs[3].set_ylabel("Current (A)")
    axs[3].plot(data_reg.t[:-1], A_real, label="Motor Current")
    axs[3].legend(loc="upper right")

    axs[4].set_ylabel("Voltage (V)")
    axs[4].set_xlabel("Time (s)")
    axs[4].plot(data_reg.t[:-1], V_real, label="Motor Voltage")
    axs[4].legend(loc="upper right")

    plt.show()
