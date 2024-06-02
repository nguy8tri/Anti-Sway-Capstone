import numpy as np
import control
import matplotlib.pyplot as plt
import scipy.integrate as si
from typing import Union, List, Tuple
from SystemParams import SystemParams

# System Constants
g = 9.81

# System Parameters
M_m = 10
l_perp = 2
B_m = 30
frac_supp = 0.5
M_p = 60
F_supp = M_p * g * frac_supp

## Begin Compensation Design: Proportional Controller with Person's Velocity as Input

def vel_cntr(K_1 : float, params : SystemParams, frac_supp : float, B_m : float) -> Tuple[control.TransferFunction, control.TransferFunction]:
    M_m, l_perp, F_supp = params.get_mean_male_params(frac_supp)

    # Open-Loop System
    new_open_num = [1]
    new_open_den = [M_m, B_m, F_supp/l_perp]

    gh : control.TransferFunction = control.tf(new_open_num, new_open_den)

    # Define Closed-Loop System
    new_num = [K_1 + F_supp/l_perp]
    new_den = [M_m, B_m, F_supp/l_perp + K_1]

    sys : control.TransferFunction = control.tf(new_num, new_den)

    return gh, sys


if __name__ == "__main__":
    new_gh, new_sys = vel_cntr(0.0001, SystemParams(), 0.6, 30)

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

