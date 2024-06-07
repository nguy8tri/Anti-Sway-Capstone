
from typing import Tuple

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import control as ct

M_u = 0.765
M_y = 0.664
M_x = 2.092
F_2_V = 0.0062 / (0.41 * 0.11)

# for the file regimen-ex1
statistics = '''Gradients: (dKp_x: -1.666e-02), (dKi_x: 0.000e+00), (dKp_y: -8.935e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.157e+01), (Ki_x: 2.089e+01), (Kp_y: 4.227e+01), (Ki_y: 2.023e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -1.612e-02), (dKp_y: 0.000e+00), (dKi_y: -8.739e-03)
New gains: (Kp_x: 4.157e+01), (Ki_x: 2.230e+01), (Kp_y: 4.227e+01), (Ki_y: 2.176e+01)
Exiting Thread
Gradients: (dKp_x: -1.119e-02), (dKi_x: 0.000e+00), (dKp_y: -7.369e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.255e+01), (Ki_x: 2.230e+01), (Kp_y: 4.356e+01), (Ki_y: 2.176e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -1.350e-02), (dKp_y: 0.000e+00), (dKi_y: -6.471e-03)
New gains: (Kp_x: 4.255e+01), (Ki_x: 2.348e+01), (Kp_y: 4.356e+01), (Ki_y: 2.290e+01)
Exiting Thread
Gradients: (dKp_x: -1.084e-02), (dKi_x: 0.000e+00), (dKp_y: -6.942e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.349e+01), (Ki_x: 2.348e+01), (Kp_y: 4.477e+01), (Ki_y: 2.290e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -5.181e-03), (dKp_y: 0.000e+00), (dKi_y: -6.513e-03)
New gains: (Kp_x: 4.349e+01), (Ki_x: 2.394e+01), (Kp_y: 4.477e+01), (Ki_y: 2.404e+01)
Exiting Thread
Gradients: (dKp_x: -1.126e-02), (dKi_x: 0.000e+00), (dKp_y: -6.258e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.448e+01), (Ki_x: 2.394e+01), (Kp_y: 4.587e+01), (Ki_y: 2.404e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -1.145e-02), (dKp_y: 0.000e+00), (dKi_y: -5.398e-03)
New gains: (Kp_x: 4.448e+01), (Ki_x: 2.494e+01), (Kp_y: 4.587e+01), (Ki_y: 2.498e+01)
Exiting Thread
Gradients: (dKp_x: -1.171e-02), (dKi_x: 0.000e+00), (dKp_y: -5.780e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.550e+01), (Ki_x: 2.494e+01), (Kp_y: 4.688e+01), (Ki_y: 2.498e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -1.066e-02), (dKp_y: 0.000e+00), (dKi_y: -6.052e-03)
New gains: (Kp_x: 4.550e+01), (Ki_x: 2.587e+01), (Kp_y: 4.688e+01), (Ki_y: 2.604e+01)
Exiting Thread
Gradients: (dKp_x: -1.013e-02), (dKi_x: 0.000e+00), (dKp_y: -5.415e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.639e+01), (Ki_x: 2.587e+01), (Kp_y: 4.783e+01), (Ki_y: 2.604e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -9.464e-03), (dKp_y: 0.000e+00), (dKi_y: -4.622e-03)
New gains: (Kp_x: 4.639e+01), (Ki_x: 2.670e+01), (Kp_y: 4.783e+01), (Ki_y: 2.685e+01)
Exiting Thread
Gradients: (dKp_x: -9.190e-03), (dKi_x: 0.000e+00), (dKp_y: -5.488e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.720e+01), (Ki_x: 2.670e+01), (Kp_y: 4.879e+01), (Ki_y: 2.685e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -9.146e-03), (dKp_y: 0.000e+00), (dKi_y: -5.074e-03)
New gains: (Kp_x: 4.720e+01), (Ki_x: 2.750e+01), (Kp_y: 4.879e+01), (Ki_y: 2.773e+01)
Exiting Thread
Gradients: (dKp_x: -7.955e-03), (dKi_x: 0.000e+00), (dKp_y: -5.040e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.789e+01), (Ki_x: 2.750e+01), (Kp_y: 4.967e+01), (Ki_y: 2.773e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -8.755e-03), (dKp_y: 0.000e+00), (dKi_y: -4.570e-03)
New gains: (Kp_x: 4.789e+01), (Ki_x: 2.827e+01), (Kp_y: 4.967e+01), (Ki_y: 2.853e+01)
Exiting Thread
Gradients: (dKp_x: -7.454e-03), (dKi_x: 0.000e+00), (dKp_y: -5.107e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.854e+01), (Ki_x: 2.827e+01), (Kp_y: 5.056e+01), (Ki_y: 2.853e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -8.926e-03), (dKp_y: 0.000e+00), (dKi_y: -4.164e-03)
New gains: (Kp_x: 4.854e+01), (Ki_x: 2.905e+01), (Kp_y: 5.056e+01), (Ki_y: 2.926e+01)
Exiting Thread
Gradients: (dKp_x: -7.429e-03), (dKi_x: 0.000e+00), (dKp_y: -1.202e-02), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.919e+01), (Ki_x: 2.905e+01), (Kp_y: 5.266e+01), (Ki_y: 2.926e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -8.935e-03), (dKp_y: 0.000e+00), (dKi_y: -4.065e-03)
New gains: (Kp_x: 4.919e+01), (Ki_x: 2.983e+01), (Kp_y: 5.266e+01), (Ki_y: 2.997e+01)
Exiting Thread
Gradients: (dKp_x: -6.371e-03), (dKi_x: 0.000e+00), (dKp_y: -5.228e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 4.975e+01), (Ki_x: 2.983e+01), (Kp_y: 5.358e+01), (Ki_y: 2.997e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -7.594e-03), (dKp_y: 0.000e+00), (dKi_y: -4.162e-03)
New gains: (Kp_x: 4.975e+01), (Ki_x: 3.049e+01), (Kp_y: 5.358e+01), (Ki_y: 3.070e+01)
Exiting Thread
Gradients: (dKp_x: -6.961e-03), (dKi_x: 0.000e+00), (dKp_y: -4.795e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 5.036e+01), (Ki_x: 3.049e+01), (Kp_y: 5.442e+01), (Ki_y: 3.070e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -7.691e-03), (dKp_y: 0.000e+00), (dKi_y: -3.338e-03)
New gains: (Kp_x: 5.036e+01), (Ki_x: 3.117e+01), (Kp_y: 5.442e+01), (Ki_y: 3.129e+01)
Exiting Thread
Gradients: (dKp_x: -5.743e-03), (dKi_x: 0.000e+00), (dKp_y: -8.553e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 5.086e+01), (Ki_x: 3.117e+01), (Kp_y: 5.591e+01), (Ki_y: 3.129e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -8.090e-03), (dKp_y: 0.000e+00), (dKi_y: -3.372e-03)
New gains: (Kp_x: 5.086e+01), (Ki_x: 3.187e+01), (Kp_y: 5.591e+01), (Ki_y: 3.188e+01)
Exiting Thread
Gradients: (dKp_x: -7.081e-03), (dKi_x: 0.000e+00), (dKp_y: -6.660e-03), (dKi_y: 0.000e+00)
New gains: (Kp_x: 5.148e+01), (Ki_x: 3.187e+01), (Kp_y: 5.708e+01), (Ki_y: 3.188e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -7.370e-03), (dKp_y: 0.000e+00), (dKi_y: -2.907e-03)
New gains: (Kp_x: 5.148e+01), (Ki_x: 3.252e+01), (Kp_y: 5.708e+01), (Ki_y: 3.238e+01)
Exiting Thread
Gradients: (dKp_x: -6.191e-03), (dKi_x: 0.000e+00), (dKp_y: -2.386e-02), (dKi_y: 0.000e+00)
New gains: (Kp_x: 5.202e+01), (Ki_x: 3.252e+01), (Kp_y: 6.125e+01), (Ki_y: 3.238e+01)
Exiting Thread
Gradients: (dKp_x: 0.000e+00), (dKi_x: -6.851e-03), (dKp_y: 0.000e+00), (dKi_y: -2.569e-03)
New gains: (Kp_x: 5.202e+01), (Ki_x: 3.312e+01), (Kp_y: 6.125e+01), (Ki_y: 3.283e+01)'''

def get_entry(data, name):
    entry = np.array(data[name])
    return entry.flatten() if np.size(entry) != 1 else entry.flatten()[0]

def find(data, point):
    L = -1
    H = len(data)

    while L + 1 != H:
        mid = (L + H) // 2
        if data[mid] <= point:
            L = mid
        else:
            H = mid
    return L

def get_theoretical_model(M, M_u, K, B, l_perp) -> Tuple[ct.TransferFunction, ct.TransferFunction]:
    g = 9.81
    F_supp = M_u * g # 100% weight support
    ang_closed_num = [M, B, 0]
    pos_closed_num = [F_supp - K]
    vel_closed_num = [F_supp- K, 0]
    closed_den = [l_perp * M, l_perp * B, F_supp - K]

    theta, pos = (ct.tf(ang_closed_num, closed_den),
                  ct.tf(pos_closed_num, closed_den))
    f = -B * ct.tf(vel_closed_num, closed_den) - K * theta
    return theta, pos, f


def diagnostic():
    data = scipy.io.loadmat("../tracking-exp1.mat")

    lo = 0

    T = 0.005
    print(data.keys())
    id = get_entry(data, "id")
    r = np.argwhere(id == 1).flatten()
    t = get_entry(data, "t")[r]
    trolley_y = get_entry(data, "trolley_pos_y")[r]
    trolley_y_prime = get_entry(data, "trolley_vel_y")[r]
    voltage_y = get_entry(data, "voltage_y")[r]
    angle_y = get_entry(data, "angle_y")[r]
    inner_y = get_entry(data, "inner_y")[r]

    assert np.size(t) == np.size(inner_y)

    plt.figure(figsize=(7, 10))
    plt.title("Position Response in Y Direction")
    plt.subplot(6, 1, 1)
    plt.plot(t, np.rad2deg(angle_y), label="Angle")
    plt.legend()
    plt.subplot(6, 1, 2)
    plt.plot(t, trolley_y, label="Position")
    plt.legend()
    plt.subplot(6, 1, 3)
    plt.plot(t, voltage_y, label="Voltage")
    plt.legend()

    K_pi = 0.001
    K_po = -3000
    B = 8 * 0.765 / 0.1

    outer = ct.tf([K_pi * K_po], [B, K_pi])
    inner = ct.tf([F_2_V * B, F_2_V * K_pi], [0, 1])

    outer_d = ct.c2d(outer, T, 'tustin')
    inner_d = F_2_V * (K_pi + B * (2 / T) * ((ct.tf('z') - 1) / (1 + ct.tf('z'))))

    factor = K_pi * T + 2 * B

    diff = (2 / T) * (ct.tf('z') - 1) / (1 + ct.tf('z'))

    # print(outer_d, (K_pi * T - 2 * B) / factor, K_pi * K_po * T / factor)

    outer_res = ct.forced_response(outer_d, U=-angle_y)
    inner_res = ct.forced_response(inner_d, U=outer_res.y[0])
    diff_res = ct.forced_response(diff, U=-B * trolley_y)

    V_theo = F_2_V * (angle_y * 1040.0 - 53.2 * trolley_y)

    plt.subplot(6, 1, 4)
    plt.plot(t, np.clip(V_theo, -10, 10), label="Theoretical Voltage")
    plt.legend()

    plt.subplot(6, 1, 5)
    plt.plot(t, inner_y, label="Outer Output")
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(t, 1040.0 * angle_y, label="Outer Output (Theoretical)")
    plt.legend()

    plt.figure()
    plt.title("Voltage Error")
    plt.plot(t, np.clip(inner_res.y[0], -10, 10) / voltage_y)
    plt.show()

    plt.figure()
    plt.title("Voltage Contributions")
    inner_res_pos = ct.forced_response(inner_d, t, U=-trolley_y)
    plt.plot(t, inner_res_pos.y[0], label="Position")
    plt.plot(t, angle_y * K_pi * K_po * F_2_V, label="Angular Diff")
    plt.legend()
    plt.show()

def tracking_experiment_1():
    data = scipy.io.loadmat("../tracking-exp1.mat")

    K_x = get_entry(data, "K_x")
    K_y = get_entry(data, "K_y")
    B_x = get_entry(data, "B_x")
    B_y = get_entry(data, "B_y")

    # Theoretical model
    l_perp = 0.47
    M_u = 0.765
    M_y = 0.664
    M_x = 2.092

    ang_model_x, pos_model_x, f_model_x = get_theoretical_model(M_x, M_u, K_x, B_x, l_perp)
    ang_model_y, pos_model_y, f_model_y = get_theoretical_model(M_y, M_u, K_y, B_y, l_perp)

    t = get_entry(data, "t")
    meas_ang_x = get_entry(data, "angle_x")
    meas_ang_y = get_entry(data, "angle_y")
    meas_pos_x = get_entry(data, "trolley_pos_x")
    meas_pos_y = get_entry(data, "trolley_pos_y")
    meas_f_x = get_entry(data, "voltage_x") / F_2_V
    meas_f_y = get_entry(data, "voltage_y") / F_2_V
    meas_vel_x = get_entry(data, "trolley_vel_x")
    meas_vel_y = get_entry(data, "trolley_vel_y")

    user_pos_x = l_perp * np.sin(meas_ang_x) + meas_pos_x
    user_pos_y = l_perp * np.sin(meas_ang_y) + meas_pos_y

    assert np.size(t) == np.size(user_pos_x)
    assert np.size(t) == np.size(user_pos_y)

    ang_model_res_x = ct.forced_response(ang_model_x, t, user_pos_x)
    pos_model_res_x = ct.forced_response(pos_model_x, t, user_pos_x)
    f_model_res_x = ct.forced_response(f_model_x, t, user_pos_x)

    ang_model_res_y = ct.forced_response(ang_model_y, t, user_pos_y)
    pos_model_res_y = ct.forced_response(pos_model_y, t, user_pos_y)
    f_model_res_y = ct.forced_response(f_model_y, t, user_pos_y)

    initial_filter = 25

    theo_ang_x = np.rad2deg(ang_model_res_x.y[0])[initial_filter:]
    theo_ang_y = np.rad2deg(ang_model_res_y.y[0])[initial_filter:]

    theo_pos_x = pos_model_res_x.y[0][initial_filter:]
    theo_pos_y = pos_model_res_y.y[0][initial_filter:]

    theo_f_x = f_model_res_x.y[0][initial_filter:]
    theo_f_y = f_model_res_y.y[0][initial_filter:]

    t = t[initial_filter:]

    meas_ang_x = np.rad2deg(meas_ang_x)[initial_filter:]
    meas_ang_y = np.rad2deg(meas_ang_y)[initial_filter:]
    meas_pos_x = meas_pos_x[initial_filter:]
    meas_pos_y = meas_pos_y[initial_filter:]
    meas_f_x = meas_f_x[initial_filter:]
    meas_f_y = meas_f_y[initial_filter:]
    meas_vel_x = meas_vel_x[initial_filter:]
    meas_vel_y = meas_vel_y[initial_filter:]

    user_pos_x = user_pos_x[initial_filter:]
    user_pos_y = user_pos_y[initial_filter:]

    plt.figure(figsize=(7, 5))
    plt.suptitle("Theoretical v. Actual Angle")

    plt.subplot(2, 1, 1)

    plt.ylabel("X Angle (deg)")
    plt.plot(t, theo_ang_x, label="Theoretical")
    plt.plot(t, meas_ang_x, label="Actual")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.xlabel("Time (s)")
    plt.ylabel("Y Angle (deg)")
    plt.plot(t, theo_ang_y, label="Theoretical")
    plt.plot(t, meas_ang_y, label="Actual")
    plt.legend()
    plt.savefig("tracking-exp2-ang")
    plt.show()

    plt.figure(figsize=(7, 5))
    plt.suptitle("Theoretical v. Actual Position (Sampled for Comparison)")

    sample = [i for i in range(0, len(t), 40)]

    plt.subplot(2, 1, 1)

    plt.ylabel("X Position (m)")
    plt.plot(t, user_pos_x, label="User")
    plt.plot(t[sample], theo_pos_x[sample], '.', label="Theoretical")
    plt.plot(t[sample], meas_pos_x[sample], '.', label="Actual")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.xlabel("Time (s)")
    plt.ylabel("Y Position (m)")
    plt.plot(t, user_pos_y, label="Ref")
    plt.plot(t[sample], theo_pos_y[sample], '.', label="Theoretical")
    plt.plot(t[sample], meas_pos_y[sample], '.', label="Actual")
    plt.legend()
    plt.savefig("tracking-exp2-pos")
    plt.show()

    plt.figure(figsize=(8, 5))
    plt.suptitle("Errors (Theoretical v. Actual)")

    plt.subplot(4, 1, 1)
    plt.ylabel("X Position \n(m)")
    plt.plot(t, theo_pos_x - meas_pos_x)

    plt.subplot(4, 1, 2)
    plt.ylabel("Y Position \n(m)")
    plt.plot(t, theo_pos_y - meas_pos_y)

    plt.subplot(4, 1, 3)
    plt.ylabel("X Angular \n(deg)")
    plt.plot(t, theo_ang_x - meas_ang_x)

    plt.subplot(4, 1, 4)
    plt.ylabel("Y Angular \n(deg)")
    plt.plot(t, theo_ang_y - meas_ang_y)
    plt.savefig("tracking-exp2-err")
    plt.show()


    plt.plot()
    plt.suptitle("Theoretical v. Actual Force")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Force (N)")
    plt.plot(t, theo_f_x, label="Theoretical")
    plt.plot(t, meas_f_x, label="Actual")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Force (N)")
    plt.xlabel("Time (s)")
    plt.plot(t, theo_f_y, label="Theoretical")
    plt.plot(t, meas_f_y, label="Actual")
    plt.legend()
    plt.savefig("tracking-exp2-force")
    plt.show()

def pos_vel_comparison():
    data = scipy.io.loadmat("../tracking-exp1.mat")

    t = get_entry(data, "t")
    trolley_pos_x = get_entry(data, "trolley_pos_x")
    trolley_vel_x = get_entry(data, "trolley_vel_x")
    trolley_pos_y = get_entry(data, "trolley_pos_y")
    trolley_vel_y = get_entry(data, "trolley_vel_y")

    # Integrator
    diff = ct.tf([1, 0], [0.000000000001, 1])

    diff_pos_x = ct.forced_response(diff, t, trolley_pos_x, trolley_vel_x[0])
    diff_pos_y = ct.forced_response(diff, t, trolley_pos_y, trolley_vel_y[0])

    filter = 1

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.ylabel("X Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(t, trolley_vel_x, label="Measured")
    plt.plot(t[filter:], diff_pos_x.y[0][filter:], label="Diff")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(t, trolley_vel_y, label="Measured")
    plt.plot(t[filter:], diff_pos_y.y[0][filter:], label="Diff")
    plt.legend()
    plt.show()

    plt.figure()
    plt.suptitle("Error")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(t[filter:], trolley_vel_x[filter:] - diff_pos_x.y[0][filter:])
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(t[filter:], trolley_vel_y[filter:] - diff_pos_y.y[0][filter:])
    plt.legend()
    plt.show()

def anti_sway_diagnostic():
    data = scipy.io.loadmat("../full-regimen-2.mat")

    id = get_entry(data, "id")
    r = np.argwhere(id == 1).flatten()
    print(get_entry(data, "K_x"))
    print(get_entry(data, "K_y"))
    # print(get_entry(data, "Kp_x'")[r][-1])
    print(2 * np.sqrt(0.47 * 9.81), 2 * np.sqrt(0.47 / 9.81) * 9.81, 0.47 * 9.81)
    print(get_entry(data, "MKi_x"), 27.7 * (2.092 + 0.765))

    t = get_entry(data, "t")
    vel_err_x = get_entry(data, "vel_err_x")
    vel_err_y = get_entry(data, "vel_err_y")
    int_out_x = get_entry(data, "int_out_x")
    int_out_y = get_entry(data, "int_out_y")

    Ki_x = get_entry(data, "MKi_x")
    Ki_y = get_entry(data, "MKi_y")

    integrator_x = ct.tf([0.0000000001, F_2_V * Ki_x], [1, 0])
    integrator_y = ct.tf([0.0000000001, F_2_V * Ki_y], [1, 0])

    # theo_int_out_x = ct.forced_response(integrator_x, t, vel_err_x)
    # theo_int_out_y = ct.forced_response(integrator_y, t, vel_err_y)

    filter = 1

    sample = [i for i in range(0, len(t), 50)]

    plt.figure()
    plt.suptitle("Integral Outputs")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Integrator")
    plt.plot(t[sample], int_out_x[sample], '.', label="Algo")
    # plt.plot(t[filter:], theo_int_out_x.y[0][filter:], label="Theo")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Integrator")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.plot(t[sample], int_out_y[sample], '.', label="Algo")
    # plt.plot(t[filter:], theo_int_out_y.y[0][filter:], label="Theo")
    plt.legend()
    plt.show()

    ## Part 2, test the step response

    K_x = get_entry(data, "K_x")
    K_y = get_entry(data, "K_y")
    angle_x = get_entry(data, "angle_x")
    angle_y = get_entry(data, "angle_y")

    vel_ref_x = get_entry(data, "vel_ref_x")
    vel_ref_y = get_entry(data, "vel_ref_y")
    vel_ref_x_ = get_entry(data, "vel_ref_x") + K_x * angle_x
    vel_ref_y_ = get_entry(data, "vel_ref_y") + K_y * angle_y
    trolley_vel_x = get_entry(data, "trolley_vel_x")
    trolley_vel_y = get_entry(data, "trolley_vel_y")

    plt.figure()
    plt.suptitle("Velocity Step Responses")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Velocity (m/s)")
    # plt.plot(t, angle_x, label="Angle")
    plt.plot(t, vel_ref_x, label="Reference")
    plt.plot(t, vel_ref_x_, label="Mod")
    plt.plot(t, trolley_vel_x, label="Response")
    # plt.plot(t, vel_ref_x - vel_ref_x_, label="Error")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(t, angle_y, label="Angle")
    plt.plot(t, vel_ref_y, label="Reference")
    plt.plot(t, vel_ref_y_, label="Mod")
    plt.plot(t, trolley_vel_y, label="Response")
    # plt.plot(t, vel_ref_y - vel_ref_y_, label="Error")
    plt.legend()
    plt.show()

    print(max(vel_ref_x), max(vel_ref_y), max(trolley_vel_x), max(trolley_vel_y))

def auto_tune_diagnostic():
    data = scipy.io.loadmat("../Training/anti-sway.mat")

    hi = len(get_entry(data, "t"))
    id = get_entry(data, "id")
    t = get_entry(data, "t")[:hi]
    K_x = get_entry(data, "K_x")
    K_y = get_entry(data, "K_y")
    angle_x = get_entry(data, "angle_x")[:hi]
    angle_y = get_entry(data, "angle_y")[:hi]
    vel_ref_x = get_entry(data, "vel_ref_x")[:hi]
    vel_ref_y = get_entry(data, "vel_ref_y")[:hi]
    vel_ref_x_ = vel_ref_x + K_x * angle_x
    vel_ref_y_ = vel_ref_y + K_y * angle_y
    trolley_vel_x = get_entry(data, "trolley_vel_x")[:hi]
    trolley_vel_y = get_entry(data, "trolley_vel_y")[:hi]
    loss_x = (vel_ref_x_ - trolley_vel_x) ** 2
    loss_y = (vel_ref_y_ - trolley_vel_y) ** 2

    total_losses_x = np.zeros(int(id[-1]))
    total_losses_y = np.zeros(int(id[-1]))
    total_pts_x = np.zeros(int(id[-1]))
    total_pts_y = np.zeros(int(id[-1]))


    for i, (l_x, l_y) in zip(id, zip(loss_x, loss_y)):
        i = int(i) - 1
        total_losses_x[i] += l_x
        total_losses_y[i] += l_y
        total_pts_x[i] += 1
        total_pts_y[i] += 1


    # total_losses_x = total_losses_x / total_pts_x
    # total_losses_y = total_losses_y / total_pts_y

    plt.figure()
    plt.suptitle("Losses per Epoch")
    # plt.subplot(2, 1, 1)
    plt.ylabel("Loss ([m/s]^2)")
    plt.plot([i for i in range(1, len(total_losses_x) + 1)], total_losses_x, label="X Loss")
    plt.plot(33, total_losses_x[32], '.', label="Chosen")
    plt.xlabel("Epochs (s)")
    plt.legend()
    # plt.subplot(2, 1, 2)
    # plt.ylabel("Loss ([m/s]^2)")
    # plt.xlabel("Epochs")
    # plt.plot(total_losses_y, label="Y Loss")
    plt.savefig("pi-auto-tune-loss")
    plt.show()

    r = np.argwhere((id == 1) | \
                    (id == 5) | \
                    (id == 10) | \
                    (id == 13) | \
                    (id == 17) | \
                    (id == 20) | \
                    (id == 25) | \
                    (id == 32))

    ids = [1, 8, 15, 24, 33]
    plt.figure(figsize=(14, 6))
    plt.suptitle("Training Regimen: Velocity Step Responses (Selected Epochs)")
    for i, j in enumerate(ids):
        ax = plt.subplot(1, len(ids), i + 1)
        if i == 0:
            plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time (s)")
        if i != 0:
            ax.get_yaxis().set_visible(False)
        ax.set_ylim(0, 0.20)
        q = np.argwhere(id == j)
        if j == 33:
            plt.title(f"Epoch {j} (Chosen)")
        else:
            plt.title(f"Epoch {j}")
        plt.plot(t[:550], vel_ref_x[q], '--', label="Reference")
        plt.plot(t[:550], vel_ref_x_[q], label="Anti-Sway\nReference")
        plt.plot(t[:550], trolley_vel_x[q], label="Response")
        if i == 0:
            plt.legend(loc="lower right")
    plt.savefig("pi-auto-tune-training-progression")
    plt.show()

    print(get_entry(data, "Kp_x'")[np.argwhere(id == 33)][0] / (M_x + M_u))
    print(get_entry(data, "Ki_x'")[np.argwhere(id == 33)][0] / (M_x + M_u))
    plt.figure()
    plt.suptitle("Velocity Step Responses")
    # plt.subplot(2, 1, 1)
    plt.ylabel("X Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(vel_ref_x, label="Reference")
    plt.plot(vel_ref_x_, label="Reference")
    plt.plot(trolley_vel_x, label="Response")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Velocity (m/s)")
    plt.xlabel("Time (s)")
    # plt.plot(angle_y, label="Angle")
    plt.plot(vel_ref_y, label="Reference")
    plt.plot(vel_ref_y_, label="Mod")
    plt.plot(trolley_vel_y, label="Response")
    plt.legend()

    plt.show()

    Kp_y = get_entry(data, "Kp_y'")[:hi]
    Kp_x = get_entry(data, "Kp_x'")[:hi]
    Ki_y = get_entry(data, "Ki_y'")[:hi]
    Ki_x = get_entry(data, "Ki_x'")[:hi]

    plt.figure()
    plt.title("Gains per Epoch")
    plt.plot(Kp_x / (M_x + M_u), label="Kp_x")
    plt.plot(Kp_y / (M_y + M_u), label="Kp_y")
    plt.plot(Ki_x / (M_x + M_u), label="Ki_x")
    plt.plot(Ki_y / (M_y + M_u), label="Ki_y")
    plt.legend()
    plt.show()

    print(id[3000])
    print(Kp_x[3000] / (M_x + M_u))
    print(Ki_x[3000] / (M_x + M_u))

    training_data = scipy.io.loadmat("../Training/anti-sway-tuning.mat")



    dKp_x = get_entry(training_data, "dKp_x")
    dKi_x = get_entry(training_data, "dKi_x")
    dKp_y = get_entry(training_data, "dKp_y")
    dKi_y = get_entry(training_data, "dKi_y")
    Kp_x = get_entry(training_data, "Kp_x")
    Ki_x = get_entry(training_data, "Ki_x")
    Kp_y = get_entry(training_data, "Kp_y")
    Ki_y = get_entry(training_data, "Ki_y")

    # Normalization
    count_x = get_entry(training_data, "count_x")
    count_y = get_entry(training_data, "count_y")

    # Correction to Erroneous normalization (remove if no longer the case)
    dKi_x = dKi_x * count_y
    dKi_y = dKi_y / count_y

    odd = [i for i in range(0, len(dKp_x), 2)]
    even = [i for i in range(1, len(dKp_x), 2)]

    plt.figure()
    plt.title("Gradients v. Epochs (Normalized)")
    plt.ylabel("Gradient")
    plt.xlabel("Epochs")
    plt.plot(dKp_x[odd], label="Kp_x")
    plt.plot(dKi_x[even], label="Ki_x")
    plt.plot(dKp_y[odd], label="Kp_y")
    plt.plot(dKi_y[even], label="Ki_y")
    plt.legend()
    plt.show()

    dKp_x = dKp_x * 550
    dKi_x = dKi_x * 550
    dKp_y = dKp_y * count_y
    dKi_y = dKi_y * count_y

    plt.figure()
    plt.title("Gradients v. Epochs (Normalized)")
    plt.ylabel("Gradient (Negative)")
    plt.xlabel("Epochs")
    plt.plot(-dKp_x[odd], label="Kₚ")
    plt.plot(-dKi_x[even], label="Kᵢ")
    # plt.plot(dKp_y[odd], label="Kp_y")
    # plt.plot(dKi_y[even], label="Ki_y")
    plt.legend()
    plt.savefig("pi-auto-tune-gradients")
    plt.show()

    plt.figure()
    plt.title("Parameters v. Epochs")
    plt.ylabel("Parameters")
    plt.xlabel("Epochs")
    plt.plot(Kp_x[odd], label="Kₚ")
    plt.plot(Ki_x[even], label="Kᵢ")
    plt.axvline(15, color='red', label="Chosen Parameters")
    plt.plot(15, Kp_x[31], 'x', color='red')
    plt.plot(15, Ki_x[31], 'x', color='red')
    # plt.plot(Kp_y[odd], label="Kp_y")
    # plt.plot(Ki_y[even], label="Ki_y")
    plt.legend()
    plt.savefig("pi-auto-tune-params")
    plt.show()

    print(Kp_x[31], Ki_x[31], Kp_y[25], Ki_y[25])

def auto_tune_gradient():
    data = scipy.io.loadmat("../full-regimen-2.mat")
    params = scipy.io.loadmat("../anti-sway-tuning-2.mat")

    id = get_entry(data, "id")
    i_prev = 23
    i = 24
    r_prev = np.argwhere(id == i_prev)
    r = np.argwhere(id == i)
    int_prev = get_entry(data, "int_out_y")[r_prev]
    int_n = get_entry(data, "int_out_y")[r]
    trolley_vel_y = get_entry(data, "trolley_vel_y")[r]
    loss_y = get_entry(data, "loss_y")[r]
    Kp_y = get_entry(params, "Ki_x")[i - 1]
    print(data.key())
    plt.figure()
    plt.plot()
    plt.plot(trolley_vel_y)
    plt.show()
def get_antisway_model():
    pass

def anti_sway_experiment():
    data_x = scipy.io.loadmat("../regimen-6X.mat")
    data_y = scipy.io.loadmat("../regimen-5.mat")

    id_x = get_entry(data_x, "id")
    r_x = np.argwhere(id_x == 10)
    t_x = get_entry(data_x, "t")[r_x]
    t_x = t_x - t_x[0]
    ref_x = get_entry(data_x, "vel_ref_x")[r_x]
    act_x = get_entry(data_x, "trolley_vel_x")[r_x]
    Kp_x = get_entry(data_x, "Kp_x'")[r_x][0]
    Ki_x = get_entry(data_x, "Ki_x'")[r_x][0]
    K_x = get_entry(data_x, "K_x")

    id_y = get_entry(data_y, "id")
    r_y = np.argwhere(id_y == 11)
    t_y = get_entry(data_y, "t")[r_y]
    t_y = t_y - t_y[0]
    ref_y = get_entry(data_y, "vel_ref_y")[r_y]
    act_y = get_entry(data_y, "trolley_vel_y")[r_y]
    Kp_y = get_entry(data_y, "Kp_y'")[r_y][0]
    Ki_y = get_entry(data_y, "Ki_y'")[r_y][0]
    K_y = get_entry(data_y, "K_y")

    plt.figure(figsize=(7, 5))
    plt.suptitle("Velocity Step Responses")

    plt.subplot(2, 1, 1)
    plt.ylabel("X Velocity (m/s)")
    plt.plot(t_x, ref_x, label="Reference")
    plt.plot(t_x, act_x, label="Actual")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.ylabel("Y Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(t_y, ref_y, label="Reference")
    plt.plot(t_y, act_y, label="Actual")
    plt.legend()

    plt.show()

def tracking_experiment_slow():
    # Obtain data
    data = scipy.io.loadmat("../tracking-slow2.mat")

    # Extract X and Y Data
    id = get_entry(data, "id")
    t = get_entry(data, "t")
    x_data = np.argwhere(id == 1)
    y_data = np.argwhere(id == id[-1])

    print(data.keys())
    t_x = t[x_data]
    t_y = t[y_data]

    s_x = get_entry(data, "trolley_pos_x")[x_data]
    s_y = get_entry(data, "trolley_pos_y")[y_data]
    v_x = get_entry(data, "trolley_vel_x")[x_data]
    v_y = get_entry(data, "trolley_vel_y")[y_data]
    ang_x = get_entry(data, "angle_x")[x_data]
    ang_y = get_entry(data, "angle_y")[y_data]
    u_x = s_x + 0.47 * np.sin(ang_x)
    u_y = s_y + 0.47 * np.sin(ang_y)
    f_x = get_entry(data, "voltage_x") / F_2_V
    f_y = get_entry(data, "voltage_y") / F_2_V


    plt.figure()
    plt.title("Diagnostic (Confirm that velocity is constant")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.plot(t_x, v_x, label="X")
    plt.plot(t_y, v_y, label="Y")
    plt.axvline(x=2.85)
    plt.axvline(x=4.245)
    plt.axvline(x=4.8)
    plt.axvline(x=6.79)
    plt.axvline(x=7.45)
    plt.axvline(x=8)
    plt.legend()
    plt.show()

    t_1x = find(t_x, 2.85)
    t_2x = find(t_x, 4.245)
    t_3x = find(t_x, 4.8)

    v_1x = v_x[t_1x]
    v_2x = v_x[t_2x]
    v_3x = v_x[t_2x:t_3x]
    v_3x = np.sum(v_3x) / len(v_3x)

    t_1y = find(t_y, 6.69)
    t_2y = find(t_y, 7.45)
    t_3y = find(t_y, 8)

    v_1y = v_y[t_1y]
    v_2y = v_y[t_2y]
    v_3y = v_y[t_2y:t_3y]
    v_3y = np.sum(v_3y) / len(v_3y)

    # Generate Steps and Actual Velocities

    t_ref_x = (t_x[t_1x:t_3x] - t_x[t_1x]).flatten()
    v_ref_x = np.append(np.linspace(v_1x, v_2x, t_2x - t_1x),
              np.array([v_3x] * (t_3x - t_2x)))
    print(np.shape(t_ref_x), np.shape(v_ref_x))
    s_ref_x = scipy.integrate.cumulative_trapezoid(v_ref_x, t_ref_x, initial=0)
    print(np.shape(s_ref_x))
    v_act_x = v_x[t_1x:t_3x]
    s_act_x = s_x[t_1x:t_3x]

    t_ref_y = (t_y[t_1y:t_3y] - t_y[t_1y]).flatten()
    v_ref_y = np.append(np.linspace(v_1y, v_2y, t_2y - t_1y),
              np.array([v_3y] * (t_3y - t_2y)))
    s_ref_y = scipy.integrate.cumulative_trapezoid(v_ref_y, t_ref_y, initial=0)
    v_act_y = v_y[t_1y:t_3y]
    s_act_y = s_x[t_1y:t_3y]

    # Get Theoretical Model
    K_x = get_entry(data, "K_x")
    K_y = get_entry(data, "K_y")
    B_x = get_entry(data, "B_x")
    B_y = get_entry(data, "B_y")

    # Theoretical model
    l_perp = 0.47
    M_u = 0.765
    M_y = 0.664
    M_x = 2.092

    ang_model_x, pos_model_x, f_model_x = get_theoretical_model(M_x, M_u, K_x, B_x, l_perp)
    ang_model_y, pos_model_y, f_model_y = get_theoretical_model(M_y, M_u, K_y, B_y, l_perp)

    ang_res_x, pos_res_x = (ct.forced_response(ang_model_x, t_ref_x, s_ref_x),
                            ct.forced_response(pos_model_x, t_ref_x, s_ref_x))
    ang_res_y, pos_res_y = (ct.forced_response(ang_model_y, t_ref_y, s_ref_y),
                            ct.forced_response(pos_model_y, t_ref_y, s_ref_y))
    f_res_x, f_res_y = (ct.forced_response(f_model_x, t_ref_x, s_ref_x),
                        ct.forced_response(f_model_y, t_ref_y, s_ref_y))

    plt.figure()
    plt.suptitle("Theoretical v. Actual Position")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Position (m)")
    plt.plot(t_ref_x, u_x[t_1x:t_3x], label="Reference")
    plt.plot(t_ref_x, pos_res_x.y[0] + s_x[t_1x], label="Theoretical")
    plt.plot(t_ref_x, s_x[t_1x:t_3x], label="Actual")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Position (m)")
    plt.xlabel("Time(s)")
    plt.plot(t_ref_y, u_y[t_1y:t_3y], label="Reference")
    plt.plot(t_ref_y, pos_res_y.y[0] + s_y[t_1y], label="Theoretical")
    plt.plot(t_ref_y, s_y[t_1y:t_3y], label="Actual")
    plt.legend()
    plt.savefig("tracking-exp1-pos")
    plt.show()

    plt.figure()
    plt.suptitle("Theoretical v. Actual Angle")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Angle (deg)")
    plt.plot(t_ref_x, np.rad2deg(ang_res_x.y[0] + ang_x[t_1x]), label="Theoretical")
    plt.plot(t_ref_x, np.rad2deg(ang_x[t_1x:t_3x]), label="Actual")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Angle (deg)")
    plt.xlabel("Time(s)")
    plt.plot(t_ref_y, np.rad2deg(ang_res_y.y[0] + ang_y[t_1y]), label="Theoretical")
    plt.plot(t_ref_y, np.rad2deg(ang_y[t_1y:t_3y]), label="Actual")
    plt.legend()
    plt.savefig("tracking-exp1-ang")
    plt.show()

    plt.figure()
    plt.suptitle("Errors (Theoretical v. Actual)")
    plt.subplot(4, 1, 1)
    plt.ylabel("X Position (m)")
    plt.plot(t_ref_x, (pos_res_x.y[0] + s_x[t_1x]).flatten() - s_x[t_1x:t_3x].flatten(), label="A-T")
    # plt.legend()
    plt.subplot(4, 1, 2)
    plt.ylabel("Y Position (m)")
    plt.xlabel("Time(s)")
    plt.plot(t_ref_y, (pos_res_y.y[0] + s_y[t_1y]).flatten() - s_y[t_1y:t_3y].flatten(), label="A-T")
    # plt.legend()
    plt.subplot(4, 1, 3)
    plt.ylabel("X Angle (deg)")
    plt.plot(t_ref_x, np.rad2deg(ang_res_x.y[0] + ang_x[t_1x]).flatten() - np.rad2deg(ang_x[t_1x:t_3x]).flatten(), label="Actual")
    # plt.legend()
    plt.subplot(4, 1, 4)
    plt.ylabel("Y Angle (deg)")
    plt.xlabel("Time(s)")
    plt.plot(t_ref_y, (np.rad2deg(ang_res_y.y[0] + ang_y[t_1y])).flatten() - np.rad2deg(ang_y[t_1y:t_3y]).flatten(), label="Actual")
    # plt.legend()
    plt.savefig("tracking-exp1-err")
    plt.show()

    f_res_x, f_res_y = (ct.forced_response(f_model_x, t_ref_x, s_ref_x),
                        ct.forced_response(f_model_y, t_ref_y, s_ref_y))

    plt.plot()
    plt.suptitle("Theoretical v. Actual Force")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Force (N)")
    plt.plot(t_ref_x, f_res_x.y[0], label="Theoretical")
    plt.plot(t_ref_x, f_x[t_1x:t_3x], label="Actual")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Force (N)")
    plt.xlabel("Time (s)")
    plt.plot(t_ref_y, f_res_y.y[0], label="Theoretical")
    plt.plot(t_ref_y, f_y[t_1y:t_3y], label="Actual")
    plt.legend()
    plt.savefig("tracking-exp1-force")
    plt.show()


    data = scipy.io.loadmat("../tracking-exp1.mat")

    t_x = get_entry(data, "t")
    t_y = t_x
    s_x = get_entry(data, "trolley_pos_x")
    s_y = get_entry(data, "trolley_pos_y")
    a_x = get_entry(data, "angle_x")
    a_y = get_entry(data, "angle_y")
    u_x = s_x + 0.47 * np.sin(a_x)
    u_y = s_y + 0.47 * np.sin(a_y)

    plt.figure()
    plt.suptitle("Controller Performance (Positional Comparison)")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Position (m)")
    plt.plot(t_x, u_x, label="User Position")
    plt.plot(t_x, s_x, label="Trolley")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Position (m)")
    plt.xlabel("Time(s)")
    plt.plot(t_y, u_y, label="User Position")
    plt.plot(t_y, s_y, label="Trolley")
    plt.legend()
    plt.savefig("tracking-exp2-perf-pos")
    plt.show()

    plt.figure()
    plt.suptitle("Controller Performance (Positional Disparity)")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Positional Error (m)")
    plt.plot(t_x, (s_x - u_x))
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Positional Error (m)")
    plt.xlabel("Time (s)")
    plt.plot(t_y, (s_y - u_y))
    plt.savefig("tracking-exp1-perf-pos-err")
    plt.show()

if __name__ == "__main__":
    tracking_experiment_slow()
