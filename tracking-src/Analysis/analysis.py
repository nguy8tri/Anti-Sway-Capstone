
from typing import Tuple

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import control as ct

M_u = 0.765
M_y = 0.664
M_x = 2.092
F_2_V = 0.0062 / (0.41 * 0.11)

def get_entry(data, name):
    entry = np.array(data[name])
    return entry.flatten() if np.size(entry) != 1 else entry.flatten()[0]

def get_theoretical_model(M, M_u, K, B, l_perp) -> Tuple[ct.TransferFunction, ct.TransferFunction]:
    g = 9.81
    F_supp = M_u * g # 100% weight support
    ang_closed_num = [M, B, 0]
    pos_closed_num = [F_supp - K]
    closed_den = [l_perp * M, l_perp * B, F_supp - K]

    return (ct.tf(ang_closed_num, closed_den),
            ct.tf(pos_closed_num, closed_den))

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

    ang_model_x, pos_model_x = get_theoretical_model(M_x, M_u, K_x, B_x, l_perp)
    ang_model_y, pos_model_y = get_theoretical_model(M_y, M_u, K_y, B_y, l_perp)

    t = get_entry(data, "t")
    meas_ang_x = get_entry(data, "angle_x")
    meas_ang_y = get_entry(data, "angle_y")
    meas_pos_x = get_entry(data, "trolley_pos_x")
    meas_pos_y = get_entry(data, "trolley_pos_y")

    user_pos_x = l_perp * np.sin(meas_ang_x) + meas_pos_x
    user_pos_y = l_perp * np.sin(meas_ang_y) + meas_pos_y

    assert np.size(t) == np.size(user_pos_x)
    assert np.size(t) == np.size(user_pos_y)

    ang_model_res_x = ct.forced_response(ang_model_x, t, user_pos_x)
    pos_model_res_x = ct.forced_response(pos_model_x, t, user_pos_x)

    ang_model_res_y = ct.forced_response(ang_model_y, t, user_pos_y)
    pos_model_res_y = ct.forced_response(pos_model_y, t, user_pos_y)

    initial_filter = 25

    theo_ang_x = np.rad2deg(ang_model_res_x.y[0])[initial_filter:]
    theo_ang_y = np.rad2deg(ang_model_res_y.y[0])[initial_filter:]

    theo_pos_x = pos_model_res_x.y[0][initial_filter:]
    theo_pos_y = pos_model_res_y.y[0][initial_filter:]

    t = t[initial_filter:]

    meas_ang_x = np.rad2deg(meas_ang_x)[initial_filter:]
    meas_ang_y = np.rad2deg(meas_ang_y)[initial_filter:]
    meas_pos_x = meas_pos_x[initial_filter:]
    meas_pos_y = meas_pos_y[initial_filter:]

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

    plt.show()

    plt.figure(figsize=(8, 5))
    plt.suptitle("Errors (Theoretical v. Actual)")

    plt.subplot(4, 1, 1)
    plt.ylabel("X Angular \n(deg)")
    plt.plot(t, theo_ang_x - meas_ang_x)

    plt.subplot(4, 1, 2)
    plt.ylabel("Y Angular \n(deg)")
    plt.plot(t, theo_ang_y - meas_ang_y)

    plt.subplot(4, 1, 3)
    plt.ylabel("X Position \n(m)")
    plt.plot(t, theo_pos_x - meas_pos_x)

    plt.subplot(4, 1, 4)
    plt.ylabel("Y Position \n(m)")
    plt.plot(t, theo_pos_y - meas_pos_y)

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
    data = scipy.io.loadmat("../anti-sway.mat")

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
    data = scipy.io.loadmat("../regimen-3.mat")

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
    loss_x = get_entry(data, "loss_x")[:hi]
    loss_y = get_entry(data, "loss_y")[:hi]

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

    total_losses_x = total_losses_x / total_pts_x
    total_losses_y = total_losses_y / total_pts_y

    plt.figure()
    plt.title("Loss per Epoch")
    plt.plot(total_losses_x, label="X Loss")
    plt.plot(total_losses_y, label="Y Loss")
    plt.show()

    plt.figure()
    plt.suptitle("Velocity Step Responses")
    plt.subplot(2, 1, 1)
    plt.ylabel("X Velocity (m/s)")
    plt.plot(vel_ref_x, label="Reference")
    plt.plot(vel_ref_x_, label="Mod")
    plt.plot(trolley_vel_x, label="Response")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.plot(angle_y, label="Angle")
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


if __name__ == "__main__":
    tracking_experiment_1()
