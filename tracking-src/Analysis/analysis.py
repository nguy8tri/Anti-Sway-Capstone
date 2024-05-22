
from typing import Tuple

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import control as ct

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

    F_2_V = 0.0062 / (0.41 * 0.11)

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

if __name__ == "__main__":
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
    user_pos_y = l_perp * np.sin(meas_ang_y)+ meas_pos_y

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
