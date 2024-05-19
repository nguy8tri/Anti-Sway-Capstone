
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import control as ct
import sys
sys.path.append("../Tracking/")
from Angular import optimize

def get_entry(data, name):
    entry = np.array(data[name])
    return entry.flatten()



if __name__ == "__main__":
    data = scipy.io.loadmat("../tracking-2derr2.mat")

    # System Constants
    g = 9.81

    # System Parameters
    M_m = 2.092  # about 9 lbs
    l_perp = 0.47  # 1.5 ft
    frac_supp = 1.0
    M_p = 0.765
    F_supp = M_p * g * frac_supp

    # Computer-World Interface Parameters
    r = 0.005
    K_m = 0.11
    K_a = 0.41
    K = K_m * K_a

    B_m, K_1, K_2 = optimize(0.1, 0.05)

    # Theoretical model
    closed_num_dist = [M_m, B_m, K_1]
    closed_den = [l_perp * M_m, l_perp * B_m, K_1 * l_perp - K_1 * K_2 + F_supp]


    lo = 0

    T = 0.005
    print(data.keys())
    id = get_entry(data, "id")
    # r = [i for i, idx in enumerate(id) if idx == 1]
    r = np.argwhere(id == 2).flatten()
    t = get_entry(data, "t")[r]
    trolley_y = get_entry(data, "trolley_pos_y")[r]
    trolley_y_prime = get_entry(data, "trolley_vel_y")[r]
    voltage_y = get_entry(data, "voltage_y")[r]
    angle_y = get_entry(data, "angle_y")[r]
    inner_y = get_entry(data, "inner_y")[r]

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

    F_2_V = 0.0062/(0.41*0.11)

    outer = ct.tf([K_pi * K_po], [B, K_pi])
    inner = ct.tf([F_2_V * B, F_2_V * K_pi], [0, 1])

    outer_d = ct.c2d(outer, T, 'tustin')
    inner_d = F_2_V * (K_pi + B * (2 / T) * ((ct.tf('z') - 1) / (1 + ct.tf('z'))))

    factor = K_pi * T + 2 * B

    diff = (2 / T) * (ct.tf('z') - 1) / (1 + ct.tf('z'))

    # print(outer_d, (K_pi * T - 2 * B) / factor, K_pi * K_po * T / factor)


    outer_res = ct.forced_response(outer_d, U=-angle_y)
    inner_res = ct.forced_response(inner_d, U=outer_res.y[0])
    diff_res = ct.forced_response(diff, U=-B*trolley_y)

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

    # print(voltage_x[lo+100:lo+110])
