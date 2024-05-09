
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import control as ct

def get_entry(data, name):
    entry = np.array(data[name])
    return entry.flatten()

if __name__ == "__main__":
    data = scipy.io.loadmat("../tracking.mat")

    lo = 0

    T = 0.005
    print(data.keys())
    t = get_entry(data, "t")
    trolley_y = get_entry(data, "trolley_y")
    voltage_y = get_entry(data, "voltage_y")
    angle_y = get_entry(data, "angle_y")

    plt.figure()
    plt.title("Position Response in X Direction")
    plt.subplot(4, 1, 1)
    plt.plot(t, angle_y, label="Angle")
    plt.legend()
    plt.subplot(4, 1, 2)
    plt.plot(t, trolley_y, label="Position")
    plt.legend()
    plt.subplot(4, 1, 3)
    plt.plot(voltage_y, label="Voltage")
    plt.legend()


    K_pi = .10
    K_po = -100
    B = 8 * 0.765 / 0.1

    F_2_V = 0.003/(0.41*0.11)

    outer = ct.tf([K_pi * K_po], [B, K_pi])
    inner = ct.tf([F_2_V * B, F_2_V * K_pi], [0, 1])

    outer_d = ct.c2d(outer, 0.005, 'tustin')
    inner_d = F_2_V * (K_pi + B * (2 / T) * ((1 - ct.tf('z')) / (1 + ct.tf('z'))))

    factor = K_pi * T + 2 * B

    print(outer_d, (K_pi * T - 2 * B) / factor, K_pi * K_po * T / factor)

    outer_res = ct.forced_response(outer_d, t, -angle_y)
    inner_res = ct.forced_response(inner_d, t, outer_res.y[0] - trolley_y)

    plt.subplot(4, 1, 4)
    plt.plot(t, inner_res.y[0], label="Theoretical Voltage")
    plt.legend()
    plt.show()

    plt.figure()
    plt.title("Voltage Error")
    plt.plot(t, inner_res.y[0] + voltage_y)
    plt.show()

    # print(voltage_x[lo+100:lo+110])
