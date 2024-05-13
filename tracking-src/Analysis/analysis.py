
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import control as ct

def get_entry(data, name):
    entry = np.array(data[name])
    return entry.flatten()



if __name__ == "__main__":
    data = scipy.io.loadmat("../perfection.mat")

    lo = 0

    T = 0.005
    print(data.keys())
    id = get_entry(data, "id")
    # r = [i for i, idx in enumerate(id) if idx == 1]
    t = get_entry(data, "t")
    trolley_y = get_entry(data, "trolley_y")#[r]
    voltage_y = get_entry(data, "voltage_y")#[r]
    angle_y = np.rad2deg(get_entry(data, "angle_y"))#[r]
    inner_y = get_entry(data, "inner_y")#[r]

    plt.figure(figsize=(7, 10))
    plt.title("Position Response in X Direction")
    plt.subplot(6, 1, 1)
    plt.plot(t, angle_y, label="Angle")
    plt.legend()
    plt.subplot(6, 1, 2)
    plt.plot(t, trolley_y, label="Position")
    plt.legend()
    plt.subplot(6, 1, 3)
    plt.plot(voltage_y, label="Voltage")
    plt.legend()


    K_pi = 0.001
    K_po = -3000
    B = 8 * 0.765 / 0.1

    F_2_V = 0.003/(0.41*0.11)

    outer = ct.tf([K_pi * K_po], [B, K_pi])
    inner = ct.tf([F_2_V * B, F_2_V * K_pi], [0, 1])

    outer_d = ct.c2d(outer, T, 'tustin')
    inner_d = F_2_V * (K_pi + B * (2 / T) * ((ct.tf('z') - 1) / (1 + ct.tf('z'))))

    factor = K_pi * T + 2 * B

    diff = (2 / T) * (ct.tf('z') - 1) / (1 + ct.tf('z'))

    # print(outer_d, (K_pi * T - 2 * B) / factor, K_pi * K_po * T / factor)


    outer_res = ct.forced_response(outer_d, T=t, U=-angle_y)
    inner_res = ct.forced_response(inner_d, U=outer_res.y[0])
    diff_res = ct.forced_response(diff, U=-B*trolley_y)
    plt.subplot(6, 1, 4)
    plt.plot(t, np.clip(inner_res.y[0]+diff_res.y[0], -10, 10), label="Theoretical Voltage")
    plt.legend()


    plt.subplot(6, 1, 5)
    plt.plot(t, inner_y, label="Outer Output")
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(t, outer_res.y[0], label="Outer Output (Theoretical)")
    plt.legend()

    plt.figure()
    plt.title("Voltage Error")
    plt.plot(t, np.clip(inner_res.y[0], -10, 10) + voltage_y)
    plt.show()

    plt.figure()
    plt.title("Voltage Contributions")
    inner_res_pos = ct.forced_response(inner_d, t, U=-trolley_y)
    plt.plot(t, inner_res_pos.y[0], label="Position")
    plt.plot(t, angle_y * K_pi * K_po * F_2_V, label="Angular Diff")
    plt.legend()
    plt.show()

    # print(voltage_x[lo+100:lo+110])
