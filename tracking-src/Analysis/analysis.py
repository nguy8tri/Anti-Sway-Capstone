
import numpy as np
import scipy.io
import matplotlib.pyplot as plt

if __name__ == "__main__":
    data = scipy.io.loadmat("../tracking2.mat")
    A_x = np.array(data['angle_x'])
    V_x = np.array(data['angle_y'])
    A_x = np.reshape(A_x, np.size(A_x))
    V_x = np.reshape(V_x, np.size(V_x))
    VV_x = np.reshape(np.array(data['trolley_x']), np.size(A_x))
    VV_y = np.reshape(np.array(data['angle_x']), np.size(A_x))

    print(data.keys())
    print(VV_x)

    plt.figure()
    # plt.subplot(2, 1, 1)
    plt.plot(A_x, label="Angle")
    # plt.subplot(2, 1, 2)
    # plt.plot(np.rad2deg(V_x), label="Voltage")
    plt.plot(VV_x, label="Trolley Position")
    # plt.plot(np.rad2deg(VV_y), label="Voltage-y")
    plt.legend()
    plt.show()
