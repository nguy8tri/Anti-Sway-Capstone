import control
import numpy as np
import matplotlib.pyplot as plt
from SystemParams import SystemParams
from Positional import vel_cntr


# System Constants
g = 9.81

# System Parameters
M_m = 10
l_perp = 2
B_m = 50
frac_supp = 0.5
M_p = 60
F_supp = M_p * g * frac_supp

K_1 = 0.0001
K_2 = l_perp

closed_num_reg = [-K_1*K_2]
closed_num_dist = [M_m, B_m, K_1]
closed_den = [l_perp * M_m, l_perp * B_m, K_1 * l_perp - K_1*K_2 + F_supp]

sys_reg : control.TransferFunction = control.tf(closed_num_reg, closed_den)
sys_dist : control.TransferFunction = control.tf(closed_num_dist, closed_den)

T = 50
samples = 100

t = np.linspace(0, T, samples)
X0 = 0.0

x_f = 5

theta_ref = np.zeros(samples)
x_ref = [i * (2 * x_f) / samples for i in range(int(samples / 2))]
x_ref.extend([x_f for i in range(int(samples/2))])

data_reg : control.TimeResponseData = control.forced_response(sys_reg, t, theta_ref, X0)
data_dist : control.TimeResponseData = control.forced_response(sys_dist, t, x_ref, X0)

plt.figure()
plt.title("Regulator Response")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.plot(data_reg.t, np.rad2deg(data_reg.u[0]), label="Regulator")
plt.plot(data_reg.t, np.rad2deg(data_reg.y[0]), label="Regulator Response")
plt.legend()

plt.figure()
plt.title("Disturbance Response")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.plot(data_dist.t, data_dist.u[0], label="Disturbance (m)")
plt.plot(data_dist.t, np.rad2deg(data_dist.y[0]), label="Disturbance Response")
plt.legend()

plt.figure()
plt.title("Combined Response")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.plot(data_reg.t, np.rad2deg(data_reg.y[0] + data_dist.y[0]), label="Combined Response")
plt.legend()
plt.show()

print(np.rad2deg(data_dist.y[0, -1]))
