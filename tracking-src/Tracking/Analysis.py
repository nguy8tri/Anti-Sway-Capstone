from Angular import simulate
import math

if __name__ == "__main__":

    vel_max = [0.25, 0.5]
    vel_op = [True, True]

    max_rpm = 0
    max_torque = 0
    max_voltage = 0
    max_current = 0
    max_x_dev = 0
    max_v_dev = 0
    max_theta = 0

    for i in range(4):
        t, \
            (B, K_i, K_o), \
            (x_ref, v_ref, theta_ref), \
            (v_real, x_real, theta_real), \
            (F_real, T_real, rpm_real, A_real, V_real) = \
                simulate(v_lim=vel_max[i % 2], vel_option=vel_op[i // 2])

        trial_max_rpm = abs(max(rpm_real))
        trial_max_torque = abs(max(T_real))
        trial_max_voltage = abs(max(V_real))
        trial_max_current = abs(max(A_real))
        trial_max_x_dev = abs(max(x_real - x_ref))
        trial_max_v_dev = abs(max(v_real - v_ref))
        trial_max_theta = abs(max(theta_real))

        print(trial_max_torque, trial_max_voltage)


        if max_rpm < trial_max_rpm:
            max_rpm = trial_max_rpm
        if max_torque < trial_max_torque:
            max_torque = trial_max_torque
        if max_voltage < trial_max_voltage:
            max_voltage = trial_max_voltage
        if max_current < trial_max_current:
            max_current = trial_max_current
        if max_x_dev < trial_max_x_dev:
            max_x_dev = trial_max_x_dev
        if max_v_dev < trial_max_v_dev:
            max_v_dev = trial_max_x_dev
        if max_theta < trial_max_theta:
            max_theta = trial_max_theta

    print(f"The maximum motor velocity is: {max_rpm:.3f} rpm")
    print(f"The maximum torque is: {max_torque:.3f} m-N")
    print(f"The maximum voltage is: {max_voltage:.3f} V")
    print(f"The maximum current is: {max_current:.3f} A")
    print(f"The maximum positional deviation is: {max_x_dev:.3f} m")
    print(f"The maximum speed deviation is: {max_v_dev:.3f} m/s")
    print(f"The maximum angle is: {math.degrees(max_theta):.3f} deg")