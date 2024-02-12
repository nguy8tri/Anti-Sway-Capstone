import numpy as np
import matplotlib.pyplot as plt

def simple_pendulum(length, mass, gravity, dt, total_time):
    # Initial conditions
    theta = 0.1  # Initial angle (radians)
    omega = 0.0  # Initial angular velocity

    # Lists to store results
    time_values = np.arange(0, total_time, dt)
    theta_values = []
    omega_values = []

    # Euler's method for numerical integration
    for t in time_values:
        alpha = -gravity / length * np.sin(theta)
        omega += alpha * dt
        theta += omega * dt

        theta_values.append(theta)
        omega_values.append(omega)

    return time_values, theta_values, omega_values

def plot_pendulum_motion(time, theta, omega):
    # Convert theta to x and y positions
    length = 1.0  # You can change the length of the string here
    x = length * np.sin(theta)
    y = -length * np.cos(theta)

    # Plot the motion
    plt.figure(figsize=(8, 6))
    plt.plot(time, x, label='x-position')
    plt.plot(time, y, label='y-position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Pendulum Motion')
    plt.legend()
    plt.grid(True)
    plt.show()

# Parameters
string_length = 1.0
pendulum_mass = 0.1
gravity_constant = 9.81
time_step = 0.01
total_simulation_time = 14

# Run simulation
time, theta, omega = simple_pendulum(string_length, pendulum_mass, gravity_constant, time_step, total_simulation_time)

# Plot results
plot_pendulum_motion(time, theta, omega)
