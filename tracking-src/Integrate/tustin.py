
import matplotlib.pyplot as plt
import numpy as np
import control as ct

class TustinIntegrator():

    def __init__(self, gain, timestep):
        self.gain = gain * timestep / 2.0
        self.prev_input = 0.0
        self.prev_output = 0.0

    def step(self, input_):
        result = self.prev_output + self.gain * (input_ + self.prev_input)
        self.prev_input = input_
        self.prev_output = result
        return result

class TustinDifferentiator():

    def __init__(self, gain, timestep):
        self.gain = gain * 2.0 / timestep
        self.prev_input = 0.0
        self.prev_output = 0.0

    def step(self, input_):
        result = -self.prev_output + \
        self.gain * (input_ - self.prev_input)

        self.prev_input = input_
        self.prev_output = result

if __name__ == "__main__":
    obj = TustinIntegrator(1.0, 0.005)
    obj2 = TustinDifferentiator(1.0, 0.05)

    f = lambda x: x ** 3
    f_int = lambda x: x ** 4 / 4
    f_dev = lambda x: 3 * x ** 2

    t = np.arange(0, 10, 0.005)

    real_int = f(t)
    approx_int = []
    approx_diff = []
    for val in real_int:
        approx_int.append(obj.step(val))
        approx_diff.append(obj2.step(val))

    integral = ct.c2d(ct.tf([1], [1, 0]), 0.005)
    print(integral)
    res = ct.forced_response(integral, t, real_int)

    plt.figure()
    plt.plot(t, f_int(t), label="Theoretical")
    plt.plot(t, approx_int, label="Experimental")
    plt.plot(t, res.y[0], label="Real Experimental")
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(t, f_dev(t), label="Theoretical")
    plt.plot(t, approx_diff, label="Experimental")
    plt.legend()
    plt.show()