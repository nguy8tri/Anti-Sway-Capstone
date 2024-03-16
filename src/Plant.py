
import control
import numpy as np
import matplotlib.pyplot as plt

from typing import Union

g = 9.81


def gen_state_model(M_m : float, B_m : float, M_p : float, B_p : float, l_perp : float, F_supp : float) -> control.StateSpace:
    '''

    Produces a StateSpace model of the Anti-Sway System 3 inputs,
    which are the Friction Force, the Motor Force, and the Applied
    Force of the Person

    :param M_m: The mass of the motor
    :param B_m: The damping coefficient of the motor
    :param F_supp: The support force provided to the person
    :param mu: The friction coefficient between the motor and the rail
    :param M_p: The mass of the person
    :param B_p: The damping coefficient of the person
    :param l_perp: The vertical length of the harness
    :return: The state space model pertaining to this system, with the state
    and output vectors being identical (Velocity of Motor, Velocity of Person,
    and Angle of Rope)


    Note:

    - l_perp: Assumed to be constant
    - F_supp: Assumed to be constant
    '''

    A = np.array([[-B_m/M_m, 0, F_supp/M_m], [0, -B_p/M_p, -F_supp/M_p], [-1/l_perp, 1/l_perp, 0]])
    B = np.array([[-1/M_m, 1/M_m, 0], [0, 0, 1/M_p], [0, 0, 0]])

    C = np.identity(3)
    D = np.zeros((3, 3))

    return control.ss(A, B, C, D)

def gen_friction_force(F_supp : Union[float, np.ndarray], M_m : float, mu : float, intervals : int = 50) -> np.ndarray:
    '''
    Generates the friction force for the plant

    :param F_supp: The support force provided to the person
    :param M_m: The mass of the motor
    :param mu: The friction coefficient between the motor and the rail
    :param intervals: The number of sample points to generate
    :return: The friction force, which is constant iff F_supp is, over the sample points

    :raise TypeError: Iff type(F_supp) == np.ndarray, and len(F_supp) != intervals
    '''

    if type(F_supp) == np.ndarray:
        if intervals != len(F_supp):
            raise TypeError("Argument 0 should have the length intervals")
    else:
        F_supp = np.ones(intervals) * F_supp

    return np.array([(f_supp + M_m * g) * mu for f_supp in F_supp])

intervals = 50000

M_m = 10
B_m = 0.5

M_p = (62 + 57) // 2
B_p = 50

l_perp = 1.5

frac_supp = 1.0
F_supp = M_p * g * frac_supp

mu = 0.1
F_f = gen_friction_force(F_supp, M_m, mu, intervals)

t = np.linspace(0, 5, intervals)
F_m = 0 * np.ones(intervals)
F_a = 0 * np.ones(intervals)

model : control.StateSpace = gen_state_model(M_m, B_m, M_p, B_p, l_perp, F_supp)

response : control.TimeResponseData = control.forced_response(model, T=t, U=[F_f, F_m, F_a], X0=[0, 5, 0])

plt.figure()
plt.plot(response.t, response.y.transpose())
plt.plot(response.t, response.u.transpose())
plt.legend(["Motor Velocity", "Person Velcoity", "Angle", "Friction Force", "Motor Force", "Applied Force"])
plt.show()