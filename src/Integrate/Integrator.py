from typing import Callable, Union, List
import numpy as np
from _distutils_hack import override
from collections.abc import Iterable


class DI:

    def __init__(self):
        raise TypeError("Cannot initialize Abstract Class DI")

    def time_sample(self, begin : float = 0.0, end : float = 50):
        t = [begin]
        while t[-1] < end:
            t.append(t[-1] + self.dt)

        return t

    def differentiate(self, f : Union[Callable[[float], Union[int, float]], List[float]],
                            t : List[float]) -> List[float]:
        raise TypeError("Cannot use Abstract Class DI for Differentiating")

    def integrate(self, f : List[float],
                        g : List[List[float]],
                        forcing : List[Union[Callable[[float], Union[int, float]], List[float]]],
                        t : List[float],
                        scheme: bool,
                        i_v : float = 0.0) -> List[float]:
        raise TypeError("Cannot use Abstract Class DI for Integration")

class Euler(DI):

    def __init__(self, dt : float):
        super().__init__()
        self.dt : float = dt

    def __fb_euler__(self, x_n, x_m):
        return (x_n - x_m) / self.dt

    def __central_euler__(self, x_n, x_m):
        return (x_n - x_m) / (2 * self.dt)

    @override
    def differentiate(self, f: Union[Callable[[float], Union[int, float]], List[float]],
                      t: List[float]) -> List[float]:
        if callable(f):
            f = f(t)
        if not isinstance(f, Iterable):
            raise TypeError("f should be a function or array of floats")
        if len(t) < 2:
            raise TypeError("The range should be greater than 2")
        if not len(t) == len(f):
            raise TypeError("f and range should have the same dimensions")

        res = []

        res.append(self.__fb_euler__(f[0], f[1]))

        for i in range(1, len(t) - 1):
            res.append(self.__central_euler__(f[i - 1], f[i + 1]))

        res.append(self.__fb_euler__(f[-2], f[-1]))

        return res

    @override
    def integrate(self, f: List[float],
                  g: List[List[float]],
                  forcing: List[Union[Callable[[float], Union[int, float]], List[float]]],
                  t: List[float],
                  i_v: Union[float, List[float]] = 0.0) -> List[float]:
        if not type(forcing) == list:
            forcing = [forcing]
        if not type(g[0]) == list:
            g = [g]

        if not type(i_v) == list:
            i_v = [[i_v] for i in range(len(f) - 1)]
        elif not len(i_v) == len(f) - 1:
            raise TypeError("The initial condition must have the same number of elements as the order of the differential equation")

        if [1 for g_fun in forcing if callable(g_fun)]:
            forcing_new = []
            for g_fun in forcing:
                if not callable(g_fun):
                    if not type(g_fun) == list:
                        raise TypeError("There is a non-function/list in here");
                    continue
                g_fun_new = []
                for time in t:
                    g_fun_new.append(g_fun(time))
                forcing_new.append(g_fun_new)
            forcing = forcing_new
        for g_fun in forcing:
            if not len(g_fun) == len(t):
                raise TypeError("A forcing function exists that is not the same size as the sample")

        G = [0] * len(t)
        for i in range(len(g[0])):
            for j, g_fun in enumerate(forcing):
                for k, val in enumerate(g_fun):
                    G[k] += g[-1 * (i + 1)] * val
                forcing[j] = self.differentiate(g_fun, t)

        res = i_v
        for row in range(len(res)):


class Tustin(Euler):
    pass
class RK4(Tustin):
    pass