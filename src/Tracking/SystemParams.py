
class SystemParams:

    g = 9.81

    def __init__(self):
        pass

    def get_mean_male_params(self, frac_supp : float):
        if frac_supp > 1.0:
            frac_supp = 1.0
        elif frac_supp < 0.0:
            frac_supp = 0.0

        M_m = 10
        l_perp = 2
        M_p = 60
        F_supp = M_p * self.g * frac_supp

        return M_m, l_perp, F_supp