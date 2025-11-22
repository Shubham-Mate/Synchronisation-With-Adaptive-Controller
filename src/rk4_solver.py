import numpy as np


class RK4_Solver:
    def __init__(self, initial_value, h, t) -> None:
        self.initial_value = initial_value
        self.h = h
        self.t = np.arange(start=0, stop=t, step=h)
        self.soln = np.zeros(shape=(initial_value.shape[0], self.t.shape[0]))

        for i in range(initial_value.shape[0]):
            self.soln[i][0] = self.initial_value[i]

    def solve(self, de):
        for i in range(1, self.t.shape[0]):
            w_i = np.array(
                [self.soln[k][i - 1] for k in range(self.initial_value.shape[0])]
            )
            k_1 = self.h * de(self.t[i - 1], w_i)
            k_2 = self.h * de(self.t[i - 1] + (self.h / 2), w_i + (k_1 / 2))
            k_3 = self.h * de(self.t[i - 1] + (self.h / 2), w_i + (k_2 / 2))
            k_4 = self.h * de(self.t[i], w_i + k_3)

            w_i_next = w_i + (k_1 + 2 * k_2 + 2 * k_3 + k_4) / 6

            for k in range(self.initial_value.shape[0]):
                self.soln[k][i] = w_i_next[k]

        return self.soln
