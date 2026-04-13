import numpy as np
import math


def score(val: np.ndarray):  # 1st is modulus, second is argument
    dx = 10.0 - val[0] * math.cos(val[1])
    dy = 20.0 - val[0] * math.sin(val[1])
    return dx * dx + dy * dy


def grad_desc():
    target = np.array([10.0, 20.0])
    curr = np.array([20, 1.5])
    print(curr)

    mod_shift = 0.01
    arg_shift = 0.01
    learn_rate = 1

    err = 1000
    for _ in range(10000):
        # if score(curr) > 2 * err:
        #     break
        err = score(curr)
        mod = curr + np.array([mod_shift, 0.0])
        mod_err = score(mod)
        arg = curr + np.array([0.0, arg_shift])
        arg_err = score(arg)
        
        dmod = (mod_err - err) / (mod[0] - curr[0])
        if dmod:
            curr -= np.array([mod_shift, 0.0]) * (err / dmod) * learn_rate
        darg = (arg_err - err) / (arg[1] - curr[1])
        if darg:
            curr -= np.array([0.0, arg_shift]) * (err / darg) * learn_rate

    print(curr, score(curr))


if __name__ == "__main__":
    grad_desc()
    print(f"truth: {np.array([math.hypot(20, 10), math.atan2(20, 10)])}")
