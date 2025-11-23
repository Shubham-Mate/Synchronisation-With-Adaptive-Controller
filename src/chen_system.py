import numpy as np
from .config import load_config

rossler_params = load_config("./src/config.yaml")["rossler_params"]
a, b, c = rossler_params["a"], rossler_params["b"], rossler_params["c"]


def chen_master_slave_system_with_adaptive_control(time, state):
    x1, y1, z1 = state[0:3]
    x2, y2, z2 = state[3:6]
    estimated_a, estimated_b, estimated_c = state[6:9]

    dx1 = a * (y1 - x1)
    dy1 = (c - a) * x1 - x1 * z1 + c * y1
    dz1 = x1 * y1 - b * z1

    e_x = x2 - x1
    e_y = y2 - y1
    e_z = z2 - z1

    u_1 = -estimated_a * (e_y - e_x) - e_x
    u_2 = (
        x2 * z2 - x1 * z1 - (estimated_c - estimated_a) * e_x - (estimated_c + 1) * e_y
    )
    u_3 = x1 * y1 - x2 * y2 + (estimated_b - 1) * e_z

    dx2 = a * (y2 - x2) + u_1
    dy2 = (c - a) * x2 - x2 * z2 + c * y2 + u_2
    dz2 = x2 * y2 - b * z2 + u_3

    da = -(e_x**2)
    db = -(e_z**2)
    dc = e_x * e_y + e_y**2

    return np.array([dx1, dy1, dz1, dx2, dy2, dz2, da, db, dc])


def chen_system(time, state):
    x1, y1, z1 = state[0:3]

    dx1 = a * (y1 - x1)
    dy1 = (c - a) * x1 - x1 * z1 + c * y1
    dz1 = x1 * y1 - b * z1

    return np.array([dx1, dy1, dz1])
