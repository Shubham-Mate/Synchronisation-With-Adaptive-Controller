import numpy as np
from .config import load_config
from .chen_system import chen_master_slave_system_with_adaptive_control
from .rk4_solver import RK4_Solver
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib

matplotlib.use("TkAgg")
np.set_printoptions(threshold=np.inf)

solver_params = load_config("./src/config.yaml")["solver_params"]

initial_value = np.array([0.5, 1, 1.5, 0.75, 0.5, 2, 40, 40, 40])

solver = RK4_Solver(
    initial_value=initial_value, h=solver_params["h"], t=solver_params["t"]
)

time_arr = solver.t
solution = solver.solve(chen_master_slave_system_with_adaptive_control)

plt.figure(figsize=(10, 5))

plt.plot(time_arr, solution[0], label="Master X")
plt.plot(time_arr, solution[3], label="Slave X")

plt.grid(True)

plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
#
# ax.plot(xs=solution[0], ys=solution[1], zs=solution[2], linewidth=1.0)
# plt.show()
