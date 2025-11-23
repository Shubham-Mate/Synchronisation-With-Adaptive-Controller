import numpy as np
from .config import load_config
from .chen_system import chen_master_slave_system_with_adaptive_control, chen_system
from .rk4_solver import RK4_Solver
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib

matplotlib.use("TkAgg")
np.set_printoptions(threshold=np.inf)

solver_params = load_config("./src/config.yaml")["solver_params"]

initial_value = np.array([0.5, 1, 1.5, -1, -1, -1, 40, 40, 40])
master_initial_value = np.array([0.5, 1, 1.5])
slave_initial_value = np.array([-1, -1, -1])

synch_solver = RK4_Solver(
    initial_value=initial_value, h=solver_params["h"], t=solver_params["t"]
)
master_solver = RK4_Solver(
    initial_value=master_initial_value, h=solver_params["h"], t=solver_params["t"]
)
slave_solver = RK4_Solver(
    initial_value=slave_initial_value, h=solver_params["h"], t=solver_params["t"]
)


time_arr = synch_solver.t
solution = synch_solver.solve(chen_master_slave_system_with_adaptive_control)
slave_soln = slave_solver.solve(chen_system)
master_soln = master_solver.solve(chen_system)

plt.figure(figsize=(10, 5))

plt.title("Master vs Slave (Z Component)")
plt.plot(time_arr, master_soln[2], label="Master Z", color="red", linewidth=2)
plt.plot(
    time_arr, slave_soln[2], label="Slave Z", color="green", linestyle="--", linewidth=2
)
plt.xlabel("t")
plt.ylabel("z(t)")
plt.grid(True)
plt.legend()

plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
#
# ax.plot(xs=solution[0], ys=solution[1], zs=solution[2], linewidth=1.0)
# plt.show()
