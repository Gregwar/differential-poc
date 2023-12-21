import placo
import numpy as np
import matplotlib.pyplot as plt
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz

"""
Doing some motion and plotting torques (optionally)
"""

# Plot torque ?
plot = False

robot = placo.RobotWrapper("differential/", placo.Flags.ignore_collisions)

viz = robot_viz(robot)

solver = placo.DynamicsSolver(robot)
solver.mask_fbase(True)

# Adding internal "gears" task
gears = solver.add_gear_task()
gears.add_gear("alpha", "upper", 0.5)
gears.add_gear("alpha", "lower", 0.5)
gears.add_gear("beta", "upper", 1)
gears.add_gear("beta", "lower", -1)
gears.configure("gears", "hard")

# Associating internal forces to the gears
internal_forces = solver.add_task_contact(gears)

# Setting alpha and beta (angles of internal structure) as passive
solver.set_passive("alpha", True, 0, 0.0)
solver.set_passive("beta", True, 0, 0.0)

# Creating joints task to control DOFs
joints = solver.add_joints_task()

t = 0.0
dt = 0.001
lower = 0.0
upper = 0.0
solver.dt = dt
data = {"t": [], "tau_lower": [], "tau_upper": []}

viz.display(robot.state.q)


@schedule(interval=dt)
def loop():
    global t, lower, upper, data
    t += dt
    viz.display(robot.state.q)

    # Moving DOFs
    if t % 6 < 1:
        lower += dt
    elif t % 6 < 2:
        upper += dt
    elif t % 6 < 3:
        lower -= dt
        upper -= dt
    elif t % 6 < 4:
        lower -= dt
        upper += dt
    elif t % 6 < 5:
        lower += dt
        upper -= dt

    joints.set_joints({"lower": lower, "upper": upper})

    # Solving
    robot.update_kinematics()
    result = solver.solve(True)

    if result.success:
        tau_lower = result.tau[robot.get_joint_v_offset("lower")]
        tau_upper = result.tau[robot.get_joint_v_offset("upper")]
        data["t"].append(t)
        data["tau_lower"].append(abs(tau_lower))
        data["tau_upper"].append(abs(tau_upper))
    else:
        print("Failed to solve()")
        exit()

    if t > 6 and plot:
        plt.clf()
        plt.plot(data["t"], data["tau_lower"], label="tau_lower")
        plt.plot(data["t"], data["tau_upper"], label="tau_upper")
        plt.title("Torques")
        plt.legend()
        plt.grid()
        plt.pause(0.1)
        data = {"t": [], "tau_lower": [], "tau_upper": []}
        t = 0


run_loop()
