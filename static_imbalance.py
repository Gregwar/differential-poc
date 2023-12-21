import placo
import numpy as np
import matplotlib.pyplot as plt
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz

"""
A static situation (with Y-oriented gravity) where torque is not balanced across motors.
"""

robot = placo.RobotWrapper("differential/", placo.Flags.ignore_collisions)
robot.set_gravity(np.array([0, 10, 0]))

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
solver.dt = dt

viz.display(robot.state.q)


@schedule(interval=dt)
def loop():
    global t, lower, upper, data
    t += dt
    viz.display(robot.state.q)

    # Joints targets are 0
    joints.set_joints({"lower": 0, "upper": 0})

    # Solving
    robot.update_kinematics()
    result = solver.solve(True)

    if result.success:
        tau_lower = result.tau[robot.get_joint_v_offset("lower")]
        tau_upper = result.tau[robot.get_joint_v_offset("upper")]
        print("Tau_lower: %.4f, Tau_upper: %.4f" % (tau_lower, tau_upper))
    else:
        print("Failed to solve()")
        exit()


run_loop()
