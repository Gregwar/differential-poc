import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz

"""
Example where all the joints are set as passive.
"""

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
solver.set_passive("lower", True, 0, 0.0)
solver.set_passive("upper", True, 0, 0.0)

t = 0.0
dt = 0.001
solver.dt = dt

viz.display(robot.state.q)


@schedule(interval=dt)
def loop():
    global t, lower, upper, data
    t += dt
    viz.display(robot.state.q)

    # Solving
    robot.update_kinematics()
    result = solver.solve(True)

    if not result.success:
        print("Failed to solve()")
        exit()


run_loop()
