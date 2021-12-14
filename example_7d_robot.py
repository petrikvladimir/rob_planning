#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-14
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import matplotlib.pyplot as plt

from robot_pyphysx import RobotPyPhysX
from rrt import RRT
import numpy as np

robot = RobotPyPhysX(robot='panda', change_robot_color=True, start_goal_alpha=0.2)


robot.add_obstacle_1()

def sample(qg):
    if np.random.choice([True, False]):
        return qg
    return robot.sample_q()


def check_path(q0, q1):
    path = robot.interpolate(q0, q1, step_size=0.02)
    for q in path:
        if robot.in_collision(q):
            return False
    return True


rrt = RRT(collision_fn=robot.in_collision, sample_fn=sample, path_fn=check_path, delta_q=2.8)
start = np.asarray(robot.default_q())
start[0] -= np.pi / 2
goal = np.asarray(robot.default_q())
goal[0] += np.pi / 2
goal = robot.sample_q()
while robot.in_collision(goal):
    goal = robot.sample_q()
path = rrt.plan(start, goal)
# interpolate_path = []
# for q0, q1 in zip(path[:-1], path[1:]):
#     interpolate_path += robot.interpolate(q0, q1)
robot.animate(path, fps=10)
