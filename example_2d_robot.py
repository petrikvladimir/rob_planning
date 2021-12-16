#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from robot_2d import Robot2D
from rrt import RRT
import numpy as np

robot = Robot2D(link_lengths=np.array([0.75, 0.5, 0.25, 0.25]))


# 1) create animation function
# robot.animate([[0, 0], [1,1]])
# robot.animate([[0, 0], [0.5, 0.5], [1, 1]])

# robot.animate(robot.interpolate([0, 0], [1, 1]), fps=3)

def sample(qg):
    if np.random.choice([True, False]):
        return qg
    return np.random.uniform(-np.pi, np.pi, size=np.asarray(qg).size)


def path_check(q0, q1):
    path = robot.interpolate(q0, q1, step_size=0.02)
    for q in path:
        if robot.in_collision(q):
            return False
    return True


rrt = RRT(collision_fn=robot.in_collision, sample_fn=sample, delta_q=0.1)
# robot.animate(rrt.plan([0, 0], [1, 1]))  # warning: too big delta_q will 'skip' the obstacle
robot.animate(
    rrt.plan_with_path_check(np.zeros_like(robot.link_lengths), np.ones_like(robot.link_lengths), path_fn=path_check)
)
