#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import matplotlib.pyplot as plt

from robot_2d import Robot2D
from rrt import RRT
import numpy as np

robot = Robot2D(link_lengths=np.array([0.75, 0.5, 0.5, 0.5]))


# 1) create animation function
# robot.animate([[0, 0]])
# robot.animate([[0, 0], [1, 1]])

# robot.animate(robot.interpolate([0, 0], [1, 1]), fps=3)

def sample(qg):
    if np.random.choice([True, False]):
        return qg
    return np.random.uniform(-np.pi, np.pi, size=np.asarray(qg).size)


def check_path(q0, q1):
    path = robot.interpolate(q0, q1, step_size=0.02)
    for q in path:
        if robot.in_collision(q):
            return False
    return True


rrt = RRT(collision_fn=robot.in_collision, sample_fn=sample, path_fn=check_path, delta_q=0.4)
path = rrt.plan([0, 0, 0, 0], [1, 1, 1, 1])
robot.animate(path, fps=3)
