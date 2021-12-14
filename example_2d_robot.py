#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import matplotlib.pyplot as plt

from robot_2d import Robot2D
from rrt import RRT
import numpy as np

robot = Robot2D()


# 1) create animation function
# robot.animate([[0, 0]])
# robot.animate([[0, 0], [1, 1]])

# robot.animate(robot.interpolate([0, 0], [1, 1]), fps=3)

def sample(qg):
    return np.random.uniform(-np.pi, np.pi, size=np.asarray(qg).size)


rrt = RRT(collision_fn=lambda q: False, sample_fn=sample)
path = rrt.plan([0, 0], [1, 1])
robot.animate(path)
