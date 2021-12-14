#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import matplotlib.pyplot as plt

from robot_2d import Robot2D

robot = Robot2D()
# 1) create animation function
# robot.animate([[0, 0]])
# robot.animate([[0, 0], [1, 1]])

robot.animate(robot.interpolate([0, 0], [1, 1]))
