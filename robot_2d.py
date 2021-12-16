#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon, Point, LineString


class Robot2D:
    def __init__(self, link_lengths=None) -> None:
        self.link_lengths = np.array([0.75, 0.5]) if link_lengths is None else link_lengths
        self.obstacles = MultiPolygon([
            Point((1.0, 1.0)).buffer(0.3, cap_style=3)
        ])

    @staticmethod
    def transformation2d(theta=0, x=0, y=0):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1],
        ])

    def fk(self, q):
        """ Compute forward kinematics of the robot. Return list of frames from base to the gripper. """
        frames = [self.transformation2d()]
        for l, qi in zip(self.link_lengths, q):
            frames.append(frames[-1].dot(self.transformation2d(theta=qi)).dot(self.transformation2d(x=l)))
        return frames

    def fk_points(self, q):
        return [(f[0, 2], f[1, 2]) for f in self.fk(q)]

    @staticmethod
    def plt_points_to_line(p1, p2):
        return (p1[0], p2[0]), (p1[1], p2[1])

    def plot_robot(self, q, ax, color):
        points = self.fk_points(q)
        joints = [ax.plot(*p, 'o', ms=5, color=color)[0] for p in points]
        links = [
            ax.plot(*self.plt_points_to_line(p1, p2), '-', color=color)[0] for p1, p2 in zip(points[:-1], points[1:])
        ]
        return joints, links

    def animate(self, path, fps=1):
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(1, 2, squeeze=True,
                                       figsize=(6.4 * 2, 4.8))  # type: plt.Figure, (plt.Axes, plt.Axes)
        joints, links = self.plot_robot(path[0], ax1, 'black')
        self.plot_robot(path[0], ax1, 'tab:green')
        self.plot_robot(path[-1], ax1, 'tab:red')
        d = sum(self.link_lengths) * 1.2
        ax1.set_xlim(-d, d)
        ax1.set_ylim(-d, d)
        ax1.axis(False)

        for p in list(self.obstacles):
            ax1.fill(*p.exterior.xy, color='tab:grey')

        config, = ax2.plot(*path[0][:2], 'o', ms=5, color='black', label='Current configuration')
        d = np.pi
        ax2.set_xlim(-d, d)
        ax2.set_ylim(-d, d)
        ax2.set_xlabel('q_1 [rad]')
        ax2.set_ylabel('q_2 [rad]')
        ax2.plot(*np.array(path)[:, :2].T, '-', color='tab:grey', label='Planned path')
        ax2.legend()

        plt.show()
        for q in path:
            if self.in_collision(q):
                for v in links + joints + [config]:
                    v.set_color('tab:red')
            points = self.fk_points(q)
            for j, p in zip(joints, points):
                j.set_data(*p)
            for l, (p1, p2) in zip(links, zip(points[:-1], points[1:])):
                l.set_data(*self.plt_points_to_line(p1, p2))
            config.set_data(*q[:2])
            fig.canvas.draw()
            plt.pause(1 / fps)

        plt.ioff()
        plt.show()

    """ Animation function creation - end """

    @staticmethod
    def interpolate(a, b, step_size=0.1):
        a, b = np.asarray(a), np.asarray(b)
        max_d = np.linalg.norm(b - a)
        normalized_d = (b - a) / max_d
        return [a + v * normalized_d for v in np.arange(0., max_d, step_size)] + [b]

    def in_collision(self, q):
        if not hasattr(self, 'obstacles'):
            return False
        ls = LineString(self.fk_points(q))
        return ls.intersects(self.obstacles)
