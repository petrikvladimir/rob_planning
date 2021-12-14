#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-11
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from pyphysx import *
from pyphysx_render.meshcat_render import MeshcatViewer
from pyphysx_render.utils import gl_color_from_matplotlib
from pyphysx_utils.urdf_robot_parser import URDFRobot
import numpy as np


class RobotPyPhysX:

    def __init__(self, robot='panda', start_goal_alpha=1., change_robot_color=True) -> None:
        super().__init__()
        self.robot_name = robot
        self.obstacles = []
        self.ignore_collisions_between_links = []

        """ Create PyPhysX scene """
        self.scene = Scene()
        self.load_plane_obstacle()
        self.robot: URDFRobot = self.load_robot()
        if change_robot_color:
            self.change_robot_color(
                self.robot, gl_color_from_matplotlib('tab:blue', alpha=1., return_rgba=True) / 255
            )
        self.start_robot: URDFRobot = self.load_robot()
        self.change_robot_color(
            self.start_robot, gl_color_from_matplotlib('tab:green', alpha=start_goal_alpha, return_rgba=True) / 255
        )
        self.goal_robot: URDFRobot = self.load_robot()
        self.change_robot_color(
            self.goal_robot, gl_color_from_matplotlib('tab:red', alpha=start_goal_alpha, return_rgba=True) / 255
        )

    @staticmethod
    def change_robot_color(robot, color):
        assert robot is not None
        for v in robot.links.values():
            for s in v.actor.get_atached_shapes():
                URDFRobot.shape_update_user_data(s, 'color', color)

    def load_plane_obstacle(self):
        self.obstacles.append(RigidStatic.create_plane(material=Material()))
        self.scene.add_actor(self.obstacles[-1])

    def default_q(self):
        if self.robot_name == 'panda':
            return [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
        elif self.robot_name == 'talos':
            q = np.zeros(26)
            q[5] = np.pi / 2
            q[9] = np.pi / 2
            q[12] = -np.pi / 2
            q[16] = -np.pi / 2
            return q
        else:
            assert 'Robot not supported'

    def q_to_dict(self, q, robot=None):
        if robot is None:
            robot = self.robot
        return dict(zip(robot.movable_joints.keys(), q))

    def load_robot(self):
        if self.robot_name == 'panda':
            robot = URDFRobot("franka_panda/panda.urdf", kinematic=True, use_random_collision_colors=True)
            for v in robot.links.values():
                for s in v.actor.get_atached_shapes():
                    URDFRobot.shape_update_user_data(s, 'visual_mesh', None)
            robot.attach_root_node_to_pose((0, 0, 0.05))
            robot.reset_pose(self.q_to_dict(self.default_q(), robot=robot))
            self.scene.add_aggregate(robot.get_aggregate())
            lnames = list(robot.links.keys())
            for l1, l2 in zip(lnames[:-1], lnames[1:]):
                self.ignore_collisions_between_links.append((l1, l2))
            self.ignore_collisions_between_links.append(('panda_link7', 'panda_hand'))
            return robot
        elif self.robot_name == 'talos':
            robot = URDFRobot("talos_desription/talos_reduced.urdf", kinematic=True, mesh_path='talos_desription/')
            robot.attach_root_node_to_pose((0, 0, 1.1))
            robot.reset_pose(self.q_to_dict(self.default_q(), robot=robot))
            self.scene.add_aggregate(robot.get_aggregate())
            lnames = list(robot.links.keys())
            for l1, l2 in zip(lnames[:-1], lnames[1:]):
                self.ignore_collisions_between_links.append((l1, l2))
            self.ignore_collisions_between_links.append(('torso_2_link', 'base_link'))
            self.ignore_collisions_between_links.append(('torso_2_link', 'imu_link'))
            self.ignore_collisions_between_links.append(('torso_2_link', 'head_1_link'))
            self.ignore_collisions_between_links.append(('torso_2_link', 'arm_left_1_link'))
            self.ignore_collisions_between_links.append(('torso_2_link', 'arm_right_1_link'))
            self.ignore_collisions_between_links.append(('arm_left_5_link', 'arm_left_7_link'))
            self.ignore_collisions_between_links.append(('arm_left_7_link', 'gripper_left_base_link'))
            self.ignore_collisions_between_links.append(('arm_right_5_link', 'arm_right_7_link'))
            self.ignore_collisions_between_links.append(('arm_right_7_link', 'gripper_right_base_link'))
            self.ignore_collisions_between_links.append(("base_link", "leg_left_1_link"))
            self.ignore_collisions_between_links.append(("base_link", "leg_right_1_link"))
            self.ignore_collisions_between_links.append(("gripper_left_base_link", "gripper_left_motor_single_link"))
            self.ignore_collisions_between_links.append(("gripper_left_base_link", "gripper_left_inner_single_link"))
            self.ignore_collisions_between_links.append(("gripper_left_motor_double_link", "gripper_left_motor_single_link"))
            self.ignore_collisions_between_links.append(("gripper_left_motor_single_link", "gripper_left_fingertip_3_link"))
            self.ignore_collisions_between_links.append(("gripper_right_base_link", "gripper_right_motor_single_link"))
            self.ignore_collisions_between_links.append(("gripper_right_base_link", "gripper_right_inner_single_link"))
            self.ignore_collisions_between_links.append(("gripper_right_motor_double_link", "gripper_right_motor_single_link"))
            self.ignore_collisions_between_links.append(("gripper_right_motor_single_link", "gripper_right_fingertip_3_link"))
            self.ignore_collisions_between_links.append(("leg_left_1_link", "leg_left_3_link"))
            self.ignore_collisions_between_links.append(("leg_left_4_link", "leg_left_6_link"))
            self.ignore_collisions_between_links.append(("leg_right_1_link", "leg_right_3_link"))
            self.ignore_collisions_between_links.append(("leg_right_4_link", "leg_right_6_link"))

            return robot
        else:
            assert 'Robot not supported'

    def in_collision(self, q, verbose=False, return_links=False):
        self.robot.reset_pose(self.q_to_dict(q))

        """ Self collisions """
        for n1, l1 in self.robot.links.items():
            for n2, l2 in self.robot.links.items():
                if n1 == n2:
                    continue
                if (n1, n2) not in self.ignore_collisions_between_links and \
                        (n2, n1) not in self.ignore_collisions_between_links:
                    if l1.actor.overlaps(l2.actor):
                        if verbose:
                            print(f'Collision: {n1}, {n2}')
                        if return_links:
                            return n1, n2
                        return True

        """ Robot - objects collisions """
        for l in self.robot.links.values():
            for o in self.obstacles:
                if l.actor.overlaps(o):
                    if verbose:
                        print(f'Robot object collision')
                    return True
        return False

    def animate(self, list_of_q, fps=10):
        render = MeshcatViewer(wait_for_open=True, open_meshcat=True, show_frames=False, render_to_animation=True,
                               animation_fps=fps)
        render.add_physx_scene(self.scene)
        self.start_robot.reset_pose(self.q_to_dict(list_of_q[0]))
        self.goal_robot.reset_pose(self.q_to_dict(list_of_q[-1]))
        for q in list_of_q:
            self.robot.reset_pose(self.q_to_dict(q))
            self.robot.update(0.01)
            render.update()
        render.publish_animation()

    def sample_q(self):
        q_min = [j.get_limits()[0] for j in self.robot.movable_joints.values()]
        q_max = [j.get_limits()[1] for j in self.robot.movable_joints.values()]
        return np.random.uniform(q_min, q_max)

    @staticmethod
    def interpolate(a, b, step_size=0.1):
        a, b = np.asarray(a), np.asarray(b)
        max_d = np.linalg.norm(b - a)
        normalized_d = (b - a) / max_d
        return [a + v * normalized_d for v in np.arange(0., max_d, step_size)] + [b]

    def add_obstacle_1(self):
        actor = RigidStatic()
        s = Shape.create_box([0.5, 0.1, 1.0], Material())
        URDFRobot.shape_update_user_data(s, 'color',
                                         gl_color_from_matplotlib('tab:purple', alpha=0.5, return_rgba=True) / 255)

        actor.attach_shape(s)
        actor.set_global_pose([0.5, 0., 0.5])
        self.scene.add_actor(actor)
        self.obstacles.append(actor)

    def add_obstacle_2(self):
        actor = RigidStatic()
        s = Shape.create_box([0.2, 0.2, 0.2], Material())
        URDFRobot.shape_update_user_data(s, 'color',
                                         gl_color_from_matplotlib('tab:purple', alpha=0.5, return_rgba=True) / 255)

        actor.attach_shape(s)
        actor.set_global_pose([0.3, 0.7, 1.5])
        self.scene.add_actor(actor)
        self.obstacles.append(actor)
