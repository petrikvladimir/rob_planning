#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2021-12-14
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import tqdm
from anytree import AnyNode, PreOrderIter, Walker
import numpy as np


class RRT:
    def __init__(self, collision_fn, sample_fn, delta_q=0.2, path_fn=None) -> None:
        self.path_fn = path_fn
        self.collision_fn = collision_fn
        self.sample_fn = sample_fn
        self.delta_q = delta_q

    def plan(self, q0, q1, max_iterations=10000):
        root = AnyNode(q=np.asarray(q0))
        nodes = []
        for _ in tqdm.trange(max_iterations):
            q_rand = np.asarray(self.sample_fn(q1))
            n_tree: AnyNode = min(PreOrderIter(root), key=lambda n: np.linalg.norm(q_rand - n.q))
            direction = (q_rand - n_tree.q)
            direction = direction / np.linalg.norm(direction)
            q_new = n_tree.q + self.delta_q * direction
            if not self.collision_fn(q_new) and (self.path_fn is None or self.path_fn(n_tree.q, q_new)):
                nodes.append(AnyNode(q=q_new, parent=n_tree))
            if np.linalg.norm(q_new - q1) < self.delta_q and (self.path_fn is None or self.path_fn(q_new, q1)):
                goal = AnyNode(q=q1, parent=nodes[-1])
                return [q0] + [n.q for n in Walker().walk(root, goal)[2]]
