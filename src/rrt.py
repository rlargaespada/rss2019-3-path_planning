"""
    RSS 2019 | rrt.py
    Implementation of RRT* (Rapidly exploring Random Trees) using Dubin
    steering.

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (acuel@mit.edu)
"""

class RRT:
    """
    """
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal


class Node:
    """
    RRT graph node
    """
    def __init__(self, x, y, theta, parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        self.cost = 0.0 # distance to parent along dubin path
