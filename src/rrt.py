"""
    RSS 2019 | rrt.py
    Implementation of RRT* (Rapidly exploring Random Trees) using Dubin
    steering.

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (acuel@mit.edu)
"""
import rospy

class RRTstar:
    """
    RRT* using Dubin Steering
    """
    def __init__(self):
        """
        start = [x, y, theta]
        goal = [x, y]
        """
        # initialize start and goal parameters
        self.start = rospy.get_param("~start_pose")
        self.goal = rospy.get_param("~goal_pose")
        self.goal_size = rospy.get_param("~goal_size")
        self.goal_region = {"xmin": goal[0] - self.goal_size/2,
                            "xmax": goal[0] + self.goal_size/2,
                            "ymin": goal[1] - self.goal_size/2,
                            "ymax": goal[1] + self.goal_size/2}

        # initialize algorithm parameters
        self.max_iter = rospy.get_param("~max_iter")
        self.epsilon = rospy.get_param("~epsilon")
        self.k = rospy.get_param("~k")
        self.d = rospy.get_param("~d")

        # TODO(alex): initialize publishers and subscribers

    def steer(self, start_node, next_node):
        """
        """
        # TODO(alex)
        pass

    def get_next(self):
        """
        """
        # TODO(alex)
        pass

    def check_collision(self, path):
        """
        """
        # TODO(alex)
        pass

    def find_nearest(self, node):
        """
        """
        # TODO(abbie)
        pass

    def rewire(self, new_node, knn):
        """
        """
        # TODO(abbie)
        pass

    def plan_path(self):
        """
        """
        # TODO(alex)
        pass

class Node:
    """
    RRT graph node
    """
    def __init__(self, x, y, theta, parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.path = [] # series of poses from parent to self
        self.parent = parent
        self.cost = 0.0 # distance to parent along dubin path

    def add_to_path(self, pose):
        """
        Adds a pose to path generated from Dubin steering
        """
        self.path.append(pose)
