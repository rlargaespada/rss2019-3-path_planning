"""
    RSS 2019 | rrt.py
    Implementation of RRT* (Rapidly exploring Random Trees) using Dubin
    steering.

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (acuel@mit.edu)
"""
import rospy
from sklearn.neighbors import KDTree

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
        self.neighbor_radius = rospy.get_param("~neighbor_radius")
        self.d = rospy.get_param("~d")

        # initilize graph structure
        self.nodes = []
        self.tree = KDTree([n.pose for n in self.nodes], leaf_size=20, metric='euclidean')

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

    def in_collision(self, path):
        """
        """
        # TODO(alex)
        pass

    def find_nearest(self, node):
        """
        Input: node: (Node) node around which to query for neighbors
        Output: idx, dists: indices of neighbors within neighbor_radius and their distances
        """
        # TODO(abbie)
        idx, dists = self.tree.query_radius([node.pose], r=self.neighbor_radius, return_distance=True, sort_results=True)
        return idx, dists

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
    def __init__(self, pose, parent = None):
        self.pose = pose # [x, y, theta]
        self.path = [] # series of poses from parent to self
        self.parent = parent
        self.cost = 0.0 # distance to source along edges

    def add_to_path(self, pose):
        """
        Adds a pose to path generated from Dubin steering
        """
        self.path.append(pose)
