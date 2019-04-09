#!/usr/bin/env python2
"""
    RSS 2019 | rrt.py
    Implementation of RRT* (Rapidly exploring Random Trees) using Dubin
    steering.

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (acuel@mit.edu)
"""
import rospy
from sklearn.neighbors import KDTree
from nav_msgs.msg import OccupancyGrid
import numpy as np
import dubins

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
        self.map_topic = rospy.get_param("~map_topic")
        self.goal_region = {"xmin": self.goal[0] - self.goal_size/2,
                            "xmax": self.goal[0] + self.goal_size/2,
                            "ymin": self.goal[1] - self.goal_size/2,
                            "ymax": self.goal[1] + self.goal_size/2}

        # initialize algorithm parameters
        self.full_region = {
                            "xmin": 0, 
                            "xmax": 100, 
                            "ymin":0, 
                            "ymax":100
                            }
        self.max_iter = rospy.get_param("~max_iter")
        self.epsilon = rospy.get_param("~epsilon")
        self.neighbor_radius = rospy.get_param("~neighbor_radius")
        self.d = rospy.get_param("~d")
        self.turning_radius = rospy.get_param("~turning_radius")
        self.path_step = rospy.get_param("~path_step")
        self.sample_from_all = .8

        # initilize graph structure
        self.nodes = []
        self.start_node = Node(self.start)
        self.current = self.start_node
        # self.tree = KDTree(np.array([n.pose for n in self.nodes]), leaf_size=20, metric='euclidean')

        # initialize publishers and subscribers
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)/100.
        map_ = np.clip(map_, 0, 1)
        print(map_.shape)
        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation

    def run_rrt(self):
        '''
        Input: None
        Output: Path from Start to Goal Region
        runs rrt algorithm
        '''
        while True:
            #If our current node is in the goal, break
            if self.in_goal(self.current):
                break
            #Get a random pose sample
            sample = self.get_next()
            #Get the closest node to our sample
            closest = self.find_nearest_node(sample)
            #Get actual pose for node
            new_pose = self.steer(closest, sample)
            #Get path from dubin. Note this is discretized as units of length
            new_path = self.create_path(closest, new_pose)

            if not is_collision(new_path):
                #Cost = distance.  Possible since path is discritized by length
                cost = len(new_path)*self.path_step()
                #Add node to nodes
                self.nodes.append(Node(new_pose, closest, new_path, cost))
                #make current node the node just added
                self.current = self.nodes[-1]
                



    def steer(self, start_node, next_pose):
        """
        Input: Parent node and proposed next pose [x, y]
        Output: the actual next pose [x, y, theta] (theta in direction of movement)
        """
        x = start_node.pose[0] + next_pose[0]*self.d
        y = start_node.pose[1] + next_pose[0]*self.d
        theta = np.arctan2(y, x)
        return [new_x, new_y, new_theta]

    def create_path(self, start_node, next_pose):
        '''
        Input: Parent node and proposed next pose [x, y, theta]
        Output: configurations with distance path_step between them
        '''
        path = dubins.shortest_path(start_node.pose, next_pose)
        configurations, _ = path.sample_many(self.)
        return configurations

    def get_next(self):
        """
        Input: None
        Output: pose [x, y] 
        Pose sampled randomly from whole space with probability = self.sample_from_all 
        and otherwise from the goal region
        """
        if np.random.random() < self.sample_from_all:
            new_x = np.random.uniform(self.full_region["xmin"], self.full_region["xmax"])
            new_y = np.random.uniform(self.full_region["ymin"], self.full_region["ymax"])
            return (new_x, new_y)
        else:
            new_x = np.random.uniform(self.goal_region["xmin"], self.goal_region["xmax"])
            new_y = np.random.uniform(self.goal_region["ymin"], self.goal_region["ymax"])
            return (new_x, new_y)

    def in_collision(self, path):
        """
        """
        # TODO(alex)
        pass

    def get_dist(pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def find_nearest_node(pose):
        '''
        Input: pose [x, y]
        Output: Node closest to given pose
        '''
        min_dist = float("inf")
        for node in self.nodes:
            if get_dist(node.pose, pose) < min_dist:
                parent_node = node
        return parent_node

    def in_goal(node):
        '''
        Input: Node object
        Output: Boolean representing if node is in goal region
        '''
        if node.pose[0] < self.goal_region["xmin"] and node.pose[0] > goal_region["xmax"] and node.pose[1] < self.goal_region["ymin"] and node.pose[1] > goal_region["ymax"]:
            return True

    def find_nearest_k(self, node):
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
    def __init__(self, pose, parent = None, path = None, cost = 0.0):
        self.pose = pose # [x, y, theta]
        self.path = path # series of poses from parent to self
        self.parent = parent
        self.cost = cost # distance to source along edges

    def add_to_path(self, pose):
        """
        Adds a pose to path generated from Dubin steering
        """
        self.path.append(pose)


if __name__ == "__main__":
    rospy.init_node("rrt")
    pf = RRTstar()
    rospy.spin()