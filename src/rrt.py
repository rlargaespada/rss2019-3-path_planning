#!/usr/bin/env python2
"""
    RSS 2019 | rrt.py
    Implementation of RRT* (Rapidly exploring Random Trees) using Dubin
    steering.

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (acuel@mit.edu)
"""
import rospy
from rtree import index
from nav_msgs.msg import OccupancyGrid
import numpy as np
import dubins
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseWithCovarianceStamped, PoseStamped, Pose
import scipy.misc

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
        self.START_TOPIC = rospy.get_param("~start_topic")
        self.GOAL_TOPIC = rospy.get_param("~goal_topic")
        self.start_pose = [0, 0, 0] # x, y, theta
        self.goal_pose = [0, 0, 0] # x, y, theta
        self.goal_size = rospy.get_param("~goal_size")
        self.map_topic = rospy.get_param("~map_topic")
        self.goal_region = {"xmin": 0, "xmax": 0, "ymin": 0, "ymax": 0} # setup in set_goal
        self.map_res = rospy.get_param("~map_res")
        self.PARTICLE_CLOUD_TOPIC = rospy.get_param("~particle_cloud")

        #Initialize visualization varibles
        self.cloud = PointCloud()

        # initialize algorithm parameters
        self.full_region = {
                            "xmin": -26.,
                            "xmax": 8.,
                            "ymin": -11,
                            "ymax": 21
                            }
        self.max_iter = rospy.get_param("~max_iter")
        self.epsilon = rospy.get_param("~epsilon")
        self.neighbor_radius = rospy.get_param("~neighbor_radius")
        self.d = rospy.get_param("~d")
        self.turning_radius = rospy.get_param("~turning_radius")
        self.path_step = rospy.get_param("~path_step")

        # initilize graph structure (start insertion in set_start)
        self.current = [0, 0, 0]
        self.nodes = []
        self.tree = index.Index() # R-Tree for querying neighbors

        # initialize publishers and subscribers
        self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)

        rospy.Subscriber(self.START_TOPIC, PoseWithCovarianceStamped, self.set_start)
        rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.set_goal)

        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def set_start(self, start_pose):
        """
        Gets starting pose from rviz pose estimate marker.
        """
        x, y = start_pose.pose.pose.position.x, start_pose.pose.pose.position.y
        theta = 2*np.arctan(start_pose.pose.pose.orientation.z/start_pose.pose.pose.orientation.w)

        self.start_pose = [x, y, theta]
        start_node = Node(self.start_pose)

        self.current = start_node
        self.nodes.append(start_node)
        self.tree_insert(start_node)

    def set_goal(self, goal_pose):
        """
        Gets goal pose from rviz nav goal marker.
        """
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y

        self.goal_pose = [x, y, 0]
        r = self.goal_size/2

        self.goal_region["xmin"] = x-r
        self.goal_region["xmax"] = x+r
        self.goal_region["ymin"] = y-r
        self.goal_region["ymax"] = y+r

    def map_callback(self, map_msg):
        while self.start_pose == [0, 0, 0] or self.goal_pose == [0, 0, 0]:
            # print (self.start_pose, self.goal_pose)
            continue

        print "Start and Goal intialized:"
        print "Start: ", self.start_pose
        print "Goal: ", self.goal_pose

        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)
        # map_ = np.clip(map_, 0, 1)
        self.map = np.reshape(map_, (map_msg.info.height, map_msg.info.width))
        #Beef up the edges
        # for i in range(self.map.shape[0]):
        #     for j in range(self.map.shape[1]):
        #         if self.map[i, j] != 0:
        #             self.map[i-10: i+10, j-10: j+10] = 1.0
        # Convert the origin to a tuple
        # scipy.misc.imsave("C:Home/map.png", self.map)
        # print(self.map.shape)
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        self.origin = (origin_p.x, origin_p.y, origin_o[2])
        # print(self.origin)
        # test_path = []
        # for i in range(-220, 440):
        #     test_path.append([0, float(i)/20, 0])
        # print(self.in_collision(test_path))
        self.run_rrt()

    def run_rrt(self):
        '''
        Input: None
        Output: Path from Start to Goal Region
        runs rrt algorithm
        '''
        while True:
            # If our current node is in the goal, break
            if self.in_goal(self.current):
                break
            # Get a random pose sample
            next_pose = self.get_next()
            # print("next_pose", next_pose)
            #Get the closest node to our sample
            closest = self.find_nearest_node(next_pose)
            # print("closest", closest.pose, closest.parent)
            #Get actual pose for node
            new_pose = self.steer(closest, next_pose)
            # print("new_pose", new_pose)
            #Get path from dubin. Note this is discretized as units of length
            new_path = self.create_path(closest, new_pose)
            # print("new_path start: ", new_path[0], "end: ", new_path[-1])
            self.create_PointCloud()
            if not self.in_collision(new_path):
                #Cost = distance.  Possible since path is discritized by length
                cost = self.get_cost(new_path)
                # print("cost", cost)
                #Add node to nodes
                new_node = Node(new_pose, closest, new_path, cost)
                self.nodes.append(new_node)
                # insert into tree
                # self.tree_insert(new_node)
                #make current node the node just added
                self.current = self.nodes[-1]
                # print("current_pose", self.current.pose)

        #Define path from the last Node considered to goal
        path_to_goal = self.create_path(self.current, self.goal)
        #Define path of the last Node to goal
        cost = self.get_cost(path_to_goal)
        #Create node at goal to and add to nodes list
        self.end_node = Node(self.goal, self.current, path_to_goal, cost)
        self.nodes.append(self.end_node)
        #Create sequence of nodes from start to goal
        self.node_path = self.plan_node_path(self.end_node)
        # print("node_path", [x.pose for x in self.node_path])
        #Create path of poses from the node_path
        self.pose_path = self.plan_pose_path()
        return self.pose_path


    def steer(self, start_node, next_pose):
        """
        Input: Parent node and proposed next pose [x, y]
        Output: the actual next pose [x, y, theta] (theta in direction of movement)
        """
        dist_ratio = self.d/self.get_dist(start_node.pose, next_pose)
        x = start_node.pose[0] + (next_pose[0] - start_node.pose[0])*dist_ratio
        y = start_node.pose[1] + (next_pose[1] - start_node.pose[1])*dist_ratio
        theta = np.arctan2(y, x)
        return [x, y, theta]

    def create_path(self, start_node, next_pose):
        '''
        Input: Parent node and proposed next pose [x, y, theta]
        Output: configurations with distance path_step between them
        '''
        path = dubins.shortest_path(start_node.pose, next_pose, self.turning_radius)
        configurations, _ = path.sample_many(self.path_step)
        return configurations

    def get_next(self):
        """
        Input: None
        Output: pose [x, y]
        Pose sampled randomly from whole space with probability = self.epsilon
        and otherwise from the goal region
        """
        if np.random.random() < self.epsilon:
            new_x = np.random.uniform(self.full_region["xmin"], self.full_region["xmax"])
            new_y = np.random.uniform(self.full_region["ymin"], self.full_region["ymax"])
            return (new_x, new_y)
        else:
            new_x = np.random.uniform(self.goal_region["xmin"], self.goal_region["xmax"])
            new_y = np.random.uniform(self.goal_region["ymin"], self.goal_region["ymax"])
            return (new_x, new_y)

    def in_collision(self, path):
        """
        True if path is not collision free, False otherwise.
        """
        path_for_map = np.array(path)
        #Take only x and y
        path_for_map = path_for_map[:, :2]
        #subtract origin from position
        path_for_map -= np.tile(np.array([self.origin[:2]]), (len(path), 1))
        #Resize to fit dimensions of map
        path_for_map /= .05
        #Cast to int for indexing
        path_for_map = path_for_map.astype(int)
        # get values from map
        map_values = self.map[path_for_map[:, 0], path_for_map[:, 1]]
        # return whether there are any non-zero values
        return np.count_nonzero(map_values) != 0

    def get_dist(self, pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def get_cost(self, path):
        # Cost = distance.  Possible since path is discritized by length
        return len(path) * self.path_step

    # def find_nearest_node(self, pose):
    #     '''
    #     Input: pose [x, y]
    #     Output: Node closest to given pose
    #     '''
    #     min_dist = float("inf")
    #     parent_node = None
    #     for node in self.nodes:
    #         if self.get_dist(node.pose, pose) < min_dist:
    #             parent_node = node
    #     return parent_node

    def find_nearest_node(self, pose):
        """
        Output: Node closest to given node
        """
        x, y = pose[0], pose[1]
        nearest = list(self.tree.nearest((x, y, x, y), 2)) # return two nearest because first nearest will always be itself

        nearest_neighbor = self.nodes[nearest[-1]] # get node item from node list
        # print nearest_neighbor
        return nearest_neighbor

    def in_goal(self, node):
        '''
        Input: Node object
        Output: Boolean representing if node is in goal region
        '''
        if node.pose[0] < self.goal_region["xmin"] and node.pose[0] > self.goal_region["xmax"] and node.pose[1] < self.goal_region["ymin"] and node.pose[1] > self.goal_region["ymax"]:
            return True

    def tree_insert(self, node):
        """
        Insert a node into the R-tree
        """
        x, y = node.pose[0], node.pose[1]
        # d = self.map_res/2
        # xmin, ymin, xmax, ymax = x-d, x+d, y-d, y+d

        # self.tree.insert(node, (xmin, ymin, xmax, ymax))
        self.tree.insert(node.id, (x, y, x, y))

    def find_neighbors(self, node):
        """
        Input: node: (Node) node around which to query for neighbors
        Output: list of indices of nodes that fall within the box defined by
                neighbor_radius
        """
        x, y = node.pose[0], node.pose[1]
        nr = self.neighbor_radius
        box = (x-nr, x+nr, y-nr, y+nr)

        neighbor_idxs = list(self.tree.intersection(box))

        return neighbor_idxs

    def rewire(self):
        """
        Rewires graph by connecting the current node to the minimum cost path
        """
        curr = self.current
        neighbor_idxs = self.find_neighbors(curr)

        # find best parent for the current node
        for n_idx in neighbor_idxs:
            n = self.nodes[n_idx]
            possible_path = self.create_path(curr, n.pose)
            if not in_collision(possible_path):
                possible_cost = n.cost + self.get_cost(possible_path)
                if possible_cost < curr.cost:
                    # better path found
                    curr.set_parent(n)

        # Check if existing paths can be improved by connecting through current node
        for n_idx in neighbor_idxs:
            n = self.nodes[n_idx]
            possible_path = self.create_path(curr, n.pose)
            if not in_collision(possible_path):
                if n.cost > curr.cost + self.get_cost(possible_path):
                    # set parent of neighbor to current node
                    print "Rewiring"
                    n.set_parent(curr)

    def plan_node_path(self, node):
        """
        Input: Node object
        Output: list of Node objects from parent to child
        """
        if node == None:
            return []
        return self.plan_node_path(node.parent) + [node]

    def plan_pose_path(self):
        '''
        Input: None
        Output: List of poses from first Node in self.node_path to
                lst node in self.node_path
        '''
        path = []
        for node in self.node_path:
            if node.path != None:
                path += node.path
        return path

    def create_PointCloud(self):
        '''
        Create and publish point cloud of particles and current pose marker
        '''
        self.cloud.header.frame_id = "/map"
        self.cloud.points = [Point32() for i in range(len(self.nodes))]
        for node in range(len(self.nodes)):
            self.cloud.points[node].x = self.nodes[node].pose[0]
            self.cloud.points[node].y = self.nodes[node].pose[1]
            self.cloud.points[node].z = 0
        # print(self.cloud.points[0].x, self.cloud.points[0].y, self.cloud.points[0].z)
        self.particle_cloud_publisher.publish(self.cloud)

class Node:
    """
    RRT graph node
    """
    id = 0 # counter for nodes

    def __init__(self, pose, parent = None, path = None, cost = 0.0):
        self.pose = pose # [x, y, theta]
        self.path = path # series of poses from parent to self
        self.parent = parent
        self.cost = cost # distance to source along edges

        self.id = Node.id # self.id = index in RRT.nodes in RRT class
        Node.id += 1

    def add_to_path(self, pose):
        """
        Adds a pose to path generated from Dubin steering
        """
        self.path.append(pose)

    def set_parent(self, parent):
        self.parent = parent


if __name__ == "__main__":
    rospy.init_node("rrt")
    rrt = RRTstar()
    rospy.spin()
