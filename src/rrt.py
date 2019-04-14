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
from geometry_msgs.msg import Point32, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
import scipy.misc
import matplotlib.pyplot as plt

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
        self.max_angle = rospy.get_param("~max_angle")
        self.PARTICLE_CLOUD_TOPIC = rospy.get_param("~particle_cloud")
        self.PATH_TOPIC = rospy.get_param("~path_topic")
        self.base_link = rospy.get_param("~base_link")
        self.map_name = rospy.get_param("~map")

        #Initialize visualization varibles
        self.cloud = PointCloud()

        # initialize algorithm parameters
        self.max_iter = rospy.get_param("~max_iter")
        self.epsilon = rospy.get_param("~epsilon")
        self.neighbor_radius = rospy.get_param("~neighbor_radius")
        self.d = rospy.get_param("~d")
        self.turning_radius = rospy.get_param("~turning_radius")
        self.path_step = rospy.get_param("~path_step")
        self.buff_factor = rospy.get_param("~buff_map")

        # initilize graph structure (start insertion in set_start)
        self.current = [0, 0, 0]
        self.nodes = []
        self.tree = index.Index() # R-Tree for querying neighbors

        # initialize publishers and subscribers
        self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)
        self.path_publisher = rospy.Publisher(self.PATH_TOPIC, Path, queue_size=10)

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
        self.map = np.reshape(map_, (map_msg.info.height, map_msg.info.width)).T
        # self.map = np.flip(self.map, axis=0)
        self.map_copy = np.copy(self.map)
        print(self.map.shape)
        #Beef up the edges
        # for i in range(self.map_copy.shape[0]):
        #     for j in range(self.map_copy.shape[1]):
        #         if self.map_copy[i, j] != 0:
        #             self.map[i-self.buff_factor: i+self.buff_factor, j-self.buff_factor: j+self.buff_factor] = 1.0
        # Convert the origin to a tuple
        # plt.imshow(self.map)
        # plt.show()
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        self.origin = (origin_p.x, origin_p.y, origin_o[2])
        if self.map_name == "stata_basement":
            self.full_region = {
                                "xmin": -map_msg.info.width*self.map_res + float(self.origin[0]),
                                "xmax": float(self.origin[0]),
                                "ymin": -map_msg.info.height*self.map_res + float(self.origin[1]),
                                "ymax": float(self.origin[1])
                                }
            self.map_flip_const = -1.
        if self.map_name == "building_31":
            self.full_region = {
                                "xmin": float(self.origin[0]),
                                "xmax": map_msg.info.width*self.map_res + float(self.origin[0]),
                                "ymin": float(self.origin[1]),
                                "ymax": map_msg.info.height*self.map_res + float(self.origin[1])
                                }
            self.map_flip_const = 1.
        print("HELLO", self.full_region)
        # self.in_collision(np.array([[0., 0.], [0., 9.]]))
        self.run_rrt()

    def run_rrt(self):
        '''
        Input: None
        Output: Path from Start to Goal Region
        runs rrt algorithm
        '''
        counter = 0
        overflow_limit = float("inf")
        already_found = False
        while counter < overflow_limit:
            # If our current node is in the goal, break
            counter += 1
            print(counter)
            if self.in_goal(self.current) and already_found == False:
                path_to_goal = self.create_path(self.current, self.goal_pose)
                cost = self.get_cost(path_to_goal)
                self.end_node = Node(self.goal_pose, self.current, path_to_goal, cost)
                self.nodes.append(self.end_node)
                print("END")
                # break
                overflow_limit = 1.5*counter
                already_found = True
                # turn off goal biasing
                # self.epsilon = 1.0
            # Get a random pose sample
            next_pose = self.get_next()
            #Get the closest node to our sample
            closest = self.find_nearest_node(next_pose)
            #Get actual pose for node
            new_pose = self.steer(closest, next_pose)
            #Get path from dubin. Note this is discretized as units of length
            new_path = self.create_path(closest, new_pose)
            # print(np.max(np.array(new_path), axis = 0))
            self.create_PointCloud(self.nodes)
            if not self.in_collision(new_path):
                cost = self.get_cost(new_path) + closest.cost
                # Add node to nodes
                new_node = Node(new_pose, closest, new_path, cost)
                self.nodes.append(new_node)
                # insert into tree
                self.tree_insert(new_node)
                #make current node the node just added
                self.current = new_node
                self.rewire()
                # print("current_pose", self.current.pose)

        #Define path from the last Node considered to goal
        # path_to_goal = self.create_path(self.current, self.goal_pose)
        #Define path of the last Node to goal
        # cost = self.get_cost(path_to_goal)
        #Create node at goal to and add to nodes list
        # self.end_node = Node(self.goal_pose, self.current, path_to_goal, cost)
        # self.nodes.append(self.end_node)
        self.node_path = self.plan_node_path(self.end_node)
        # self.nodes.append(self.end_node)

        for node in self.node_path[4:]:
            self.check_ancestors(node)

        self.node_path = self.plan_node_path(self.end_node)
        self.create_PointCloud(self.node_path)
        #Create path of poses from the node_path
        self.pose_path = self.plan_pose_path()
        print(len(self.pose_path))
        self.draw_path(self.pose_path)
        return self.pose_path


    def steer_alt(self, start_node, next_pose):
        """
        Input: Parent node and proposed next pose [x, y]
        Output: the actual next pose [x, y, theta] (theta in direction of movement)
        """
        #Represents the theta that the wheels should turn: max(min(angle between start pose and new point, start angle + max_angle), start angle - max_angle)
        theta = max(min((-np.arctan2(next_pose[1] - start_node.pose[1], next_pose[0] - start_node.pose[0]) + start_node.pose[2]), start_node.pose[2] + self.max_angle), start_node.pose[2] - self.max_angle)
        print("thetaaaaaaaa:", theta)
        theta = self.angle_correct(theta)
        #Rotation matrix based starting position
        rotation = np.array([[np.cos(start_node.pose[2]), -np.sin(start_node.pose[2]), 0],
                             [np.sin(start_node.pose[2]), np.cos(start_node.pose[2]),  0],
                             [0, 0, 1]])
        #Turning radius based on wheels angled at theta
        R = self.base_link/np.tan(theta) #Radius of circle
        #Angle the robot wants to sweep out long the circle with R radus given distance d it drives during the steer function
        phi = 2*np.pi*(self.d/(2*np.pi*R))
        #Angle between the starting and ending poses of the robot with reference to the perpendicular to the car's direction
        beta = (np.pi - phi)/2
        #striaght line distance the car will travel during steer
        L = 2*R*np.cos(beta)
        #change in [x, y, theta] from start pose to end of steer in base_link reference frame
        delta_np = np.array([L*np.cos(beta), L*np.sin(beta), phi])
        #New pose after steer in map reference frame
        new = np.array(start_node.pose) + np.matmul(rotation, delta_np).T
        new = new.tolist()
        # print("theta: ", theta)
        # print("R", R)
        # print("start: ", start_node.pose)
        # print("next: ", next_pose)
        # print("steer: ", new)
        return new

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
        # path = dubins.shortest_path(start_node.pose, next_pose, self.turning_radius)
        # configurations, _ = path.sample_many(self.path_step)
        # return configurations
        path = []
        dist_ratio = self.path_step/self.get_dist(start_node.pose, next_pose)
        dx = (next_pose[0] - start_node.pose[0])*dist_ratio
        dy = (next_pose[1] - start_node.pose[1])*dist_ratio
        theta = np.arctan2(dy, dx)
        x_vals = np.arange(start_node.pose[0], next_pose[0], dx)
        y_vals = np.arange(start_node.pose[1], next_pose[1], dy)
        theta_vals = np.tile(theta, x_vals.size)
        path = np.column_stack((x_vals, y_vals, theta_vals))
        return path

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
            new = (new_x, new_y)
        else:
            new_x = np.random.uniform(self.goal_region["xmin"], self.goal_region["xmax"])
            new_y = np.random.uniform(self.goal_region["ymin"], self.goal_region["ymax"])
            new = (new_x, new_y)

        if not self.in_collision([new]):
            return new
        else:
            return self.get_next()

    def in_collision(self, path):
        """
        True if path is not collision free, False otherwise.
        """
        path_for_map = np.array(path)
        #Take only x and y
        path_for_map = path_for_map[:, :2]*self.map_flip_const
        #subtract origin from position
        path_for_map -= np.tile(np.array([self.origin[0], self.origin[1]]), (len(path), 1))*self.map_flip_const
        # print(path_for_map)
        #Resize to fit dimensions of map
        path_for_map /= self.map_res
        # print(path_for_map)
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
    #         print(node.pose[2])
    #         if self.get_dist(node.pose, pose) + 3*abs((-np.arctan2(pose[1] - node.pose[1], pose[0] - node.pose[0]) + self.angle_correct(node.pose[2]))) < min_dist:
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
        if node.pose[0] > self.goal_region["xmin"] and node.pose[0] < self.goal_region["xmax"] and node.pose[1] > self.goal_region["ymin"] and node.pose[1] < self.goal_region["ymax"]:
            return True

    def tree_insert(self, node):
        """
        Insert a node into the R-tree
        """
        # print "Inserting into tree"
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
        box = (x-nr, y-nr, x+nr, y+nr)

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
            if curr.pose != n.pose:
                possible_path = self.create_path(curr, n.pose)
                if not self.in_collision(possible_path):
                    # print(curr.pose, n.pose)
                    possible_cost = n.cost + self.get_cost(possible_path)
                    if possible_cost < curr.cost:
                        print("rwrng")
                        # better path found
                        curr.set_parent(n)

        # Check if existing paths can be improved by connecting through current node
        for n_idx in neighbor_idxs:
            n = self.nodes[n_idx]
            if curr.pose != n.pose:
                possible_path = self.create_path(curr, n.pose)
                if not self.in_collision(possible_path):
                    if n.cost > curr.cost + self.get_cost(possible_path):
                        # print(n.pose, curr.pose)
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

        path = np.vstack([x.path for x in self.node_path[1:]])
        return path

    def check_ancestors(self, node):
        parent = node.parent
        grandparent = parent.parent
        path = self.create_path(grandparent, node.pose)
        if not self.in_collision(path):
            cost = self.get_cost(path)
            if cost + grandparent.cost < node.cost and cost < 1:
                print("CONNECT WITH YOUR ROOTS")
                node.parent = grandparent

    def create_PointCloud(self, nodes):
        '''
        Create and publish point cloud of particles and current pose marker
        '''
        self.cloud.header.frame_id = "/map"
        self.cloud.points = [Point32() for i in range(len(nodes))]
        for node in range(len(nodes)):
            self.cloud.points[node].x = nodes[node].pose[0]
            self.cloud.points[node].y = nodes[node].pose[1]
            self.cloud.points[node].z = 0
        self.particle_cloud_publisher.publish(self.cloud)

    def draw_path(self, pos_path):
        header = Header()
        path = Path()
        header.stamp = rospy.rostime.Time.now()
        header.frame_id = "/map"
        pose_stamp = PoseStamped()
        pose_stamp.header = header
        for pos in pos_path:
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = pos[2]
            orient = Quaternion()
            quat = tf.transformations.quaternion_from_euler(0, 0,pos[2])
            orient.x = quat[0]
            orient.y = quat[1]
            orient.z = quat[2]
            orient.w = quat[3]
            pose = Pose()
            pose.position = point
            pose.orientation = orient
            pose_stamp.pose = pose
            path.poses.append(pose_stamp)
        path.header = pose_stamp.header
        self.path_publisher.publish(path)

    def angle_correct(self, theta):
        theta %= 2*np.pi
        if theta > np.pi:
            theta = theta - 2*np.pi
        return theta

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
