#!/usr/bin/env python2
"""
    RSS 2019 | rrt.py
    Implementation of RRT* (Rapidly exploring Random Trees) with path smoothing

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (alexcuel@mit.edu)
"""
import rospy
from rtree import index
from nav_msgs.msg import OccupancyGrid
import numpy as np
import dubins
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from nav_msgs.msg import Path
from lab6.msg import PathData

import scipy.misc
import matplotlib.pyplot as plt

class RRTstar:
    """
    RRT* with path smoothing
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
        self.secondary_goal_multiplier = rospy.get_param("~big_region_size")
        self.goal_region = {"xmin": 0, "xmax": 0, "ymin": 0, "ymax": 0} # setup in set_goal
        self.map_res = rospy.get_param("~map_res")
        self.max_angle = rospy.get_param("~max_angle")
        self.PARTICLE_CLOUD_TOPIC = rospy.get_param("~particle_cloud")
        self.TREE_TOPIC = rospy.get_param("~tree_topic")
        self.PATH_TOPIC = rospy.get_param("~path_topic")
        self.base_link = rospy.get_param("~base_link")
        self.map_name = rospy.get_param("~map")
        self.origin_x_offset = rospy.get_param("~origin_x_offset")
        self.origin_y_offset = rospy.get_param("~origin_y_offset")
        self.POSE_PATH_TOPIC = rospy.get_param("~path_send_topic")

        #Initialize visualization varibles
        self.cloud = PointCloud()
        self.vis_tree = Marker()

        # initialize algorithm parameters
        self.max_iter = rospy.get_param("~max_iter")
        self.epsilon_all = rospy.get_param("~epsilon_all")
        self.epsilon_big = rospy.get_param("~epsilon_big")
        self.neighbor_radius = rospy.get_param("~neighbor_radius")
        self.d = rospy.get_param("~d")
        self.turning_radius = rospy.get_param("~turning_radius")
        self.path_step = rospy.get_param("~path_step")
        self.buff_factor = rospy.get_param("~buff_map")
        self.counter = 0
        self.NUM_GOAL_REGIONS = rospy.get_param("~num_goal_regions")

        # initilize graph structure (start insertion in set_start)
        # self.current = [0, 0, 0]
        # self.nodes = []
        # self.tree = index.Index() # R-Tree for querying neighbors
        # self.end_node = None
        self.goal_poses = []
        self.goal_regions = []
        self.goal_regions_secondary = []

        # initialize publishers and subscribers
        self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)
        self.tree_pub = rospy.Publisher(self.TREE_TOPIC, Marker, queue_size=10)
        self.path_publisher = rospy.Publisher(self.PATH_TOPIC, Path, queue_size=10)
        self.full_path_pub = rospy.Publisher(self.POSE_PATH_TOPIC, PointCloud, queue_size=10)
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
        # start_node = Node(self.start_pose)

        # self.current = start_node
        # self.nodes.append(start_node)
        # self.tree_insert(start_node)

    def set_goal(self, goal_pose):
        """
        Gets goal pose from rviz nav goal marker.
        """
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y

        self.goal_poses.append([x, y, 0])
        r = self.goal_size/2
        goal_region = {}
        goal_region["xmin"] = x-r
        goal_region["xmax"] = x+r
        goal_region["ymin"] = y-r
        goal_region["ymax"] = y+r
        self.goal_regions.append(goal_region)
        goal_region_secondary = {}
        goal_region_secondary["xmin"] = x-self.secondary_goal_multiplier*r
        goal_region_secondary["xmax"] = x+self.secondary_goal_multiplier*r
        goal_region_secondary["ymin"] = y-self.secondary_goal_multiplier*r
        goal_region_secondary["ymax"] = y+self.secondary_goal_multiplier*r
        self.goal_regions_secondary.append(goal_region)

    def map_callback(self, map_msg):
        while self.start_pose == [0, 0, 0] or len(self.goal_regions) < self.NUM_GOAL_REGIONS:
            continue
        self.checkpoints = [self.start_pose] + self.goal_poses
        print(self.checkpoints)
        print "Loading map:", rospy.get_param("~map"), "..."
        print "Start and Goal intialized:"
        print "Start: ", self.start_pose
        print "Goal: ", self.goal_pose
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)
        # map_ = np.clip(map_, 0, 1)
        self.map = np.reshape(map_, (map_msg.info.height, map_msg.info.width)).T
        # self.map = np.flip(self.map, axis=0)
        self.map_copy = np.copy(self.map)
        #Beef up the edges
        for i in range(self.map_copy.shape[0]):
            for j in range(self.map_copy.shape[1]):
                if self.map_copy[i, j] != 0:
                    self.map[i-self.buff_factor: i+self.buff_factor, j-self.buff_factor: j+self.buff_factor] = 1.0
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
        self.origin = (origin_p.x+self.origin_x_offset, origin_p.y+self.origin_y_offset, origin_o[2])
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

        self.full_node_path = []
        # self.full_pose_path = []
        for checkpoint in range(self.NUM_GOAL_REGIONS):
            self.start_pose = self.checkpoints[checkpoint]
            self.start_node = Node(self.start_pose, start=True)
            self.goal_pose = self.checkpoints[checkpoint + 1]
            self.goal_region = self.goal_regions[checkpoint]
            self.goal_region_secondary = self.goal_regions_secondary[checkpoint]
            self.counter = 0
            self.nodes = []
            self.tree = index.Index()
            self.end_node = None
            self.current = self.start_node
            self.nodes.append(self.start_node)
            self.tree_insert(self.start_node)
<<<<<<< HEAD
            next_path = self.run_rrt() # list of nodes
            # self.full_pose_path.extend(next_path)
            if self.full_node_path != []:
                next_path[0].set_parent(self.full_node_path[-1])
            self.full_node_path.extend(next_path)

        print "Len path:", len(self.full_node_path)

        for node in self.full_node_path[4:]:
            self.check_ancestors(node)

        self.final_path = self.plan_node_path(self.full_node_path[-1])
        start = self.final_path[0]
        end = self.final_path[-1]
        start.set_parent(end)

        # make path between start and end
        loop_path = self.create_path(end, start.pose)
        start.set_path(loop_path)

        self.full_pose_path = self.plan_pose_path(self.final_path)
        self.create_PointCloud_pose(self.full_pose_path)
        self.draw_path(self.full_pose_path)
=======
            next_path = self.run_rrt()
            self.full_pose_path.extend(next_path)
        # print([[x[0], x[1]] for x in self.full_pose_path])
        # print(self.full_pose_path)
        self.create_PointCloud_pose(self.full_pose_path, self.full_path_pub)
        # self.draw_path(self.full_pose_path)
>>>>>>> simple pure pursuit and rrt

    def run_rrt(self):
        '''
        Input: None
        Output: Path from Start to Goal Region
        runs rrt algorithm
        '''
        already_found = False
        self.max_iter = rospy.get_param("~max_iter")
        while self.counter < self.max_iter:
            # If our current node is in the goal, break
            self.counter += 1
            if self.in_goal(self.current) and already_found == False:
                print "FOUND GOAL IN", self.counter, "STEPS."
                path_to_goal = self.create_path(self.current, self.goal_pose)
                cost = self.get_cost(path_to_goal) + self.current.cost

                # Save first node found inside goal region
                self.end_node = Node(self.goal_pose, self.current, path_to_goal, cost)
                self.nodes.append(self.end_node)
                self.node_path = self.plan_node_path(self.end_node)

                self.max_iter = 1.3*self.counter # run for 1.5 the amount of time it took to find goal in order to optimize
                already_found = True

            # Get a random pose sample
            if not already_found:
                next_pose = self.get_next()
            else:
                # draw samples along path to goal
                next_pose = self.optimize_path_next()
                #Create path of poses from the node_path
                self.node_path = self.plan_node_path(self.end_node)
<<<<<<< HEAD
                self.pose_path = self.plan_pose_path(self.node_path)
                self.create_PointCloud_pose(self.pose_path)
=======
                self.pose_path = self.plan_pose_path()
                self.create_PointCloud_pose(self.pose_path, self.particle_cloud_publisher)
>>>>>>> simple pure pursuit and rrt
            #Get the closest node to our sample
            closest_multiple = self.find_nearest_node(next_pose)
            #Get actual pose for node
            for closest in closest_multiple:
                new_pose = self.steer(closest, next_pose)
                # Get path from dubin. Note this is discretized as units of length
                new_path = self.create_path(closest, new_pose)

                # self.create_PointCloud(self.nodes)
                self.create_vistree()
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
                    break

        if self.end_node is None:
            print "Failed to find goal within", self.max_iter, "steps."
            return None

        # Get path to source from goal node
        self.node_path = self.plan_node_path(self.end_node)

        for node in self.node_path[4:]:
            self.check_ancestors(node)

        self.node_path = self.plan_node_path(self.end_node)
        self.create_PointCloud(self.node_path)
        #Create path of poses from the node_path
<<<<<<< HEAD
        self.pose_path = self.plan_pose_path(self.node_path)
        self.create_PointCloud_pose(self.pose_path)
=======
        self.pose_path = self.plan_pose_path()
        self.create_PointCloud_pose(self.pose_path, self.particle_cloud_publisher)
>>>>>>> simple pure pursuit and rrt
        print "Length of path:", len(self.pose_path)
        # return self.pose_path
        return self.node_path

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
        path = []
        dist_ratio = self.path_step/self.get_dist(start_node.pose, next_pose)
        dx = (next_pose[0] - start_node.pose[0])*dist_ratio
        dy = (next_pose[1] - start_node.pose[1])*dist_ratio
        theta = np.arctan2(dy, dx)
        x_vals = np.arange(start_node.pose[0], next_pose[0], dx)
        y_vals = np.arange(start_node.pose[1], next_pose[1], dy)
        theta_vals = np.tile(theta, x_vals.size)
        path = np.column_stack((x_vals, y_vals, theta_vals)).tolist()
        return path

    def get_next(self):
        """
        Input: None
        Output: pose [x, y]
        Pose sampled randomly from whole space with probability = self.epsilon
        and otherwise from the goal region
        """
        num = np.random.random()
        if num < self.epsilon_all:
            new_x = np.random.uniform(self.full_region["xmin"], self.full_region["xmax"])
            new_y = np.random.uniform(self.full_region["ymin"], self.full_region["ymax"])
            new = (new_x, new_y)
        elif num < self.epsilon_all + self.epsilon_big:
            new_x = np.random.uniform(self.goal_region["xmin"], self.goal_region["xmax"])
            new_y = np.random.uniform(self.goal_region["ymin"], self.goal_region["ymax"])
            new = (new_x, new_y)
        else:
            new_x = np.random.uniform(self.goal_region_secondary["xmin"], self.goal_region_secondary["xmax"])
            new_y = np.random.uniform(self.goal_region_secondary["ymin"], self.goal_region_secondary["ymax"])
            new = (new_x, new_y)
        return new
        # if not self.in_collision([new]):
        #     return new
        # else:
        #     return self.get_next()

    def optimize_path_next(self):
        """
        Input: None
        Output: next pose to steer towards from near the path
        """
        # select a random point along the current path to goal
        rand_idx = np.random.randint(len(self.node_path))
        x, y, theta = tuple(self.node_path[rand_idx].pose)
        nr = 1.5 * self.neighbor_radius

        rand_x = np.random.uniform(x-nr, x+nr)
        rand_y = np.random.uniform(y-nr, y+nr)

        return (rand_x, rand_y)

    def in_collision(self, path):
        """
        Input: list of poses that represent a path
        Output: True if path is not collision free, False otherwise.
        """
        path_for_map = np.array(path)
        # Take only x and y
        path_for_map = path_for_map[:, :2]*self.map_flip_const
        # Subtract origin from position
        path_for_map -= np.tile(np.array([self.origin[0], self.origin[1]]), (len(path), 1))*self.map_flip_const
        # Resize to fit dimensions of map
        path_for_map /= self.map_res
        # Cast to int for indexing
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

    def find_nearest_node(self, pose):
        """
        Input: pose [x, y] or [x, y, theta]
        Output: Node closest to given node
        """
        x, y = pose[0], pose[1]
        # return two nearest because first nearest will always be itself
        nearest = list(self.tree.nearest((x, y, x, y), 4))
        if len(nearest) == 1:
            nearest_neighbor = [self.nodes[nearest[0]]]
        else:
            nearest_neighbor = [self.nodes[x] for x in nearest[1:]] # get node item from node list
        return nearest_neighbor

    def in_goal(self, node):
        '''
        Input: Node object
        Output: Boolean representing if node is in goal region
        '''
        if self.goal_region["xmin"] < node.pose[0] < self.goal_region["xmax"] and self.goal_region["ymin"] < node.pose[1] < self.goal_region["ymax"]:
            return True

    def tree_insert(self, node):
        """
        Insert a node into the R-tree
        """
        x, y = node.pose[0], node.pose[1]
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
        Input: None
        Output: None
        Rewires graph by connecting the current node to the minimum cost path.
        """
        curr = self.current
        neighbor_idxs = self.find_neighbors(curr)

        # find best parent for the current node
        for n_idx in neighbor_idxs:
            n = self.nodes[n_idx]
            if curr.pose != n.pose:
                possible_path = self.create_path(curr, n.pose)
                if not self.in_collision(possible_path):
                    possible_cost = n.cost + self.get_cost(possible_path)
                    if possible_cost < curr.cost:
                        # better path found
                        curr.set_parent(n)
                        curr.set_path(possible_path)
                        curr.set_cost(possible_cost)

        # Check if existing paths can be improved by connecting through current node
        for n_idx in neighbor_idxs:
            n = self.nodes[n_idx]
            if curr.pose != n.pose:
                possible_path = self.create_path(curr, n.pose)
                if not self.in_collision(possible_path):
                    possible_cost = self.get_cost(possible_path) + curr.cost
                    if possible_cost < n.cost:
                        # set parent of neighbor to current node
                        n.set_parent(curr)
                        n.set_path(possible_path)
                        n.set_cost(possible_cost)

    def plan_node_path(self, node):
        """
        Input: Node object
        Output: list of Node objects from parent to child
        """
        if node.parent == None:
            return []
        return self.plan_node_path(node.parent) + [node]

    def plan_pose_path(self, node_path):
        '''
        Input: List of Node objects representing a path
        Output: List of poses from first Node in self.node_path to
                lst node in self.node_path
        '''

        # path = np.vstack([x.path for x in node_path[1:]])
        path = []
        for x in node_path:
            path += x.path
        return path

    def check_ancestors(self, node):
        """
        Input: Node object
        Output: None
        Checks if cost can be improved by setting the parent of the input node
        to one of its ancestors.
        """
        # TODO(abbie/alex): preserve points around corners
        parent = node.parent
        grandparent = parent.parent
        path = self.create_path(grandparent, node.pose)
        if not self.in_collision(path):
            cost = self.get_cost(path)
            if cost + grandparent.cost < node.cost and cost < 6 * self.neighbor_radius:
                # print("CONNECT WITH YOUR ROOTS")
                node.set_parent(grandparent)
                node.set_path(path)
                node.set_cost(cost+grandparent.cost)

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

    def create_PointCloud_pose(self, nodes, pub):
        '''
        Create and publish point cloud of particles and current pose marker
        '''
        self.cloud.header.frame_id = "/map"
        self.cloud.points = [Point32() for i in range(len(nodes))]
        for node in range(len(nodes)):
            self.cloud.points[node].x = nodes[node][0]
            self.cloud.points[node].y = nodes[node][1]
            self.cloud.points[node].z = 0
        pub.publish(self.cloud)

    def create_vistree(self):
        """
        Create and publish a Marker object representing the tree
        """
        # Setup header
        self.vis_tree.header.frame_id = "/map"
        self.vis_tree.header.stamp = rospy.Time.now()
        self.vis_tree.ns = "rrt_tree"
        self.vis_tree.id = 0
        self.vis_tree.type = 5 # line list

        # Set visualization params
        self.vis_tree.scale.x = 0.05
        self.vis_tree.color.b = 1.0
        self.vis_tree.color.a = 1.0

        self.vis_tree.points = []
        for node in self.nodes:
            self.vis_tree.points += node.line

        self.tree_pub.publish(self.vis_tree)

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


class Node:
    """
    RRT graph node
    """
    id = 0 # counter for nodes

    def __init__(self, pose, parent = None, path = None, cost = 0.0, start = False):
        self.pose = pose # [x, y, theta]
        self.path = path # series of poses from parent to self
        self.parent = parent
        self.cost = cost # distance to source along edges
        if start:
            Node.id = 0
        else:
            Node.id += 1
        self.id = Node.id # self.id = index in RRT.nodes in RRT class
        self.line = [Point32(), Point32()]
        self.create_marker()

    def set_parent(self, parent):
        # print "Resetting parent of Node", self.id
        self.parent = parent
        self.create_marker()

    def set_path(self, path):
        self.path = path

    def set_cost(self, cost):
        self.costs = cost

    def create_marker(self):
        """
        Creates a Marker message from self to parent. Regenerates everytime
        parent is reset.
        """
        if self.parent is not None:
            # Define points
            self.line[0].x = self.parent.pose[0]
            self.line[0].y = self.parent.pose[1]
            self.line[0].z = 0

            self.line[1].x = self.pose[0]
            self.line[1].y = self.pose[1]
            self.line[1].z = 0

if __name__ == "__main__":
    rospy.init_node("rrt")
    rrt = RRTstar()
    rospy.spin()
