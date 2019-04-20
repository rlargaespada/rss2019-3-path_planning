#!/usr/bin/env python2
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
# import dubins
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
# import scipy.misc
import matplotlib.pyplot as plt
import graph
import search
try:
   import cPickle as pickle
except:
   import pickle

class AStar:
    """
    A* path planning from ROS map
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
        self.origin_x_offset = rospy.get_param("~origin_x_offset")
        self.origin_y_offset = rospy.get_param("~origin_y_offset")

        #Initialize visualization variables
        self.buff_factor = rospy.get_param("~buff_map")
        self.cloud = PointCloud()

        self.raw_path = []
        self.path = []
        self.pos_path = []

        self.map_loaded = False
        self.map_graph = None
        self.map_file = 'map_file'

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

    def real_world_to_occ(coord, resolution, origin):
        x = int((coord[0]-origin[0])/resolution)
        y = int((coord[1]-origin[1])/resolution)
        return (x,y)

    def occ_to_real_world(coord, resolution, origin):
        x = coord[0]/resolution + origin[0]
        y = coord[1]/resolution + origin[1]
        return (x,y)

    def set_start(self, start_pose):
        """
        Gets starting pose from rviz pose estimate marker.
        """
        x, y = start_pose.pose.pose.position.x, start_pose.pose.pose.position.y
        theta = 2*np.arctan(start_pose.pose.pose.orientation.z/start_pose.pose.pose.orientation.w)

        self.start_pose = [x, y, theta]

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
            continue
        self.start_pose = self.real_world_to_occ(self.start_pose, map_msg.info.resolution, map_msg.info.origin)
        self.goal_pose = self.real_world_to_occ(self.goal_pose, map_msg.info.resolution, map_msg.info.origin)

        # print "Loading map:", rospy.get_param("~map"), "..."
        # print "Start and Goal intialized:"
        # print "Start: ", self.start_pose
        # print "Goal: ", self.goal_pose
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)
        map_ = np.clip(map_, 0, 1)
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

        self.map_graph = self.check_if_graph()
        if self.map_graph == None:
            #map is an array of zeros and ones, convert into graph
            self.map_graph = graph.Graph(self.start_pose[:2], self.goal_pose[:2])
            self.map_graph.build_map(self.map)
            self.save_graph()
        
        #convert map so that large empty cells are consolidated

        self.map_loaded = True
        print("yay, we built the graph!")

        self.path = search.a_star(self.map_graph, self.start_pose[:2], self.goal_pose[:2])
        print(self.path)
        #self.path = self.smooth_path()
        self.create_PointCloud()
        self.pos_path = self.create_pose_path()
        self.draw_path()

    def save_graph(self):
        with open(self.map_file, 'wb') as f:
            pickle.dump(self.map_graph,f)

    def check_if_graph(self):
        try:
            with open(self.map_file) as f:
                graph = pickle.load(f)
            return graph

        except:
            return None

    # def smooth_path(self):
    #     path = []
    #     return path

    def create_pose_path(self):
        pose_path = []
        for p in range(len(self.path-1)):
            pose = self.steer(self.path[p], self.path[p-1])
            pose_path.append(pose)
            
        return pose_path

    def steer(self, start_node, next_pose):
        """
        Input: Parent node and proposed next pose [x, y]
        Output: the actual next pose [x, y, theta] (theta in direction of movement)
        """
        x = start_node.pose[0] + (next_pose[0] - start_node.pose[0])
        y = start_node.pose[1] + (next_pose[1] - start_node.pose[1])
        theta = np.arctan2(y, x)
        return [x, y, theta]

    def create_PointCloud(self):
        '''
        Create and publish point cloud of particles and current pose marker
        '''
        self.cloud.header.frame_id = "/map"
        self.cloud.points = [Point32() for i in range(len(self.path))]
        for p in range(len(self.path)):
            self.cloud.points[p].x = self.path[p][0]
            self.cloud.points[p].y = self.path[p][1]
            self.cloud.points[p].z = 0
        self.particle_cloud_publisher.publish(self.cloud)

    def draw_path(self):
        header = Header()
        path = Path()
        header.stamp = rospy.rostime.Time.now()
        header.frame_id = "/map"
        pose_stamp = PoseStamped()
        pose_stamp.header = header
        for pos in self.pos_path:
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

if __name__ == "__main__":
    rospy.init_node("astar")
    astar = AStar()
    rospy.spin()