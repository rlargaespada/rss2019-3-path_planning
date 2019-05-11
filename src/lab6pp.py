#!/usr/bin/env python2

"""
    RSS 2019 | Pure Pursuit controller for path tracking
    Author: Kyle Morgenstein
"""

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import MultiArrayLayout, Float32
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
import numpy as np
import warnings


warnings.simplefilter('ignore', np.RankWarning)

class PathPlanning:
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    PATH_TOPIC = "/test_path"
    DRIVE_TOPIC = rospy.get_param("/Trajectory_follower/drive_topic")
    VELOCITY = float(rospy.get_param("/Trajectory_follower/speed"))  # [m/s]
    #POSE_TOPIC = rospy.get_param("/particle_filter/pose_topic")
    local_topic = "/pf/viz/inferred_pose"
    float_topic = "/numbers"
    path_cloud_topic = "/path_cloud"
    #POSITION = None #rospy.get_param("~position") # (x,y), robot position in map frame



    def __init__(self):
        print("HELLO")
        self.pose_sub = rospy.Subscriber(self.local_topic,PoseStamped,self.pose_callback,queue_size=10)
        self.POSE = []
        self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
        self.pub_float = rospy.Publisher(self.float_topic, Float32, queue_size=10)
        self.path_cloud_pub = rospy.Publisher(self.path_cloud_topic, PointCloud, queue_size=10)
        self.path_sub = rospy.Subscriber(self.PATH_TOPIC, PointCloud, self.pp_callback, queue_size=10)
        self.cloud = PointCloud()
        self.list_pos = 0
        self.lookahead = 1
        self.base_link = .324
        self.position = np.zeros(2)

    def pp_callback(self, path_points):
        '''
        Pure pursuit on a list of close together points.  
        Will pursue the first point that falls outside the lookahead distance.
        Since points are close together this should translate into relatively smooth curve
        '''
        self.path = np.array([[point.x, point.y] for point in path_points.points])

    def dist(self, pos1, pos2):
        '''
        returns: distance between two points
        '''
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def get_target_point(self, path, pose, list_pos, l):
        '''
        Chooses the correct point for pure pursuit by iterating through points until the distance covered 
        is greater than the lookahead
        '''
        min_pt = []
        min_idx = []
        closest_point = None
        min_dist = float("inf")
        #Iterate from the last_pos to find the next point outside radius.  
        #We do this to avoid following points behind us.
        for idx in range(list_pos, list_pos + len(path)):
            pt = path[idx%(len(path) - 1)]
            distance = self.dist(pt, pose)
            if distance > l:
                closest_index = idx
                closest_point = pt
                break
        return closest_point, closest_index, distance

    def get_closest_point(self):
        """
        Returns index, coords, around distance of closest point in the path
        """
        pose = np.array(self.POSE[:2])
        poses = np.tile(pose, (self.path.shape[0], 1))
        # create an array same length as path of the x,y coords of th pose
        distances = self.path - poses
        dists = np.array(np.linalg.norm(distances, axis=1))
        closest_idx = np.argmin(dists)
        # find the index pf the point that is the smallest distance between current pose and the path
        return closest_idx      

    def pose_callback(self, data):
        '''
        Updates the current pose and uses pure pursuit to follow the path
        '''
        self.POSE = np.array([data.pose.position.x, data.pose.position.y, 2*np.arctan(data.pose.orientation.z/data.pose.orientation.w)]) #sets global position variable
        
        self.list_pos = self.get_closest_point() # index of the closest point
        
        # gets the target point for pure pursuit
        target_point, target_index, distance = self.get_target_point(self.path, self.POSE, self.list_pos, self.lookahead)
        
        curvature = self.get_curvature(target_point)
        self.set_lookahead(curvature)

        self.PointCloud_path([self.path[self.list_pos, :], target_point])
        self.path_cloud_pub.publish(self.cloud)

        x_new, y_new = target_point[0] - self.POSE[0], target_point[1] - self.POSE[1]

        eta = np.arctan2(y_new, x_new) - self.POSE[2] # angle between velocity vector and desired path [rad]
        u = np.arctan(2*self.base_link*np.sin(eta)/distance) # sets input steering angle from controller [rad]

        self.create_ackermann_message(u)        

    def create_ackermann_message(self, steering_angle):
        '''
        Creates an Ackermann drive message and publishes it
        '''
        A = AckermannDriveStamped()
        A.drive.speed = self.VELOCITY
        A.drive.steering_angle = steering_angle
        A.drive.steering_angle_velocity = 0 # determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
        self.pub.publish(A) 


    def PointCloud_path(self, points):
        self.cloud.header.frame_id = "/map"
        self.cloud.points = [Point32() for i in range(len(points))]
        for point in range(len(points)):
            self.cloud.points[point].x = points[point][0]
            self.cloud.points[point].y = points[point][1]
            self.cloud.points[point].z = 0

    def get_curvature(self, target_pt):
        """
        Input: target_pt: x, y coordinates of point that we are steering toward
        Output: radius of curvature: a metric used to determine the next lookahead distance
        """
        pose = self.path[self.list_pos]

        goal_angle = np.arctan2(pose[1]-target_pt[1], pose[0]-target_pt[0])-self.POSE[2]

        rad = self.lookahead/(2*np.sin(goal_angle))

        return abs(rad)

    def set_lookahead(self, curv):
        """
        chooses lookahead distance based on curvature of the path
        more curvature = shorter lookahead, less curvature = longer lookahead
        """
        if curv < 3:
            self.lookahead = 1
            self.VELOCITY = 2
        elif curv < 5:
            self.lookahead = 2
            self.VELOCITY = 2.5
        else:
            self.lookahead = 4
            self.VELOCITY = 3

    
if __name__ == "__main__":
    rospy.init_node("Trajectory_follower")
    path_planning = PathPlanning()
    rospy.spin()
