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
        self.path = []
        self.cloud = PointCloud()
        self.lookahead = 1	#Lookahead distance
        self.position = np.zeros(2)

    def pp_callback(self, path_points):
        '''
        Pure pursuit on a list of close together points.  
        Will pursue the first point that falls outside the lookahead distance.
        Since points are close together this should translate into relatively smooth curve
        '''
        print("HIII")
        self.path = [[point.x, point.y] for point in path_points.points]
        # path = [[0.05*x, 0] for x in range(100)]
        # path += [[5, 0.05*x] for x in range(100)]
        # path = [(0.1, -0.4), (0.0, -0.3), (-0.1, -0.2), (-0.2, -0.2), (-0.3, -0.2), (-0.4, -0.2), (-0.5, -0.2), (-0.6, -0.2), (-0.7, -0.2), (-0.8, -0.2), (-0.9, -0.2), (-1.0, -0.2), (-1.1, -0.2), (-1.2, -0.2), (-1.3, -0.2), (-1.4, -0.2), (-1.5, -0.2), (-1.6, -0.2), (-1.7, -0.2), (-1.8, -0.2), (-1.9, -0.2), (-2.0, -0.2), (-2.1, -0.2), (-2.2, -0.2), (-2.3, -0.2), (-2.4, -0.2), (-2.5, -0.2), (-2.6, -0.2), (-2.7, -0.2), (-2.8, -0.2), (-2.9, -0.2), (-3.0, -0.2), (-3.1, -0.2), (-3.2, -0.2), (-3.3, -0.2), (-3.4, -0.2), (-3.5, -0.2), (-3.6, -0.2), (-3.7, -0.2), (-3.8, -0.2), (-3.9, -0.2), (-4.0, -0.2), (-4.1, -0.2), (-4.2, -0.2), (-4.3, -0.2), (-4.4, -0.2), (-4.5, -0.2), (-4.6, -0.2), (-4.7, -0.2), (-4.8, -0.2), (-4.9, -0.2), (-5.0, -0.2), (-5.1, -0.2), (-5.2, -0.2), (-5.3, -0.2), (-5.4, -0.2), (-5.5, -0.2), (-5.6, -0.2), (-5.7, -0.2), (-5.8, -0.2), (-5.9, -0.2), (-6.0, -0.2), (-6.1, -0.2), (-6.2, -0.2), (-6.3, -0.2), (-6.4, -0.2), (-6.5, -0.2), (-6.6, -0.2), (-6.7, -0.2), (-6.8, -0.2), (-6.9, -0.2), (-7.0, -0.2), (-7.1, -0.2), (-7.2, -0.2), (-7.3, -0.2), (-7.4, -0.2), (-7.5, -0.2), (-7.6, -0.2), (-7.7, -0.2), (-7.8, -0.2), (-7.9, -0.2), (-8.0, -0.2), (-8.1, -0.2), (-8.2, -0.2), (-8.3, -0.2), (-8.4, -0.2), (-8.5, -0.2), (-8.6, -0.2), (-8.7, -0.2), (-8.8, -0.2), (-8.9, -0.2), (-9.0, -0.2), (-9.1, -0.2), (-9.2, -0.2), (-9.3, -0.2), (-9.4, -0.2), (-9.5, -0.2), (-9.6, -0.2), (-9.7, -0.2), (-9.8, -0.2), (-9.9, -0.2), (-10.0, -0.2), (-10.1, -0.2), (-10.2, -0.2), (-10.3, -0.2), (-10.4, -0.2), (-10.5, -0.2), (-10.6, -0.2), (-10.7, -0.2), (-10.8, -0.2), (-10.9, -0.2), (-11.0, -0.2), (-11.1, -0.2), (-11.2, -0.2), (-11.3, -0.2), (-11.4, -0.2), (-11.5, -0.2), (-11.6, -0.2), (-11.7, -0.2), (-11.8, -0.2), (-11.9, -0.2), (-12.0, -0.2), (-12.1, -0.2), (-12.2, -0.2), (-12.3, -0.2), (-12.4, -0.2), (-12.5, -0.2), (-12.6, -0.2), (-12.7, -0.2), (-12.8, -0.2), (-12.9, -0.2), (-13.0, -0.2), (-13.1, -0.2), (-13.2, -0.2), (-13.3, -0.2), (-13.4, -0.2), (-13.5, -0.2), (-13.6, -0.2), (-13.7, -0.2), (-13.8, -0.2), (-13.9, -0.2), (-14.0, -0.2), (-14.1, -0.2), (-14.2, -0.2), (-14.3, -0.2), (-14.4, -0.2), (-14.5, -0.2), (-14.6, -0.2), (-14.7, -0.2), (-14.8, -0.2), (-14.9, -0.2), (-15.0, -0.2), (-15.1, -0.2), (-15.2, -0.2), (-15.3, -0.2), (-15.4, -0.2), (-15.5, -0.2), (-15.6, -0.2), (-15.7, -0.2), (-15.8, -0.2), (-15.9, -0.2), (-16.0, -0.2), (-16.1, -0.1), (-16.2, -0.1), (-16.3, -0.0), (-16.4, 0.0), (-16.5, 0.1), (-16.6, 0.2), (-16.7, 0.2), (-16.8, 0.3), (-16.9, 0.3), (-17.0, 0.3), (-17.1, 0.3), (-17.2, 0.3), (-17.3, 0.3), (-17.4, 0.3), (-17.5, 0.4), (-17.6, 0.4), (-17.7, 0.4), (-17.8, 0.4), (-17.9, 0.4), (-18.0, 0.4), (-18.1, 0.4), (-18.2, 0.4), (-18.2, 0.5), (-18.2, 0.6), (-18.2, 0.7), (-18.2, 0.8), (-18.2, 0.9), (-18.2, 1.0), (-18.2, 1.1), (-18.2, 1.2), (-18.2, 1.3), (-18.2, 1.4), (-18.2, 1.5), (-18.2, 1.6), (-18.2, 1.7), (-18.2, 1.8), (-18.3, 1.9), (-18.3, 2.0), (-18.3, 2.1), (-18.3, 2.2), (-18.3, 2.3), (-18.3, 2.4), (-18.3, 2.5), (-18.3, 2.6), (-18.3, 2.7), (-18.3, 2.8), (-18.3, 2.9), (-18.3, 3.0), (-18.2, 3.1), (-18.2, 3.2), (-18.2, 3.3), (-18.2, 3.4), (-18.2, 3.5), (-18.2, 3.6), (-18.2, 3.7), (-18.2, 3.8), (-18.2, 3.9), (-18.2, 4.0), (-18.2, 4.1), (-18.2, 4.2), (-18.1, 4.3), (-18.1, 4.4), (-18.1, 4.5), (-18.1, 4.6), (-18.1, 4.7), (-18.1, 4.8), (-18.1, 4.9), (-18.1, 5.0), (-18.1, 5.1), (-18.1, 5.2), (-18.1, 5.3), (-18.1, 5.4), (-18.1, 5.5), (-18.1, 5.6), (-18.1, 5.7), (-18.1, 5.8), (-18.1, 5.9), (-18.1, 6.0), (-18.1, 6.1), (-18.1, 6.2), (-18.1, 6.3), (-18.1, 6.4), (-18.1, 6.5), (-18.1, 6.6), (-18.0, 6.7), (-18.0, 6.8), (-18.0, 6.9), (-18.0, 7.0), (-18.0, 7.1), (-18.0, 7.2), (-17.9, 7.3), (-17.8, 7.4), (-17.7, 7.5), (-17.6, 7.6), (-17.5, 7.7), (-17.5, 7.8), (-17.4, 7.9), (-17.3, 8.0), (-17.2, 8.1), (-17.1, 8.2), (-17.0, 8.3), (-16.9, 8.4), (-16.8, 8.5), (-16.7, 8.6), (-16.7, 8.7), (-16.6, 8.8), (-16.5, 8.9), (-16.4, 9.0), (-16.3, 9.1), (-16.2, 9.2), (-16.1, 9.3), (-16.0, 9.4), (-15.9, 9.5), (-15.8, 9.6), (-15.7, 9.7), (-15.6, 9.8), (-15.5, 9.9), (-15.4, 10.0), (-15.3, 10.1), (-15.2, 10.2), (-15.1, 10.3), (-15.0, 10.4), (-14.9, 10.5), (-14.8, 10.6), (-14.7, 10.7), (-14.7, 10.8), (-14.7, 10.9), (-14.6, 11.0), (-14.6, 11.1), (-14.5, 11.2), (-14.5, 11.3), (-14.4, 11.4), (-14.4, 11.5)]
        self.PointCloud_path(self.path)
        list_pos = 0	#The lat position in the path that came inside the radius
        L = .324
        while True:
            if len(self.POSE) > 0:
                # compute ackermann steering angle to feed into cotroller
                # self.path_cloud_pub.publish(self.cloud)
                closest_index = self.get_closest_point()
                target_point = self.get_target_point(closest_index)
                print(target_point)
                distance = self.dist(target_point, self.POSE)
                self.path_cloud_pub.publish(self.cloud)
                x_new, y_new = target_point[0] - self.POSE[0], target_point[1] - self.POSE[1]
                eta = np.arctan2(y_new, x_new) - self.POSE[2] #angle between velocity vector and desired path [rad]
                # print("eta: ", eta)
                u = np.arctan(2*L*np.sin(eta)/distance) #sets input steering angle from controller [rad]
                A = AckermannDriveStamped()
                A.drive.speed = self.VELOCITY #self.VELOCITY #sets velocity [m/s]s
                A.drive.steering_angle = u #determines input steering control
                A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
                self.pub.publish(A) #publish steering command
                rospy.sleep(.02)

    def get_closest_point(self):
        """
        Returns index, coords, around distance of closest point in the path
        """
        pose = np.array(self.POSE[:1])
        dists = np.array([np.linalg.norm(pt-pose) for pt in self.path])
        closest_idx = np.argmin(dists)
        return closest_idx

    def dist(self, pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def get_target_point(self, closest_point_idx):
        """
        Input: path: path segment on which to search for a target point
        Output: intersect: bool representing an intersection between the line
                           seg of the path and the circle of radius lookahead
                           distance centered at the car.
                target: point on the path that is lookahead away from the car.
                        None if no intersection
        """
        pose = np.array(self.POSE)[:2]
        # this defintion should be dependent on lookahead dist
        end = closest_point_idx + 20
        p1 = np.array(self.path[closest_point_idx]) # only want points in front of car
        p2 = np.array(self.path[end])
        v = p2-p1

        a = v.dot(v)
        b = 2 * v.dot(p1-pose)
        c = p1.dot(p1) + pose.dot(pose) - 2 * p1.dot(pose) - self.lookahead**2

        # calculate discriminant
        disc = b**2 - 4 * a * c
        if disc < 0:
            # circle does not intersect the path, increase lookahead
            self.lookahead += 1
            return None

        sqrt_disc = disc**0.5
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        if 0 <= t1 <= 1:
            # try t1 first because it's further ahead
            return p1 + t1*v

        if 0 <= t2 <=1 and closest_point_idx > len(self.path) - 20:
            return p1 + v

        t = max(0, min(1, - b / (2 * a)))

        return p1 + t*v
            

    def pose_callback(self, data):
        self.POSE = np.array([data.pose.position.x, data.pose.position.y, 2*np.arctan(data.pose.orientation.z/data.pose.orientation.w)]) #sets global position variable

    def PointCloud_path(self, points):
        self.cloud.header.frame_id = "/map"
        self.cloud.points = [Point32() for i in range(len(points))]
        for point in range(len(points)):
            self.cloud.points[point].x = points[point][0]
            self.cloud.points[point].y = points[point][1]
            self.cloud.points[point].z = 0

    
if __name__ == "__main__":
    rospy.init_node("Trajectory_follower")
    path_planning = PathPlanning()
    # path_planning.pp_callback([5, 5])
    rospy.spin()
