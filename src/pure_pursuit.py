#!/usr/bin/env python
"""
    RSS 2019 | pure_pursuit.py
    Pure Pursuit Controller using Ackermann steering and path curvature
    dependent speed control.

    Author: Abbie Lee (abbielee@mit.edu)
"""

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import MultiArrayLayout, Float32
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
import numpy as np
import warnings

class PurePursuit(object):
    """
    Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.trajectory_topic = rospy.get_param("~trajectory_topic")
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = rospy.get_param("~lookahead")
        self.speed            = float(rospy.get_param("~velocity"))
        self.wrap             = bool(rospy.get_param("~wrap"))
        self.wheelbase        = float(rospy.get_param("~wheelbase"))
        self.drive_topic      = rospy.get_param("~drive_topic")
        self.pose_topic       = "/estim_pose"
        self.path_topic       = "/test_path"

        # Init variables
        self.pose = []
        self.path = []
        self.received_path = False
        self.path_idx = 0 # How much progress have we made??

        # Message
        self.ack_msg = AckermannDriveStamped()

        # subs and pubs
        self.path_sub = rospy.Subscriber(self.path_topic, PointCloud, self.path_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Point32, self.pose_cb, queue_size=10)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)

    def trajectory_callback(self, msg):
        """
        Clears the currently followed trajectory, and loads the new one from the message
        """
        print "Receiving new trajectory:", len(msg.polygon.points), "points"
        self.trajectory.clear()
        self.trajectory.fromPolygon(msg.polygon)
        self.trajectory.publish_viz(duration=0.0)

    def path_cb(self, path_points):
        if not self.received_path:
            print "Receiving path:", len(path_points.points), "points."
            self.received_path = True
            path = [[point.x, point.y] for point in path_points.points]
            if self.wrap:
                path = path + path
            self.path = np.array(path)

    def get_steering_ang(self, target):
        """
        Input: target: point in robot coordinate frame to steer towards
        Output: steer: Ackermann steering angle from current point
        """
        # eta  = angle between velocity vector and desired point [rad]
        eta = np.arctan2(target[1], target[0])-self.pose[2]

        # computes steering angle using ackermann steering [rad]
        steer = np.arctan(2*self.wheelbase*np.sin(eta)/self.lookahead)

        return steer

    def pub_drive(self, angle):
        self.ack_msg.drive.speed = self.speed #sets velocity [m/s]s
        self.ack_msg.drive.steering_angle = angle #determines input steering control
        self.ack_msg.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
        self.drive_pub.publish(self.ack_msg) # publish steering command

    # def get_closest_point(self, path, pose, list_pos, l):
    #     #Iterates from list_pos until we find a point that is outside our radius.
    #     min_pt = []
    #     min_idx = []
    #     min_dist = float("inf")
    #     # Iterate from the last_pos to find the next point outside radius.
    #     # We do this to avoid following points behind us.
    #     for idx in range(list_pos, len(path)):
    #         pt = path[idx]
    #         distance = self.dist(pt, pose)
    #         if distance > self.lookahead:
    #             closest_index = idx
    #             closest_point = pt
    #             break
    #     return closest_point, closest_index, distance

    def get_closest_point(self):
        """
        Returns index and distance of closest point in the path
        """
        pose = np.array(self.pose[:1])
        dists = np.array([np.linalg.norm(pt-pose) for pt in self.path])
        closest_idx = np.argmin(dists)

        return closest_idx, self.path[closest_idx], dists[closest_idx]

    def get_target_point(self, path):
        """
        Input: path: path segment on which to search for a target point
        Output: target: point on the path that is lookahead away from the car
        """
        pose = np.array(self.pose[:1])
        relpath = path - pose
        dists = np.array([np.linalg.norm(pt-pose)-self.lookahead for pt in path])
        non_neg_dists = dists[dists>=0]
        new_points = relpath[dists>=0]

        return new_points[np.argmin(non_neg_dists)]


    # def find_lookahead_pt(self, i_seg):
    #     """
    #         args:
    #             pt0: (x,y) position of the robot
    #             i_seg: the index of the first point of the segment
    #         returns: the intersection of the segment and a circle around the robot
    #             with lookahead radius or None if no intersection
    #     """
    #     q = np.array(self.pose[:1])                        # Center of circle
    #     r = self.lookahead                       # Radius of circle
    #     p1 = self.path[i_seg:]           # Start of line segment
    #     v = self.path[i_seg+1:] - p1     # Vector along line segment
    #
    #     # Intersection at |p1 + t*v - q| = r
    #     a = np.dot(v, v)
    #     b = 2 * np.dot(v, p1 - q)
    #     c = np.dot(p1, p1) + np.dot(q, q) - 2*np.dot(p1, q) - r**2
    #     disc = b**2 - 4*a*c
    #     sqrt_disc = np.sqrt(disc)
    #
    #     if disc < 0:
    #         return None
    #
    #     t1 = (0 - b + sqrt_disc) / (2. * a)
    #     t2 = (0 - b - sqrt_disc) / (2. * a)
    #
    #     # Try t1 since it should be closer to the end of the segment
    #     if 0 <= t1 <= 1:
    #         return p1 + t1 * v
    #
    #     # If t1 is out of range and we're on the last segment and the intersection is behind the car
    #     if 0 <= t2 <= 1 and i_seg == len(self.path) - 2:
    #         return p1 + v
    #
    #     # If there is no intersection with the lookahead circle, the line segment might be too short
    #     # If it's the last segment, make sure to go to the end
    #     x0, y0 = self.pose[0], self.pose[1]
    #     x1, y1 = self.path[i_seg:]
    #     x2, y2 = self.path[i_seg+1:]
    #     if i_seg == len(self.path) - 2 and self.find_closest_pt()[2] < self.lookahead:
    #         return p1 + v
    #
    #     # Both are out of range
    #     return None


    def dist(self, pos1, pos2):
		return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def pose_cb(self, pose):
        self.pose = [pose.x, pose.y, pose.z]

        if self.received_path:
            idx, pt, dist = self.get_closest_point()
            self.path_idx = idx
            target = self.get_target_point(self.path[self.path_idx:])
            print "Target:", target

            steer = self.get_steering_ang(target)

            self.pub_drive(steer)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
