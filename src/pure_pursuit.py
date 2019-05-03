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
        self.lookahead        = 4 #rospy.get_param("~lookahead")
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
        self.target_pub = rospy.Publisher("/target_pt", Point32, queue_size=10)

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

    def get_closest_point(self):
        """
        Returns index, coords, around distance of closest point in the path
        """
        pose = np.array(self.pose[:1])
        dists = np.array([np.linalg.norm(pt-pose) for pt in self.path])
        closest_idx = np.argmin(dists)

        return closest_idx, self.path[closest_idx], dists[closest_idx]

    def get_curvature(self, target_pt):
        """
        Input: target_pt: x, y coordinates of point that we are steering toward
        Output: curvature: a metric used to determine the next lookahead distance
        """
        pose = np.array(self.pose)[:2]
        target = np.array(target_pt)

        x_off = np.abs(2 * pose[0]-target[0])
        y_off = np.abs(2 * pose[1]-target[1])

        curv = x_off/self.lookahead**2

        return curv

    def set_lookahead(self, curv):
        """
        """
        if curv > 6:
            self.lookahead = 1
        elif 2 < curv < 6:
            self.lookahead = 2
        else:
            self.lookahead = 4

    def get_target_point(self, closest_point_idx):
        """
        Input: path: path segment on which to search for a target point
        Output: intersect: bool representing an intersection between the line
                           seg of the path and the circle of radius lookahead
                           distance centered at the car.
                target: point on the path that is lookahead away from the car.
                        None if no intersection
        """
        pose = np.array(self.pose)[:2]
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

        if 0 <= t2 <=1 and closest_point_idx > len(self.path-20):
            return p1 + v

        t = max(0, min(1, - b / (2 * a)))

        return p1 + t*v

        def dist(self, pos1, pos2):
		return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def pose_cb(self, pose):
        self.pose = [pose.x, pose.y, pose.z]

        if self.received_path:
            idx, pt, dist = self.get_closest_point()
            self.path_idx = idx
            target = self.get_target_point(idx)

            if target is not None:
                self.publish_point(target)
                # set next lookahead distance
                curv = self.get_curvature(target)
                print "Curv:", curv
                self.set_lookahead(curv)
                print "Lookahead:", self.lookahead
                # print "Target:", target
                steer = self.get_steering_ang(target)

                self.pub_drive(steer)

    def publish_point(self, point):
        pt_msg = Point32()
        pt_msg.x, pt_msg.y = point[0], point[1]
        self.target_pub.publish(pt_msg)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
