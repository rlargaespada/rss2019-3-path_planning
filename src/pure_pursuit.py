#!/usr/bin/env python
"""
    RSS 2019 | pure_pursuit.py
    Pure Pursuit Controller using Ackermann steering and path curvature
    dependent speed control.
    Author: Abbie Lee (abbielee@mit.edu) and Kayla Holman (kholman@mit.edu)
"""

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import MultiArrayLayout, Float32
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion
import numpy as np
import warnings
import math

class PurePursuit(object):
    """
    Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.trajectory_topic = rospy.get_param("~trajectory_topic")
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 1 # rospy.get_param("~lookahead")
        self.speed            = float(rospy.get_param("~speed"))
        self.wrap             = bool(rospy.get_param("~wrap"))
        self.wheelbase        = float(rospy.get_param("~wheelbase"))
        self.drive_topic      = rospy.get_param("~drive_topic")
        self.pose_topic       = "/pf/viz/inferred_pose"
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
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=10)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)
        self.target_pub = rospy.Publisher("/target_pt", Point32, queue_size=10)

        self.target_viz_pub = rospy.Publisher("/target_viz", Marker, queue_size=10)
        self.marker_pub = rospy.Publisher("line_boi", Marker, queue_size=10)

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
        eta = np.arctan2(target[1]-self.pose[1], target[0]-self.pose[0]) - self.pose[2]

        # computes steering angle using ackermann steering [rad]
        steer = np.arctan(2*self.wheelbase*np.sin(eta)/self.lookahead)

        return steer

    def pub_drive(self, angle):
        self.ack_msg.drive.speed = self.speed # sets velocity [m/s]
        self.ack_msg.drive.steering_angle = angle # determines input steering control
        self.ack_msg.drive.steering_angle_velocity = 0 # determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
        self.drive_pub.publish(self.ack_msg) # publish steering command

    def get_closest_point(self, point):
        """
        Returns index, coords, around distance of closest point in the path
        """
        pose_array = np.tile(point, (self.path.shape[0], 1))
        dists = np.linalg.norm(self.path-pose_array, axis=1)
        closest_idx = np.argmin(dists)
        return closest_idx, self.path[closest_idx], dists[closest_idx]

    def get_curvature(self, target_pt):
        """
        Input: target_pt: x, y coordinates of point that we are steering toward
        Output: curvature: a metric used to determine the next lookahead distance
        """
        # pose = np.array([0,0,0])
        pose = self.pose

        goal_angle = math.atan2(pose[1]-target_pt[1], pose[0]-target_pt[0])-pose[2]

        rad = self.lookahead/(2*math.sin(goal_angle))

        return abs(rad)

    def set_lookahead(self, curv):
        """
        chooses lookahead distance based on curvature of the path
        """
        if curv < 3:
            self.lookahead = 1
        elif curv < 9:
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
        # this defintion should be dependent on lookahead dist
        end = min(closest_point_idx + 10*self.lookahead, len(self.path)-1) # pts about 10 cm apart, this gets us to distance of lookahead
        # print(self.path[end])
        # return self.path[end]

        forward_points = np.array(self.path[closest_point_idx:end, :])
        # a,b = np.polyfit(forward_points[:,0]-self.pose[0], forward_points[:,1]-self.pose[1], 1)

        # forward_points[:,0]-=self.pose[0]
        # forward_points[:,1]-=self.pose[1]

        # transform = np.array([[np.cos(self.pose[2]), -np.sin(self.pose[2])], [np.sin(self.pose[2]), np.cos(self.pose[2])]])

        # pts = np.matmul(transform, forward_points.T)
        # points = pts.T

        # points[:,0]-=self.pose[0]
        # points[:,1]-=self.pose[1]

        a,b = np.polyfit(forward_points[:,0], forward_points[:,1], 1)
        # self.make_line(a,b)

        center = self.pose[:2]
        cx = center[0]
        cy = center[1]
        poss_xs = np.roots([a**2+1, 2*a*b - 2*cx - 2*a*cy, b**2+cx**2+cy**2-2*cy*b-self.lookahead**2])
        poss_ys = a*poss_xs+b

        if np.iscomplexobj([poss_xs[0], poss_ys[0]]) and np.iscomplexobj([poss_xs[1], poss_ys[1]]):
            print "no intersection between path and lookahead"
            return

        ref = [0,0]
        ind_p1, cp1, dist1 = self.get_closest_point([poss_xs[0], poss_ys[0]])
        ind_p2, cp2, dist2 = self.get_closest_point([poss_xs[1], poss_ys[1]])
        # print(poss_xs, poss_xs)
        if ind_p1>ind_p2:
            ref[0]=poss_xs[0]
            ref[1]=poss_ys[0]
        else:
            ref[0]=poss_xs[1]
            ref[1]=poss_ys[1]

        return np.array(ref)


        # p1 = np.array(self.path[closest_point_idx]) # only want points in front of car
        # p2 = np.array(self.path[end])

        # v = p2-p1

        # a = v.dot(v)
        # b = 2 * v.dot(p1-pose)
        # c = p1.dot(p1) + pose.dot(pose) - 2 * p1.dot(pose) - self.lookahead**2

        # # calculate discriminant
        # disc = b**2 - 4 * a * c
        # if disc < 0:
        #     # circle does not intersect the path, increase lookahead
        #     self.lookahead += 1
        #     return None

        # sqrt_disc = disc**0.5
        # t1 = (-b + sqrt_disc) / (2 * a)
        # t2 = (-b - sqrt_disc) / (2 * a)

        # if 0 <= t2 <= 1:
        #     # try t1 first because it's further ahead
        #     return p1 + t2*v

        # if 0 <= t1 <=1: #and closest_point_idx > len(self.path):
        #     return p1 + v

        # t = max(0, min(1, - b / (2 * a)))

        # return p1 + t*v

    def dist(self, pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

    def pose_cb(self, msg):
        # angle = 2*np.arctan(msg.pose.orientation.z/msg.pose.orientation.w)
        roll, pitch, angle = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.pose = [msg.pose.position.x, msg.pose.position.y, angle]

        if self.received_path:
            idx, pt, dist = self.get_closest_point(self.pose[:2])
            self.viz_point(pt)
            self.path_idx = idx
            # self.visualize_circle(self.lookahead)
            target = self.get_target_point(idx)
            if target is not None:
                # self.publish_point(target)
                # self.viz_point(target)
                # set next lookahead distance
                # curv = self.get_curvature(target)
                # print "Curv:", curv
                
                # print "Target:", target
                steer = self.get_steering_ang(target)

                # self.set_lookahead(curv)
                # print "Lookahead:", self.lookahead

                self.pub_drive(steer)

    def publish_point(self, point):
        pt_msg = Point32()
        pt_msg.x, pt_msg.y = point[0], point[1]
        self.target_pub.publish(pt_msg)

    def visualize_circle(self, r):
        line = Marker()
        line.header.frame_id = "base_link"
        line.header.stamp = rospy.get_rostime()
        line.ns = "circle"
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.1
        line.color.b = 1.0
        line.color.a = 1.0
        for i in range(0, 360):
            p = Point()
            p.x = r * math.cos(math.radians(i))
            p.y = r * math.sin(math.radians(i))
            line.points.append(p)
        self.marker_pub.publish(line)

    def make_line(self, m, b):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "line"
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.scale.x = 0.1

        for i in range(-9, 10):
            pt = Point()
            pt.x = i
            pt.y = m*i+b
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    def viz_point(self, point):
        #generates marker message
        m = Marker()
        m.header.frame_id="map"
        m.action=0
        m.id = 1
        m.scale.x=1
        m.scale.y=1
        m.scale.z=1
        m.pose.position.x = point[0]
        m.pose.position.y = point[1]
        m.pose.orientation.w=1
        m.type=Marker.SPHERE
        m.color.g=1
        m.color.a=1
        self.target_viz_pub.publish(m)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
