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
from geometry_msgs.msg import PolygonStamped, Point32
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
	PATH_TOPIC = rospy.get_param("/Trajectory_follower/path_topic")
	DRIVE_TOPIC = rospy.get_param("/Trajectory_follower/drive_topic")
	VELOCITY = float(rospy.get_param("/Trajectory_follower/velocity"))  # [m/s]
	#POSE_TOPIC = rospy.get_param("/particle_filter/pose_topic")
	local_topic = "/estim_pose"
	float_topic = "/numbers"
	path_cloud_topic = "/path_cloud"
	#POSITION = None #rospy.get_param("~position") # (x,y), robot position in map frame



	def __init__(self):
		self.pose_sub = rospy.Subscriber(self.local_topic,Point32,self.pose_callback,queue_size=10)
		self.POSE = []
		self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
		self.pub_float = rospy.Publisher(self.float_topic, Float32, queue_size=10)
		self.path_cloud_pub = rospy.Publisher(self.path_cloud_topic, PointCloud, queue_size=10)
		self.path_sub = rospy.Subscriber(self.PATH_TOPIC, PointCloud, self.pp_callback, queue_size=10)
		self.cloud = PointCloud()
		self.position = np.zeros(2)

	def pp_callback(self, path_points):
		'''
		Pure pursuit on a list of close together points.  
		Will pursue the first point that falls outside the lookahead distance.
		Since points are close together this should translate into relatively smooth curve
		'''
		path = [[point.x, point.y] for point in path_points.points]
		# path = [[0.05*x, 0] for x in range(100)]
		# path += [[5, 0.05*x] for x in range(100)]
		self.PointCloud_path(path)
		list_pos = 0	#The lat position in the path that came inside the radius
		L = .324
		l = 1	#Lookahead distance
		while True:
			if len(self.POSE) > 0:
				# compute ackermann steering angle to feed into cotroller
				# self.path_cloud_pub.publish(self.cloud)
				closest_point, closest_index, distance = self.get_closest_point(path, self.POSE, list_pos, l)
				print("closest_index: ", closest_index)
				print("around closest: ", path[closest_index - 2: closest_index + 2])
				# self.PointCloud_path([closest_point])
				self.path_cloud_pub.publish(self.cloud)
				list_pos = closest_index
				x_new, y_new = closest_point[0] - self.POSE[0], closest_point[1] - self.POSE[1]
				eta = np.arctan2(y_new, x_new) - self.POSE[2] #angle between velocity vector and desired path [rad]
				print("eta: ", eta)
				u = np.arctan(2*L*np.sin(eta)/distance) #sets input steering angle from controller [rad]
				A = AckermannDriveStamped()
				A.drive.speed = self.VELOCITY #self.VELOCITY #sets velocity [m/s]s
				A.drive.steering_angle = u #determines input steering control
				A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
				self.pub.publish(A) #publish steering command
				rospy.sleep(.02)

	def dist(self, pos1, pos2):
		return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5

	def get_closest_point(self, path, pose, list_pos, l):
		#Iterates from list_pos until we find a point that is outside our radius.
		min_pt = []
		min_idx = []
		min_dist = float("inf")
		#Iterate from the last_pos to find the next point outside radius.  
		#We do this to avoid following points behind us.
		for idx in range(list_pos, len(path)):
			pt = path[idx]
			distance = self.dist(pt, pose)
			if distance > l:
				closest_index = idx
				closest_point = pt
				break
		return closest_point, closest_index, distance
			

	def pose_callback(self, pose):
		self.POSE = [pose.x, pose.y, pose.z]

	def PointCloud_path(self, points):
		self.cloud.header.frame_id = "/map"
		self.cloud.points = [Point32() for i in range(len(points))]
		for point in range(len(points)):
			self.cloud.points[point].x = points[point][0]
			self.cloud.points[point].y = points[point][1]
			self.cloud.points[point].z = 0

	
if __name__ == "__main__":
	rospy.init_node("path_planning")
	path_planning = PathPlanning()
	# path_planning.pp_callback([5, 5])
	rospy.spin()
