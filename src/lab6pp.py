#!/usr/bin/env python2

"""
    RSS 2019 | Pure Pursuit controller for path tracking
    Author: Kyle Morgenstein
"""

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import MultiArrayLayout
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import Marker
import numpy as np
import warnings
from std_msgs import Float32
warnings.simplefilter('ignore', np.RankWarning)

class PathPlanning:
	# Access these variables in class functions with self:
	# i.e. self.CONSTANT
	PATH_TOPIC = rospy.get_param("~path_topic")
	DRIVE_TOPIC = rospy.get_param("~drive_topic")
	VELOCITY = float(rospy.get_param("~velocity"))  # [m/s]
	POSITION = rospy.get_param("~position") # (x,y), robot position in map frame



	def __init__(self):
		# TODO:
		# Initialize your publishers and
		# subscribers here
		self.sub = rospy.Subscriber(self.PATH_TOPIC, MultiArrayLayout, self.callback, queue_size=10)
		self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
		self.trajectory  = utils.LineTrajectory("/followed_trajectory")
		self.traj_sub = rospy.Subscriber(self.trajectory_topic, PolygonStamped, self.trajectory_callback, queue_size=1)


	# TODO:
	# Write your callback functions here.
	def trajectory_callback(self, msg):
		''' Clears the currently followed trajectory, and loads the new one from the message
		'''
		print "Receiving new trajectory:", len(msg.polygon.points), "points" 
		self.trajectory.clear()
		self.trajectory.fromPolygon(msg.polygon)
		self.trajectory.publish_viz(duration=0.0)

	def callback(self,data):
		'''
		main functionality of node
		inputs trajectory data and outputs control action u

		input: data: a MultiArray message containing:
						list of x,y pairs in map frame (n,2)
						n = # of waypoints

						[[x0,y0],[x1,y1],[x2,y2],...,[xn,yn]]

		ASSUMPTIONS:
		INPUT COORDS ARE IN MAP FRAME
		PATH IS IN ORDER FROM START POINT TO END GOAL

		output: u: AckermannDriveStamped Message containing:
						steering angle [rad]
						rate of steering control [rad/s]
						velocity [m/s]

		'''
		## data preprocessing
		data_vec = np.array(data) # (n,2)
		pos_map = np.array([[POSITION[0],POSITION[1]]]) # (1,2)

		d = data_vec-pos_map # (n,2), puts data in robot frame (robot is at (0,0)), splits data into x and y each of length n

		if d.size==0:
			#sets drive conditions if robot out of range
			A.drive.steering_angle = .436 #sets steering angle to turn counterclockwise [rad]
			self.pub.publish(A) #publish steering command
			return

		dists = np.sqrt(np.einsum('ij,ij->i',d,d)) #list of distances from car to waypoint
		i = np.argmin(dists) #index of closest waypoint
		path_remaining = d[i:,i:] #cuts off prior waypoints already passed
		dists_remaining = dists[i:]

		#combined proportional-pure persuit controller with Ackermann steering
		L = .324 #length of wheel base [m]
		
		x,y = path_remaining
		m,b = np.polyfit(x[0:10],y[0:10],1)
		#if path heading is within 15 degrees of straight ahead
		#
		#
		# THIS PARAM MUST BE TUNED!!!!
		# WANT CAR TO TURN IN TIME BUT BE ROBUST TO SLIGHT DEVIATIONS IN PATH
		# SHOULD TEST IN SIMULATION TO FIND OPTIMAL VALUE
		#
		if -.26<np.arctan(m)<.26:
			l = .5*self.VELOCITY #look ahead dist [m]
		else:
			l = .2*self.VELOCITY #look ahead dist [m]


		path_step=.5 #distance between waypoints
		#find point in path one lookahead distance out
		#this is the fewest possible computations
		for j in range(len(dists_remaining)):
			#check if path is more than one lookahead distance from robot
			if l<dists_remaining[j]:
				ind = j
				l = dists_remaining[j]
				break
			# find index of first waypoint in the line segment containing 
			# the point on the path one lookahead distance away from the robot	
			if 0<l-dists_remaining[j]<path_step:
				ind = j
				break
		else:
			path_remaining = d
			ind = 0
		#find slope of line segment containing the target point
		x1 = path_remaining[ind,0]
		y1 = path_remaining[ind,1]
		x2 = path_remaining[ind+1,0]
		y2 = path_remaining[ind+1,1]
		#edge cases
		flag = False
		if y2==y1:
			y_new = y1
			x_new = np.sqrt(l**2-y1**2)
			flag = True
		if x2==x1:
			x_new = x1
			y_new = np.sqrt(l**2-x1**2)
			flag = True
		#standard computation
		if not flag:
			m = (y2-y1)/(x2-x1) #slope
			#compute target coord
			inner = -(y1*m)**2+(l*m)**2+(l**2)*(m**4)+2*x1*y1*m**3-(m**4)*(x1**2)
			if inner<0:
				y_new=(y1+y2)/2
			else:
				y_new = (y1-m*x1+np.sqrt(inner))/(1+m**2)
			x_new = x1+(y_new-y1)/m

		# compute ackermann steering angle to feed into cotroller
		eta = np.arctan(y_new/x_new) #angle between velocity vector and desired path [rad]
		u = np.arctan(2*L*np.sin(eta)/l) #sets input steering angle from controller [rad]


		A = AckermannDriveStamped()
		A.drive.speed = self.VELOCITY #sets velocity [m/s]
		A.drive.steering_angle = u #determines input steering control
		A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
		self.pub.publish(A) #publish steering command


	
if __name__ == "__main__":
	rospy.init_node("path_planning")
	path_planning = PathPlanning()
	rospy.spin()