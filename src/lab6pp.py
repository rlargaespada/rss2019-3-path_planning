#!/usr/bin/env python2

"""
    RSS 2019 | Pure Pursuit controller for path tracking
    Author: Kyle Morgenstein
"""
from __future__ import division
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import MultiArrayLayout, Float32
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
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
	#POSITION = None #rospy.get_param("~position") # (x,y), robot position in map frame



	def __init__(self):
		# TODO:
		# Initialize your publishers and
		# subscribers here
		#self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.set_start)
		self.pose_sub = rospy.Subscriber(self.local_topic,Point32,self.pose_callback,queue_size=10)
		self.POSE = []
		self.sub = rospy.Subscriber(self.PATH_TOPIC, Path, self.callback, queue_size=10)
		#self.pos_sub = rospy.Subscriber(self.POSE_TOPIC, Marker, self.pose_callback, queue_size=10)
		self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
		self.pub_line = rospy.Publisher("marker",Marker,queue_size=10)
		self.pub_dot = rospy.Publisher("marker",Marker,queue_size=10)
		#self.trajectory  = utils.LineTrajectory("/followed_trajectory")
		#self.traj_sub = rospy.Subscriber(self.trajectory_topic, PolygonStamped, self.trajectory_callback, queue_size=1)
		self.position = np.zeros(2)
		self.path = 0

	# TODO:
	# Write your callback functions here.
	#def trajectory_callback(self, msg):
		''' Clears the currently followed trajectory, and loads the new one from the message
		'''
	#	print "Receiving new trajectory:", len(msg.polygon.points), "points" 
	#	self.trajectory.clear()
	#	self.trajectory.fromPolygon(msg.polygon)
	#	self.trajectory.publish_viz(duration=0.0)

	def pose_callback(self,data):
		self.position = np.array([data.x,data.y,data.z])
		pos_map = self.position[0:2]
		data_vec = self.path
		if self.path==0:
			pass
		d = data_vec-pos_map # (n,2), puts data in robot frame (robot is at (0,0)), splits data into x and y each of length n
		
		#if d.size==(1,2):
		#	#sets drive conditions if robot out of range
		#	A.drive.steering_angle = .436 #sets steering angle to turn counterclockwise [rad]
		#	self.pub.publish(A) #publish steering command
		#	return
		#dists1 = np.sqrt(d[:,0]**2+d[:,1]**2) next line does the same thing faster
		dists = np.sqrt(np.einsum('ij,ij->i',d,d)) #list of distances from car to waypoint
		#print(dists1==dists)
		i = np.argmin(dists) #index of closest waypoint
		print(i)
		try:
			path_remaining = d[i:i+10,:] #cuts off prior waypoints already passed
			dists_remaining = dists[i:i+10]
		except: #warps path to cyclic if nearing the end of the path
			path_remaining = np.concatenate((d[i:,:],d[:10,:]))
			dists_remaining = np.concatenate((dists[i:,:],dists[:10]))
			print("wtf")

		#combined proportional-pure persuit controller with Ackermann steering
		L = .324 #length of wheel base [m]
		
		
		print(path_remaining)
		print(path_remaining.shape)
			

		#x,y = path_remaining.T
		#m,b = np.polyfit(x[0:5],y[0:5],1)
		#rospy.loginfo("m, %s",m)
		#rospy.loginfo("b, %s",b)
		#if path heading is within 15 degrees of straight ahead
		#
		#
		# THIS PARAM MUST BE TUNED!!!!
		# WANT CAR TO TURN IN TIME BUT BE ROBUST TO SLIGHT DEVIATIONS IN PATH
		# SHOULD TEST IN SIMULATION TO FIND OPTIMAL VALUE
		#



		v = self.VELOCITY
		if v<1:
			l=1
		elif 1<v<5:
			l=v/5+1
		else:
			l=v/2


		#need speed controller for corners if want to attempt high speed manuvers



		#if -.26<np.arctan(m)<.26:
		#	l = .5*self.VELOCITY #look ahead dist [m]
		#else:
		#	l = .2*self.VELOCITY #look ahead dist [m]


		#l = 1.5




		#path_step=.5 #distance between waypoints
		#find point in path one lookahead distance out
		#this is the fewest possible computations
		for j in range(len(dists_remaining)):
			# find index of first waypoint in the line segment containing 
			# the point on the path one lookahead distance away from the robot
			try:
				p2 = path_remaining[j+1,:]
			except:
				ind = j
				break
			p1 = path_remaining[j,:]
			path_step = np.linalg.norm(p2-p1)
			if 0<l-dists_remaining[j]<path_step:
				ind = j
				print(j)
				print('yay')
				break
			#check if path is more than one lookahead distance from robot
			if l<dists_remaining[j]:
				ind = j
				l = dists_remaining[j]
				print(j)
				print('its only this j')

				break

		else:
			ind = 0
			print('this prob should not happen often')
		flag = False
		#find slope of line segment containing the target point
		try: 
			
			x1 = path_remaining[ind,0]
			y1 = path_remaining[ind,1]
			x2 = path_remaining[ind+1,0]
			y2 = path_remaining[ind+1,1]
			if y2==y1:
				y_new = y1
				x_new = np.sqrt(l**2-y1**2)
				flag = True
			if x2==x1:
				x_new = x1
				y_new = np.sqrt(l**2-x1**2)
				flag = True
		except:
			
			x_new = path_remaining[ind,0]
			y_new = path_remaining[ind,1]
			flag = True
		#edge cases
		

		#standard computation
		if not flag:
			
			p1 = np.array([[x1,y1]])
			print('p1',p1)
			p2 = np.array([[x2,y2]])
			print('p2',p2)
			v = p2-p1

			a = np.dot(v,v.T)
			#print('a',a)
			b = 2*(np.dot(v,p1.T))
			#print('b',b)
			c = np.dot(p1,p1.T)-l**2
			#print('c',c)
			t1 = min(1,max(0,-b-np.sqrt(b**2-4*a*c)/(2*a)))
			t2 = min(1,max(0,-b+np.sqrt(b**2-4*a*c)/(2*a)))
			t = max(t1,t2)
			if 0<t<1:
				print('t is good')
			new_point = p1 + t*v
			x_new,y_new = new_point.T

		print('x,y',x_new,y_new)
		print('pose',pos_map)
		#marks = self.mark2(np.array([[x_new],[y_new]])) #generate marker message
		#self.pub_dot.publish(marks) #publish marker

			#m = (y2-y1)/(x2-x1) #slope
			#compute target coord
			#inner = -(y1*m)**2+(l*m)**2+(l**2)*(m**4)+2*x1*y1*m**3-(m**4)*(x1**2)
			#if inner<0:
			#	y_new=(y1+y2)/2
			#else:
			#	y_new = (y1-m*x1+np.sqrt(inner))/(1+m**2)
			#x_new = x1+(y_new-y1)/m

		# compute ackermann steering angle to feed into cotroller
		eta = np.arctan2(y_new,x_new)-self.position[2] #angle between velocity vector and desired path [rad]
		u = np.arctan(2*L*np.sin(eta)/l) #sets input steering angle from controller [rad]


		A = AckermannDriveStamped()
		A.drive.speed = self.VELOCITY #sets velocity [m/s]
		A.drive.steering_angle = u #determines input steering control
		A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
		self.pub.publish(A) #publish steering command

	def make_marker(self,d):
		#generates marker message
		m = Marker()
		m.header.frame_id="map"
		m.action=0
		m.id = 1
		m.pose.orientation.w=1
		m.type=Marker.LINE_LIST
		m.scale.x=.1
		m.color.g=1
		m.color.a=1
		l = []
		for i in d.tolist():
			p = Point()
			p.x=i[0]
			p.y=i[1]
			p.z=0
			l.append(p)
		m.points=l

		return m

	#def mark2(self,d):
	#	m = Marker()
	#	m.header.frame_id="map"
	#	m.action=0
	#	m.id = 2
	#	m.pose.orientation.w=1
	#	m.pose.position = d[0]
	#	m.pose.position = d[1]
	#	m.type=Marker.CYLINDER
	#	m.scale.x=2
	#	m.scale.y=2
	#	m.scale.z=2
	#	m.color.g=1
	#	m.color.a=1
	#	return m

	#def callback(self,path_info):
	def callback(self,d):
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
		def conv():
			d = [{"y": -0.8895988464355469, "x": 0.4171772003173828}, {"y": -0.6701974868774414, "x": -30.42536163330078}, {"y": -0.3597745895385742, "x": -32.4092903137207}, {"y": 0.8645005226135254, "x": -33.037513732910156}, {"y": 30.7249698638916, "x": -33.17718505859375}, {"y": 32.2868766784668, "x": -33.165679931640625}, {"y": 33.415687561035156, "x": -33.283687591552734}, {"y": 33.91817855834961, "x": -32.811187744140625}, {"y": 34.09068298339844, "x": -31.738685607910156}, {"y": 33.959041595458984, "x": -29.598907470703125}, {"y": 33.96776580810547, "x": -0.5014395713806152}, {"y": 34.000389099121094, "x": 0.5176341533660889}, {"y": 33.48455810546875, "x": 1.3077607154846191}, {"y": 32.55713653564453, "x": 1.553515911102295}, {"y": 31.470308303833008, "x": 1.4412281513214111}, {"y": 26.959705352783203, "x": 1.5491182804107666}, {"y": 26.068689346313477, "x": 1.6574804782867432}, {"y": 25.549406051635742, "x": 2.4118423461914062}, {"y": 25.40235137939453, "x": 3.219785213470459}, {"y": 25.483911514282227, "x": 4.841587543487549}, {"y": 25.64862060546875, "x": 6.924839496612549}, {"y": 25.571475982666016, "x": 13.471232414245605}, {"y": 25.419279098510742, "x": 15.124458312988281}, {"y": 24.356502532958984, "x": 16.07210922241211}, {"y": 22.99414825439453, "x": 16.297348022460938}, {"y": 21.70294761657715, "x": 15.853641510009766}, {"y": 20.401371002197266, "x": 14.811742782592773}, {"y": 10.674541473388672, "x": 6.36682653427124}, {"y": 8.482307434082031, "x": 4.451013088226318}, {"y": 6.8183746337890625, "x": 3.0730931758880615}, {"y": 5.353919506072998, "x": 2.6476709842681885}, {"y": 1.4918694496154785, "x": 1.9159927368164062}]
			a = []
			for i in d:
				a.append([i.values()[1]-22,i.values()[0]])
			return a
		# data = [[pose_stamped.pose.position.x, pose_stamped.pose.position.y] for pose_stamped in path_info.poses]
		#data_ved = np.array(data)
		data_vec = [[-.05*x, .0] for x in range(400)] #+ [[20, .05*x] for x in range(100)]
		#data_vec = conv()
		#pos_map = np.array([[self.position[0],self.position[1]]]) # (1,2)
		self.path = data_vec
		mark = self.make_marker(np.array(data_vec)) #generate marker message
		self.pub_line.publish(mark) #publish marker
		

	#def pose_callback(self, pose_marker):
	#	self.POSE = [pose_marker.points[0].x, pose_marker.points[0].y]


	
if __name__ == "__main__":
	rospy.init_node("path_planning")
	path_planning = PathPlanning()

	rospy.spin()
