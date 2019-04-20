#!/usr/bin/env python2

"""
    RSS 2019 | Pure Pursuit controller for path tracking
    Author: Kyle Morgenstein
"""
from __future__ import division
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
#from nav_msgs.msg import Path
import numpy as np
import warnings



warnings.simplefilter('ignore', np.RankWarning)

class PathPlanning:
	# Access these variables in class functions with self:
	# i.e. self.CONSTANT
	PATH_TOPIC = rospy.get_param("/Trajectory_follower/path_topic")
	DRIVE_TOPIC = rospy.get_param("/Trajectory_follower/drive_topic")
	VELOCITY = float(rospy.get_param("/Trajectory_follower/velocity"))  # [m/s]
	local_topic = "/estim_pose"
	



	def __init__(self):
		#subs
		self.pose_sub = rospy.Subscriber(self.local_topic,Point32,self.pose_callback,queue_size=10)
		self.sub = rospy.Subscriber(self.PATH_TOPIC, PointCloud, self.callback, queue_size=10)
		#pubs
		self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
		self.pub_line = rospy.Publisher("marker",Marker,queue_size=10)
		#initialized vars
		self.position = np.zeros(2)
		self.path = 0



	def pose_callback(self,data):
		'''
		input: position [x,y,th]
		output: u: AckermannDriveStamped Message containing:
						steering angle [rad]
						rate of steering control [rad/s]
						velocity [m/s]
		'''
		self.position = np.array([data.x,data.y,data.z]) #sets global position variable
		pos_map = self.position[0:2] #keeps track of x,y for path transform
		data_vec = self.path #imports global path
		if self.path==0: #checks that path has been received
			pass
		d = np.array(data_vec-pos_map).reshape(-1,2) # (n,2), puts data in robot frame (robot is at (0,0)), splits data into x and y each of length n
		
		dists = np.einsum('ij,ij->i',d,d)
		i = np.argmin(dists) #index of closest waypoint
		try:
			path_remaining = d[i:i+25,:] #cuts off prior waypoints already passed
			dists_remaining = dists[i:i+25]
		except: #warps path to cyclic if nearing the end of the path
			path_remaining = np.concatenate((d[i:,:],d[:25,:]))
			dists_remaining = np.concatenate((dists[i:,:],dists[:25]))

		#combined proportional-pure persuit controller with Ackermann steering
		L = .324 #length of wheel base [m]
			
		#need speed controller for corners if want to attempt high speed maneuvers
		#x,y = path_remaining.T     to be used for speed controller in the future
		#m,b = np.polyfit(x[0:5],y[0:5],1)

		#dynamic lookahead distance
		v = self.VELOCITY
		if v<1:
			l=1
		elif 1<v<5:
			l=v/5+1
		else:
			l=v/2


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
			path_step = np.linalg.norm(p2-p1) #takes distance between p2 and p1 to find path step
			if 0<l-dists_remaining[j]<path_step:
				ind = j
				break
			#check if path is more than one lookahead distance from robot
			if l<dists_remaining[j]:
				ind = j
				l = dists_remaining[j]
				break
		else:
			ind = 0
		
		flag = False
		#take x,y coor of p1 and p2
		try: 
			
			x1 = path_remaining[ind,0]
			y1 = path_remaining[ind,1]
			x2 = path_remaining[ind+1,0]
			y2 = path_remaining[ind+1,1]
			if y2==y1: #edge case
				y_new = y1
				x_new = np.sqrt(l**2-y1**2)
				flag = True
			if x2==x1: #edge case
				x_new = x1
				y_new = np.sqrt(l**2-x1**2)
				flag = True
		except: #if at the end of the path, set point to last waypoint
			x_new = path_remaining[ind,0]
			y_new = path_remaining[ind,1]
			flag = True
		

		#standard computation
		if not flag:

			p1 = np.array([[x1,y1]])
			p2 = np.array([[x2,y2]])
			v = p2-p1

			a = np.dot(v,v.T)
			b = 2*(np.dot(v,p1.T))
			c = np.dot(p1,p1.T)-l**2
			
			t1 = min(1,max(0,(-b-np.sqrt(b**2-4*a*c))/(2*a)))
			t2 = min(1,max(0,(-b+np.sqrt(b**2-4*a*c))/(2*a)))
			t = max(t1,t2)

			new_point = p1 + t*v
			x_new,y_new = new_point.T


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
		for i in d:
			p = Point()
			p.x=i[0]
			p.y=i[1]
			p.z=0
			l.append(p)
		m.points=l

		self.pub_line.publish(m) #publish marker


	#def callback(self,path_info):
	def callback(self,d):
		'''
		sets path in map

		'''
		## data preprocessing
		#def conv():
		#	d = [{"y": -0.8895988464355469, "x": 0.4171772003173828}, {"y": -0.6701974868774414, "x": -30.42536163330078}, {"y": -0.3597745895385742, "x": -32.4092903137207}, {"y": 0.8645005226135254, "x": -33.037513732910156}, {"y": 30.7249698638916, "x": -33.17718505859375}, {"y": 32.2868766784668, "x": -33.165679931640625}, {"y": 33.415687561035156, "x": -33.283687591552734}, {"y": 33.91817855834961, "x": -32.811187744140625}, {"y": 34.09068298339844, "x": -31.738685607910156}, {"y": 33.959041595458984, "x": -29.598907470703125}, {"y": 33.96776580810547, "x": -0.5014395713806152}, {"y": 34.000389099121094, "x": 0.5176341533660889}, {"y": 33.48455810546875, "x": 1.3077607154846191}, {"y": 32.55713653564453, "x": 1.553515911102295}, {"y": 31.470308303833008, "x": 1.4412281513214111}, {"y": 26.959705352783203, "x": 1.5491182804107666}, {"y": 26.068689346313477, "x": 1.6574804782867432}, {"y": 25.549406051635742, "x": 2.4118423461914062}, {"y": 25.40235137939453, "x": 3.219785213470459}, {"y": 25.483911514282227, "x": 4.841587543487549}, {"y": 25.64862060546875, "x": 6.924839496612549}, {"y": 25.571475982666016, "x": 13.471232414245605}, {"y": 25.419279098510742, "x": 15.124458312988281}, {"y": 24.356502532958984, "x": 16.07210922241211}, {"y": 22.99414825439453, "x": 16.297348022460938}, {"y": 21.70294761657715, "x": 15.853641510009766}, {"y": 20.401371002197266, "x": 14.811742782592773}, {"y": 10.674541473388672, "x": 6.36682653427124}, {"y": 8.482307434082031, "x": 4.451013088226318}, {"y": 6.8183746337890625, "x": 3.0730931758880615}, {"y": 5.353919506072998, "x": 2.6476709842681885}, {"y": 1.4918694496154785, "x": 1.9159927368164062}]
		#	a = []
		#	for i in d:
		#		a.append([i.values()[1]-22,i.values()[0]])
		#	return a
		# data = [[pose_stamped.pose.position.x, pose_stamped.pose.position.y] for pose_stamped in path_info.poses]
		#data_ved = np.array(data)
		#data_vec = [[-.05*x, .0] for x in range(400)] #+ [[20, .05*x] for x in range(100)]
		#data_vec = [[i.x,i.y] for i in d]
		data_vec = [[13.761675857774913, -0.18534033611027692], [13.808737887808133, -0.20222717504337123], [13.855799917841354, -0.21911401397646554], [13.902861947874575, -0.23600085290955986], [13.949923977907796, -0.25288769184265414], [13.996986007941016, -0.2697745307757485], [14.044048037974237, -0.2866613697088428], [14.091110068007458, -0.3035482086419371], [14.138172098040679, -0.3204350475750314], [14.1852341280739, -0.3373218865081257], [14.23229615810712, -0.35420872544122006], [14.27935818814034, -0.37109556437431435], [14.326420218173562, -0.38798240330740863], [14.373482248206782, -0.40486924224050297], [14.420544278240003, -0.4217560811735973], [14.467606308273224, -0.4386429201066916], [14.514668338306445, -0.4555297590397859], [14.561730368339665, -0.47241659797288016], [14.608792398372886, -0.48930343690597455], [14.655854428406107, -0.5061902758390688], [14.702916458439327, -0.5230771147721631], [14.749978488472548, -0.5399639537052574], [14.797040518505769, -0.5568507926383518], [14.84410254853899, -0.5737376315714461], [14.89116457857221, -0.5906244705045404], [14.938226608605431, -0.6075113094376347], [14.985288638638652, -0.624398148370729], [15.032350668671873, -0.6412849873038233], [15.079412698705093, -0.6581718262369176], [12.794087183457918, 0.06498637182395212], [12.84249345855256, 0.052463092243035206], [12.890899733647203, 0.039939812662118296], [12.939306008741845, 0.027416533081201386], [12.987712283836487, 0.014893253500284476], [13.036118558931129, 0.0023699739193675656], [13.084524834025771, -0.010153305661549344], [13.132931109120413, -0.022676585242466255], [13.181337384215055, -0.035199864823383165], [13.229743659309698, -0.047723144404300075], [13.27814993440434, -0.060246423985216985], [13.326556209498982, -0.07276970356613388], [13.374962484593624, -0.0852929831470508], [13.423368759688266, -0.09781626272796773], [13.471775034782908, -0.11033954230888462], [13.52018130987755, -0.12286282188980152], [13.568587584972192, -0.13538610147071845], [13.616993860066835, -0.14790938105163537], [13.665400135161477, -0.16043266063255227], [13.713806410256119, -0.17295594021346916], [12.794087183457918, 0.06498637182395212], [12.744103016243233, 0.0662445561722967], [12.694118849028548, 0.06750274052064129], [12.644134681813863, 0.06876092486898587], [12.594150514599178, 0.07001910921733046], [12.544166347384493, 0.07127729356567504], [12.494182180169808, 0.07253547791401962], [12.444198012955123, 0.07379366226236421], [12.394213845740438, 0.0750518466107088], [12.344229678525753, 0.07631003095905338], [12.294245511311068, 0.07756821530739796], [12.244261344096383, 0.07882639965574255], [12.194277176881698, 0.08008458400408713], [12.144293009667013, 0.08134276835243172], [12.094308842452328, 0.0826009527007763], [12.044324675237643, 0.08385913704912089], [11.994340508022958, 0.08511732139746547], [11.944356340808273, 0.08637550574581006], [11.894372173593588, 0.08763369009415464], [11.844388006378903, 0.08889187444249923], [11.794403839164218, 0.09015005879084381], [11.744419671949533, 0.0914082431391884], [11.694435504734848, 0.09266642748753298], [11.644451337520163, 0.09392461183587757], [11.594467170305478, 0.09518279618422215], [11.544483003090793, 0.09644098053256674]]
		#data_vec = conv()
		#pos_map = np.array([[self.position[0],self.position[1]]]) # (1,2)
		self.path = data_vec
		self.make_marker(data_vec) #generate marker message
		
		
	
if __name__ == "__main__":
	rospy.init_node("path_planning")
	path_planning = PathPlanning()
	rospy.spin()
