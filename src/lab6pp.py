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
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
import utils
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
#from nav_msgs.msg import Path
import numpy as np
import warnings



warnings.simplefilter('ignore', np.RankWarning)

class PureP:
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    PATH_TOPIC = rospy.get_param("/Trajectory_follower/path_topic")
    DRIVE_TOPIC = rospy.get_param("/Trajectory_follower/drive_topic")
    VELOCITY = float(rospy.get_param("/Trajectory_follower/velocity"))  # [m/s]
    local_topic = "/pf/viz/inferred_pose"

    def __init__(self):
        # controller params
        self.Kd_gain = float(rospy.get_param("/Trajectory_follower/Kd_gain"))
        self.seg_len = int(rospy.get_param("/Trajectory_follower/seg_len")) # distance ahead for Linear Regression
        self.corner_angle = int(rospy.get_param("/Trajectory_follower/corner_ang")) # Threshold for corner
        #subs
        self.pose_sub = rospy.Subscriber(self.local_topic,PoseStamped,self.pose_callback,queue_size=1)
        # self.sub = rospy.Subscriber(self.PATH_TOPIC, PointCloud, self.callback, queue_size=10)
        self.sub = rospy.Subscriber(self.PATH_TOPIC, PointCloud, self.callback, queue_size=10)
        # pubs
        self.pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=10)
        self.pub_line = rospy.Publisher("marker",Marker,queue_size=10)
    self.pt_pub = rospy.Publisher("/next_pt", PointCloud, queue_size=10)
    self.rel_path_pub = rospy.Publisher("/rel_path", PointCloud, queue_size=10)
        #initialized vars
        self.position = np.zeros(2)
        self.path = 0
        self.HAVE_PATH = False
        print "Pure Pursuit initialized"
        self.last_dist = 0
        self.last_t = rospy.get_time()
        self.num_deriv = 5 # number of samples for running average of derivative
        self.derivs = [] 


    def pose_callback(self,data):
        '''
        input: position [x,y,th]
        output: u: AckermannDriveStamped Message containing:
                        steering angle [rad]
                        rate of steering control [rad/s]
                        velocity [m/s]
        '''
        #print "tracking path"
        time = rospy.get_time()
        euler_angles = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        self.position = np.array([data.pose.position.x,data.pose.position.y,euler_angles[2]]) #sets global position variable
        pos_map = self.position[0:2] #keeps track of x,y for path transform
        data_vec = np.array(self.path) #imports global path
        if self.path==0: #checks that path has been received
            A = AckermannDriveStamped()
            A.drive.speed = 0 #sets velocity [m/s]
            A.drive.steering_angle = 0 #determines input steering control
            A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
            self.pub.publish(A) #publish steering command
            return

        L = .324 #length of wheel base [m]
        dists = np.sqrt((data_vec.T[0]-self.position[0])**2+(data_vec.T[1]-self.position[1])**2)
        i = np.argmin(dists) #index of closest waypoint
        dists_remaining = dists[i:]
        path_remaining = data_vec[i:,:]

        d = data_vec
        for index in range(len(dists)):
            d[index] = np.matmul(np.array([[np.cos(self.position[2]), -np.sin(self.position[2])], [np.sin(self.position[2]), np.cos(self.position[2])]]).T, np.array([self.path[index][0] - self.position[0], self.path[index][1] - self.position[1]]).T)


        x,y = d[i:,:].T     #to be used for speed controller in the future
        m,b = np.polyfit(x[0:self.seg_len],y[0:self.seg_len],1) #need to set seg_lengh to ~30
        delt = (np.arctan2(m,1))*180/np.pi
        # print('delt:  ',delt)
        if delt>110:
            delt-=180
        elif delt<-110:
            delt+=180
        if not -self.corner_angle<delt<self.corner_angle:
            #print('slow corner')
            vel = 2
            l = 1.5
        else:
            #print('fast')
            vel = self.VELOCITY
            l = 2.7

        for j in range(1,len(dists_remaining)):
            p1 = dists_remaining[j-1]
            p2 = dists_remaining[j]
            if p1<=l<p2:
                ind = j-1
                print('ind in path: ',ind)
                break
        else:
            ind = 0 #far from path condition
            l = dists_remaining[0]


        #if l<dists_remaining[0]:
        #    l = dists_remaining[0]
        #    print('far from path')
        #    new_ind = 0
        #else:
        #    new_ind = int(10*l)

        # print('ind:  ',new_ind)
        x_new,y_new = path_remaining[ind,:].T
        #kp = 0.1
        #prop = path_remaining[0,1]
        self.pub_point(path_remaining[0])
        #self.pub_rel_path(path_remaining)
        #print "Proportional Error:", prop
        # compute ackermann steering angle to feed into cotroller
        eta = np.arctan2(y_new,x_new)-self.position[2] #angle between velocity vector and desired path [rad]
        u = np.arctan(2*L*np.sin(eta)/l)#+kp*prop #+self.Kd_gain*err_d#+kp #sets input steering angle from controller [rad]
        #print "sending steering command"
        A = AckermannDriveStamped()
        A.drive.speed = vel #sets velocity [m/s]
        A.drive.steering_angle = u #determines input steering control
        A.drive.steering_angle_velocity = 0 #determines how quickly steering is adjuted, 0 is instantaneous [rad/s]
        self.pub.publish(A) #publish steering command

    def get_deriv(self, new_d):
        """
        Calculates running average of the derivative of
        the crosstrack error
        """
        if len(self.derivs) < self.num_deriv:
            self.derivs.append(new_d)

        else:
            self.derivs.append(new_d)
            self.derivs.pop(0)

        return np.average(self.derivs)

    def pub_point(self, point):
        '''
        Create and publish point cloud of a point.
        '''
        self.cloud = PointCloud()
        self.cloud.header.frame_id = "/base_link"
        self.cloud.points = [Point32()]

        self.cloud.points[0].x = point[0]
        self.cloud.points[0].y = point[1]

        self.pt_pub.publish(self.cloud)

    def pub_rel_path(self, path_pts):
        self.path_cloud = PointCloud()
        self.path_cloud.header.frame_id = "/base_link"
        self.path_cloud.points = [Point32() for p in path_pts]

    for i, pt in enumerate(path_pts):
        self.path_cloud.points[i].x = pt[0]
        self.path_cloud.points[i].y = pt[1]

        self.rel_path_pub.publish(self.path_cloud)


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
        #   d = [{"y": -0.8895988464355469, "x": 0.4171772003173828}, {"y": -0.6701974868774414, "x": -30.42536163330078}, {"y": -0.3597745895385742, "x": -32.4092903137207}, {"y": 0.8645005226135254, "x": -33.037513732910156}, {"y": 30.7249698638916, "x": -33.17718505859375}, {"y": 32.2868766784668, "x": -33.165679931640625}, {"y": 33.415687561035156, "x": -33.283687591552734}, {"y": 33.91817855834961, "x": -32.811187744140625}, {"y": 34.09068298339844, "x": -31.738685607910156}, {"y": 33.959041595458984, "x": -29.598907470703125}, {"y": 33.96776580810547, "x": -0.5014395713806152}, {"y": 34.000389099121094, "x": 0.5176341533660889}, {"y": 33.48455810546875, "x": 1.3077607154846191}, {"y": 32.55713653564453, "x": 1.553515911102295}, {"y": 31.470308303833008, "x": 1.4412281513214111}, {"y": 26.959705352783203, "x": 1.5491182804107666}, {"y": 26.068689346313477, "x": 1.6574804782867432}, {"y": 25.549406051635742, "x": 2.4118423461914062}, {"y": 25.40235137939453, "x": 3.219785213470459}, {"y": 25.483911514282227, "x": 4.841587543487549}, {"y": 25.64862060546875, "x": 6.924839496612549}, {"y": 25.571475982666016, "x": 13.471232414245605}, {"y": 25.419279098510742, "x": 15.124458312988281}, {"y": 24.356502532958984, "x": 16.07210922241211}, {"y": 22.99414825439453, "x": 16.297348022460938}, {"y": 21.70294761657715, "x": 15.853641510009766}, {"y": 20.401371002197266, "x": 14.811742782592773}, {"y": 10.674541473388672, "x": 6.36682653427124}, {"y": 8.482307434082031, "x": 4.451013088226318}, {"y": 6.8183746337890625, "x": 3.0730931758880615}, {"y": 5.353919506072998, "x": 2.6476709842681885}, {"y": 1.4918694496154785, "x": 1.9159927368164062}]
        #   a = []
        #   for i in d:
        #       a.append([i.values()[1]-22,i.values()[0]])
        #   return a
        # data = [[pose_stamped.pose.position.x, pose_stamped.pose.position.y] for pose_stamped in path_info.poses]
        #data_ved = np.array(data)
        #data_vec = [[-.05*x, .0] for x in range(400)] #+ [[20, .05*x] for x in range(100)]
        #print("map initialized")

        data_vec = [[i.x,i.y] for i in d.points]
        #data_vec = conv()
        #pos_map = np.array([[self.position[0],self.position[1]]]) # (1,2)
        self.path = data_vec
        dmark = []
        for i in data_vec:
            dmark.append(i)
            if i!=data_vec[0] and i!=data_vec[-1]:
                dmark.append(i)
        self.HAVE_PATH = True
        self.make_marker(dmark) #generate marker message



if __name__ == "__main__":
    rospy.init_node("PureP")
    path_planning = PureP()
    rospy.spin()