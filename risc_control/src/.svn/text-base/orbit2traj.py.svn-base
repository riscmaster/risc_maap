#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	August  2014
    File name: 	        orbit2traj.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import cv2
import time
import rospkg

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import Waypoints  # for publishing waypoints
from risc_msgs.msg import Waypoint   # for initialization of waypoints

    #==========================#
    #    Trackbar Variables    #
    #==========================#

P = 100
R = 80
X = 50
Y = 50

    #========================#
    #        Globals         #
    #========================#

period     = .1*P
radius     = 0.01*R
center     = [-0.01105+(X-50)*.01,-0.0425+(Y-50)*.01]
image      = 0
start_time = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('waypoints',Waypoints,queue_size = None)

    #==========================#
    #    Trackbar Functions    #
    #==========================#

# Period
def FP(x):
    global P,period
    period = .1*x
    P = x
    if x==0:
        period = .000001

# Center
def FcenterX(x):
    global center,X
    center[0] = -0.01105+(x-50)*.01
    X = x
def FcenterY(x):
    global Y,center
    center[1] = -0.0425+(x-50)*.01
    Y = x

# Radius
def FR(x):
    global R,radius
    radius = 0.01*x
    R = x

    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global center, radius, period, image, start_time,pub, PI
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    WP 		= Waypoints()
    WP.Obj 	= [Waypoint()]*1
    cycle 	= floor(t/period)
    theta 	= 2*PI*(t-cycle*period)/period

    #==============================================#
    #    Circle Trajectory Heading facing Center   #
    #==============================================#

    wp1 = Waypoint()
    wp1.x = radius*cos(theta)+center[0]
    wp1.y = radius*sin(theta)+center[1]
    wp1.z = 1
#    wp1.heading = (theta+PI)*180/PI
#    if wp1.heading>=180:
#           wp1.heading = wp1.heading-360
#    if wp1.heading<-180:
#           wp1.heading = wp1.heading+360

    wp2 = Waypoint()
    wp2.x = radius*cos(theta+PI)+center[0]
    wp2.y = radius*sin(theta+PI)+center[1]
    wp2.z = 1
#    wp2.heading = (theta)*180/PI
#    if wp2.heading>=180:
#           wp2.heading = wp2.heading-360
#    if wp2.heading<-180:
#           wp2.heading = wp2.heading+360


#    #=========================================#
#    #    Circle Trajectory Heading Spinning   #
#    #=========================================#
#
#    wp1 = Waypoint()
#    wp1.x = radius*cos(theta)+center[0]
#    wp1.y = radius*sin(theta)+center[1]
#    wp1.z = 1
#    wp1.heading = (theta*2+PI)*180/PI
#
#
#    wp2 = Waypoint()
#    wp2.x = radius*cos(theta+PI)+center[0]
#    wp2.y = radius*sin(theta+PI)+center[1]
#    wp2.z = 1
#    wp2.heading = (theta*2)*180/PI
#

    #==================#
    #     Publish      #
    #==================#

    WP.Obj = [wp1, wp2]
    pub.publish(WP)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('orbit')
    start_time = rospy.get_time()

    #=======================#
    #    Create Trackbars   #
    #=======================#

    rospack = rospkg.RosPack()
    path = rospack.get_path('risc_visual')
    image = cv2.imread(path+"/mountains.jpg")
    cv2.namedWindow("Properties")
    cv2.createTrackbar("Radius 100=1m", "Properties", R, 100, FR)
    cv2.createTrackbar("X_center = -0.0425+(x-50)*.01", "Properties", X, 100, FcenterX)
    cv2.createTrackbar("Y_center = -0.0425+(y-50)*.01", "Properties", Y, 100, FcenterY)
    cv2.createTrackbar("Period 100=10s", "Properties", P, 1000, FP)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        cv2.imshow("Properties",image)
        cv2.waitKey(3)
        Datahandler()
        r.sleep()
    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Awkward...')

