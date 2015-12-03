#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	June  2014
    File name: 	        trajectory.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import cv2
import time

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

    #==============================#
    #    Figure Eight Trajectory   #
    #==============================#

    wp1 = Waypoint()
    wp1.x 	= radius*cos(theta)+center[0]
    wp1.y 	= radius*sin(2*theta)+center[1]
    wp1.z 	= 1
    if wp1.x <= 0:
        wp1.heading = 90
    if wp1.x >0:
        wp1.heading = 0


    wp2 = Waypoint()
    wp2.x 	= -radius*cos(theta)+center[0]
    wp2.y 	= -radius*sin(2*theta)+center[1]
    wp2.z 	= 1
    if wp2.x <= 0:
        wp2.heading = 90
    if wp2.x >0:
        wp2.heading = 0
    #========================#
    #    Circle Trajectory   #
    #========================#

#    wp1 = Waypoint()
#    wp1.x = radius*cos(0.5*period*(t)+center[0])
#    wp1.y = radius*sin(0.5*period*(t)+center[1])
#    wp1.z = 1

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
    rospy.init_node('risc_hand_controller')
    start_time = rospy.get_time()

    #=======================#
    #    Create Trackbars   #
    #=======================#

    image = cv2.imread("/home/ece/ros_ws/src/quad_command/mario.jpg")
    cv2.resize(image,(321,123))
    cv2.namedWindow("Properties")
    cv2.createTrackbar("Radius 100=1m", "Properties", R, 100, FR)
    cv2.createTrackbar("X_center = -0.0425+(x-50)*.01", "Properties", X, 100, FcenterX)
    cv2.createTrackbar("Y_center = -0.0425+(y-50)*.01", "Properties", Y, 100, FcenterY)
    cv2.createTrackbar("Period 100=10s", "Properties", P, 1000, FP)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        cv2.imshow("Properties",image)
        cv2.waitKey(3)
        Datahandler()
        r.sleep()
    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Awkward...')
    sys.exit(status)

