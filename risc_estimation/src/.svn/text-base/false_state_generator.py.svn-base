#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	October  2014
    File name: 	        false_state_generator.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import cv2
import time
import numpy as np

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import Cortex
from risc_msgs.msg import States

    #==========================#
    #    Trackbar Variables    #
    #==========================#

noise = 0
act_noise = .0001*noise

    #========================#
    #        Globals         #
    #========================#

image      = 0
start_time = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('false_data',Cortex,queue_size = None)

    #==========================#
    #    Trackbar Functions    #
    #==========================#

# Noise
def Fnoise(x):
    global noise,act_noise
    noise = x
    act_noise = noise*.01


    #======================================#
    #    Update and Publish False States   #
    #======================================#

def Datahandler():
    global start_time,pub, PI,act_noise
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    St   	= Cortex()
    St.Obj 	= [States()]*1
    period      = 10
    cycle 	= floor(t/period)
    theta 	= 2*PI*(t-cycle*period)/period
    radius      = 1
    center      = [0,0]

    #==============================#
    #    Figure Eight Trajectory   #
    #==============================#

#    St1   = States()
#    n = sqrt(act_noise)
#    if n == 0:
#        n = .00001
#    St1.x = radius*cos(theta)+center[0] + np.random.normal(0,n,1)[0]
#    St1.y = radius*sin(2*theta)+center[1]+ np.random.normal(0,n,1)[0]
#   St1.z = 1+np.random.normal(0,n,1)[0]

    #========================#
    #    Circle Trajectory   #
    #========================#
    St1   = States() 
    n = sqrt(act_noise)
    if n == 0:
       n = .00001
    St1.x = radius*cos(0.2*period*(t)+center[0])+np.random.normal(0,n,1)[0]
    St1.y = radius*sin(0.5*period*(t)+center[1])+np.random.normal(0,n,1)[0]
    St1.z = 1

    #==================#
    #     Publish      #
    #==================#

    St.Obj = [St1]
    pub.publish(St)

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
    cv2.namedWindow("Noise")
    cv2.createTrackbar("Noise*.01", "Noise", noise, 100, Fnoise)

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
    sys.exit(status)

