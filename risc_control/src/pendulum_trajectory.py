#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	December  2014
    File name: 	        pendulum_trajectory.py
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

from risc_msgs.msg import *

    #========================#
    #        Globals         #
    #========================#

# enable time, pi and publisher
start_time = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 1)

# Trajectory Variables
period     = 5 # seconds
a          = 0 #.75
b          = 0 # .5
c          = 0
n          = 1.5
w1         = 2*PI/period
w2         = w1
w3         = w1

    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, PI, pub, period, a, b, c, n, w1, w2, w3
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    WP 		= Trajectories()
    WP.Obj 	= [Trajectory()]*1

    #=================#
    #    Trajectory   #
    #=================#

    traj1 = Trajectory()
    traj1.name = "pendulum trajectory"
    # Position
    traj1.x       = a*cos(w2*t)
    traj1.y       = b*sin(w1*t)
    traj1.z       = n+c*sin(w3*t)
    traj1.psi     = 0
    # Velocity
    traj1.xdot    = -a*w2*sin(w2*t)
    traj1.ydot    = b*w1*cos(w1*t)
    traj1.zdot    = c*w3*cos(w3*t)
    traj1.psidot  = 0
    # Acceleration
    traj1.xddot   = -a*w2*w2*cos(w2*t)
    traj1.yddot   = -b*w1*w1*sin(w1*t)
    traj1.zddot   = -c*w3*w3*sin(w3*t)+9.81
    traj1.psiddot = 0

    #==================#
    #     Publish      #
    #==================#

    WP.Obj = [traj1]
    pub.publish(WP)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('pendulum_trajectory')
    start_time = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        Datahandler()
        r.sleep()
    # and only progresses to here once the application has been shutdown
    rospy.loginfo("Trajectory Node Has Shutdown.")
    rospy.signal_shutdown(0)
