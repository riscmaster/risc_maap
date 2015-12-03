#!/usr/bin/env python

'''======================================================
    Created by:  	Jose Lopez
    Last updated: 	January 2015
    File name: 	        coop_tarj1.py
    Organization:	RISC Lab, Utah State University

    Notes:

          the AR Drone will fly back and forth in the x axis direction
	  switching from one landmark to another.

 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
import numpy as np
from math import *
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
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 200)

# Trajectory Variables
period     = 30 # seconds
a          = 0.3
n          = 1
w1         = 2*PI/period
w2         = w1
w3         = w1

    #===================================#
    #    Radians between + or - pi/2    #
    #===================================#

def pi2pi(angle):
    while abs(angle) > np.pi:
        angle = angle - 2*np.pi*abs(angle)/angle
    return angle

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
    # Position
    traj1.x       = 0#a*cos(w2*t)
    traj1.y       = 0
    traj1.z       = n
    traj1.psi     = 0 #pi2pi(w2*t) #-PI/4 + PI*cos(w2*t)/2
    # Velocity
    traj1.xdot    = 0  #-a*w2*sin(w2*t)
    traj1.ydot    = 0
    traj1.zdot    = 0
    traj1.psidot  = 0
    traj1.psi     = pi2pi(w2) #-PI/4 + PI*cos(w2*t)/2
    # Acceleration
    traj1.xddot   = 0#-a*w2*w2*cos(w2*t)
    traj1.yddot   = 0
    traj1.zddot   = 0
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
    rospy.init_node('topology_trajectory')
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
