#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	December  2014
    File name: 	        toroid_knot.py
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
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 200)

# Trajectory Variables
period     = 10 # seconds
a          = 0.65
b          = 0.25
c          = 0.25
n          = 1
q          = 3 # number of loops and number of quads
p          = q-1
w1         = 2*PI/period

    #========================================#
    #    Generate A Trajectory given Theta   #
    #========================================#

def GenerateTraj(theta):
    global PI, period, a, b, c, n,p,q
    r = a+b*cos(q*theta/p)
    z = n+c*sin(q*theta/p)
    rdot = -b*q*sin(q*theta/p)
    zdot = c*q*cos(q*theta/p)
    rddot = -b*q*q*cos(q*theta/p)/(p*p)
    zddot = -c*q*q*sin(q*theta/p)/(p*p)

    #=================#
    #    Trajectory   #
    #=================#

    traj = Trajectory()
    traj.name = "toroid knot"
    # Position
    traj.x       = r*cos(theta)
    traj.y       = r*sin(theta)
    traj.z       = z
    traj.psi     = 0
    # Velocity
    traj.xdot    = rdot*cos(theta) - r*sin(theta)
    traj.ydot    = rdot*sin(theta) + r*cos(theta)
    traj.zdot    = zdot
    traj.psidot  = 0
    # Acceleration
    traj.xddot   = rddot*cos(theta)-2*rdot*sin(theta)-r*cos(theta)
    traj.yddot   = rddot*sin(theta)+2*rdot*cos(theta)-r*sin(theta)
    traj.zddot   = zddot+9.81
    traj.psiddot = 0
    return traj


    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, PI, pub,w1
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    WP 		= Trajectories()
    WP.Obj 	= [Trajectory()]*q
    offset      = 2*PI*p/q
    for i in range(q):
        theta = w1*t+i*offset
        WP.Obj[i] = GenerateTraj(theta)

    #==================#
    #     Publish      #
    #==================#

    pub.publish(WP)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('toroid_knot_trajectory')
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
