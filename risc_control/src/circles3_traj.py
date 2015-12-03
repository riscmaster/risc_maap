#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	July  2015
    File name: 	        circles3_traj.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *

    #========================#
    #        Globals         #
    #========================#

start_time = 0
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 200)

# Trajectory Variables
period     = 8 # seconds
a          = 0
b          = 0
c          = 0
n          = 1
w1         = 2*np.pi/period

    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, pub, period, a, b, c, n, w1
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    WP 		= Trajectories()
    num_traj    = 3 # number of trajectories
    WP.Obj 	= [Trajectory()]*num_traj
    d = 0.5 #Distance from origin

    #=================#
    #    Trajectory   #
    #=================#

    traj1 = Trajectory()
    # Position
    traj1.x       = d*cos(0*np.pi/num_traj)+a*cos(w1*t)
    traj1.y       = d*sin(0*np.pi/num_traj)+b*sin(w1*t)
    traj1.z       = n+c*sin(w1*t)
    traj1.psi     = w1*t
    # Velocity
    traj1.xdot    = -a*w1*sin(w1*t)
    traj1.ydot    = b*w1*cos(w1*t)
    traj1.zdot    = c*w1*cos(w1*t)
    traj1.psidot  = w1
    # Acceleration
    traj1.xddot   = -a*w1*w1*cos(w1*t)
    traj1.yddot   = -b*w1*w1*sin(w1*t)
    traj1.zddot   = -c*w1*w1*sin(w1*t)
    traj1.psiddot = 0


    traj2 = Trajectory()
    # Position
    traj2.x       = d*cos(2*1*np.pi/num_traj)+a*cos(w1*t+period/num_traj)
    traj2.y       = d*sin(2*1*np.pi/num_traj)+b*sin(w1*t+period/num_traj)
    traj2.z       = n+c*sin(w1*t+period/num_traj)
    traj2.psi     = w1*t+period/num_traj
    # Velocity
    traj2.xdot    = -a*w1*sin(w1*t+period/2)
    traj2.ydot    = b*w1*cos(w1*t+period/2)
    traj2.zdot    = c*w1*cos(w1*t+period/2)
    traj2.psidot  = w1
    # Acceleration
    traj2.xddot   = -a*w1*w1*cos(w1*t+period/2)
    traj2.yddot   = -b*w1*w1*sin(w1*t+period/2)
    traj2.zddot   = -c*w1*w1*sin(w1*t+period/2)
    traj2.psiddot = 0


    traj3 = Trajectory()
    # Position
    traj3.x       = d*cos(2*2*np.pi/num_traj)+a*cos(w1*t+2*period/num_traj)
    traj3.y       = d*sin(2*2*np.pi/num_traj)+b*sin(w1*t+2*period/num_traj)
    traj3.z       = n+c*sin(w1*t+2*period/num_traj)
    traj3.psi     = w1*t+2*period/num_traj
    # Velocity
    traj3.xdot    = -a*w1*sin(w1*t+period/2)
    traj3.ydot    = b*w1*cos(w1*t+period/2)
    traj3.zdot    = c*w1*cos(w1*t+period/2)
    traj3.psidot  = w1
    # Acceleration
    traj3.xddot   = -a*w1*w1*cos(w1*t+period/2)
    traj3.yddot   = -b*w1*w1*sin(w1*t+period/2)
    traj3.zddot   = -c*w1*w1*sin(w1*t+period/2)
    traj3.psiddot = 0

    #==================#
    #     Publish      #
    #==================#

    WP.Obj = [traj1, traj2, traj3]
    pub.publish(WP)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    rospy.init_node('circles_traj')
    start_time = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        Datahandler()
        r.sleep()
    rospy.loginfo("Trajectory Node Has Shutdown.")
    rospy.signal_shutdown(0)
