#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		quadrants_traj.py
    Organization:	RISC Lab, Utah State University
    Notes:
    ======================================================'''

    #================================#
    #    Libraries/modules Needed    #
    #================================#

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

    #========================#
    #        Globals         #
    #========================#

rate            = 45      # Hz
start_time      = 0
back            = 0
forward         = 0
mode            = 1 # mode of 4 listed under cases
old_mode        = 0
ctrl_status     = False
cases           = ['Origin','Quadrant I','Quadrant II',\
                   'Quadrant III','Quadrant IV','Quadrant I']
    #==================#
    #    Publishers    #
    #==================#

pub_traj        = rospy.Publisher('/trajectory', Trajectories, queue_size = 1)

    #=========================================#
    #    Bound Radians between + or - pi/2    #
    #=========================================#

def pi2pi(angle):
     if abs(angle)>PI/2:
           if angle>0:
                angle = angle-PI
           else:
                angle = angle+PI
     return angle
    
    #==============#
    #    Get Joy   #
    #==============#

def GetJoy(joy):
    global start_time, mode, old_mode, forward, back
    if (joy.buttons[5] == 1 and forward == 1) or (joy.buttons[4] == 1 and back == -1):
        mode = mode
    else:
        back    = -joy.buttons[4]
        forward = joy.buttons[5]
        old_mode    = mode
        start_time  = rospy.get_time()
        mode        = mode + back + forward
        if mode > 6:
            mode = 1
        if mode < 1:
            mode = 6

    #============================#
    #    Get Controller Status   #
    #============================#

def GetStatus(S):

    global ctrl_status
    ctrl_status = S.data

    #==========================#
    #   Various Trajectories   #
    #==========================#

def Origin():

    global K_way
    traj = GetTrajectory(10, 0,0,0,1,0,0,0,mode)

def QuadrantI():

    global K_way
    traj = GetWaypoint(.75,.75,1,mode)

def QuadrantII():

    global K_way
    traj = GetWaypoint(-.75,.75,1,mode)

def QuadrantIII():

    global K_way
    traj = GetWaypoint(-.75,-.75,1,mode)

def QuadrantIV():

    global K_way
    traj = GetWaypoint(.75,-.75,1,mode)


def Datahandler():
    global mode, old_mode
    if mode == 1:
        Origin()
        if old_mode != mode:
            rospy.loginfo("Origin")
            old_mode = mode
    if mode == 2 or mode == 6:
        QuadrantI()
        if old_mode != mode:
            rospy.loginfo("Quadrant I")
            old_mode = mode
    if mode == 3:
        QuadrantII()
        if old_mode != mode:
            rospy.loginfo("Quadrant II")
            old_mode = mode
    if mode == 4:
        mode = 4 # redefine mode to prevent control lockout (I don't know why this is necessary but it fixes it)
        QuadrantIII()
        if old_mode != mode:
            rospy.loginfo("Quadrant III")
            old_mode = mode
    if mode == 5:
        mode = 5 # redefine mode to prevent control lockout (I don't know why this is necessary but it fixes it)
        QuadrantIV()
        if old_mode != mode:
            rospy.loginfo("Quadrant IV")
            old_mode = mode
    if rospy.get_param('controller_status',False):
        start_time = rospy.get_time()

    #===================#
    #    Get Waypoint   #
    #===================#

def GetWaypoint(x,y,z,case):

    global start_time, cases, pub_traj
    time_now    = rospy.get_time()
    t           = time_now-start_time
    WP          = Trajectories()
    WP.Obj      = [Trajectory()]*1

    #=================#
    #    Trajectory   #
    #=================#

    traj = Trajectory()
    traj.name = cases[case-1]
    # Position
    traj.x       = x
    traj.y       = y
    traj.z       = z
    traj.psi     = 0
    # Velocity
    traj.xdot    = 0
    traj.ydot    = 0
    traj.zdot    = 0
    traj.psidot  = 0
    # Acceleration
    traj.xddot   = 0
    traj.yddot   = 0
    traj.zddot   = 0
    traj.psiddot = 0

    WP.Obj = [traj]
    pub_traj.publish(WP)
    return WP



    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTrajectory(period,a,b,c,n,w1,w2,w3,case):

    global start_time, cases, pub_traj
    time_now    = rospy.get_time()
    t           = time_now-start_time
    WP          = Trajectories()
    WP.Obj      = [Trajectory()]*1
    #=================#
    #    Trajectory   #
    #=================#
    traj = Trajectory()
    traj.name    = cases[case-1]
    # Position
    traj.x       = a*cos(w2*t)
    traj.y       = b*sin(w1*t)
    traj.z       = n+c*sin(w3*t)
    traj.psi     = 0
    # Velocity
    traj.xdot    = -a*w2*sin(w2*t)
    traj.ydot    = b*w1*cos(w1*t)
    traj.zdot    = c*w3*cos(w3*t)
    traj.psidot  = 0
    # Acceleration
    traj.xddot   = -a*w2*w2*cos(w2*t)
    traj.yddot   = -b*w1*w1*sin(w1*t)
    traj.zddot   = -c*w3*w3*sin(w3*t)
    traj.psiddot = 0
    WP.Obj = [traj]
    pub_traj.publish(WP)
    return WP

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Trajectory')
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        sub3 = rospy.Subscriber('/joy' , Joy, GetJoy)
        sub4 = rospy.Subscriber('/controller_status' , Bool, GetStatus)
        Datahandler()
        r.sleep()

