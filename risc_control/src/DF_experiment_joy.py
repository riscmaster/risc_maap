#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	February 2015
    File name: 		DF_experiment_joy.py
    Organization:	RISC Lab, Utah State University
    Notes:
        This file is meant to allow use of the joystick
        to toggle between trajectories.
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import rospkg
import numpy as np
import scipy.linalg as la
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
states          = Cortex()
states.Obj      = [States()]*1
euler_max       = 0.349066 #in radians
max_yaw_rate    = .3490659 #in radians/sec
max_alt_rate    = 1000     # in mm/sec
rate            = 45      # Hz
start_time      = 0
back            = 0
forward         = 0
mode            = 1 # mode of 4 listed under cases
old_mode        = 0
cases           = ['Origin','Slanted Figure Eight','Origin',\
                   'Flat Figure Eight','Origin','Circle']
    #==================#
    #    Publishers    #
    #==================#

pubTakeoff      = rospy.Publisher('/ardrone/takeoff',Empty, queue_size = 1)
pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)
pub_traj        = rospy.Publisher('/trajectory', Trajectories, queue_size = 1)

    #=====================#
    #    Gain Matrices    #
    #=====================#

#K_way           = np.matrix([[  .1,       0,    0,  .25,    0,      0, 0],\
#                             [    0,     .1,    0,    0,     .25,    0, 0],\
#                             [    0,       0,    -.4,    0,       0,    -0.7, 0],\
#                             [    0,       0,    0,    0,    0,      0, 1]])
K_way           = np.matrix([[   .2,    0,    0,  .34,    0,    0, 0],\
                             [    0,   .2,    0,    0,  .34,    0, 0],\
                             [    0,    0, -.6,    0,    0, -6.6, 0],\
                             [    0,    0,    0,    0,    0,    0, 1]])
K_slf8          = np.matrix([[  .27,    0,    0, .7,    0,    0, 0],\
                             [    0, .27,    0,    0, .7,    0, 0],\
                             [    0,    0,  -.6,    0,    0, -6.6, 0],\
                             [    0,    0,    0,    0,    0,    0, 1]])
K_flf8          = np.matrix([[  .27,    0,    0, .7,    0,    0, 0],\
                             [    0, .27,    0,    0, .7,    0, 0],\
                             [    0,    0,  -.6,    0,    0, -6.6, 0],\
                             [    0,    0,    0,    0,    0,    0, 1]])
K_crcl          = np.matrix([[  .27,    0,    0, .7,    0,    0, 0],\
                             [    0, .27,    0,    0, .7,    0, 0],\
                             [    0,    0, -.6,    0,    0, -6.6, 0],\
                             [    0,    0,    0,    0,    0,    0, 1]])

    #===================================#
    #    Radians between + or - pi/2    #
    #===================================#

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

    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTrajectory(period,a,b,c,n,w1,w2,w3,case):
    global start_time, time_past, cases, pub_traj
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

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #============#
    #   Origin   #
    #============#

def Origin():

    global K_way
    traj = GetTrajectory(10, 0,0,0,1,0,0,0,mode)
    Basic_Controller(traj, K_way)

    #======================#
    #   Slanted Figure 8   #
    #======================#

def Slanted_Figure_8():

    global K_slf8,cycles
    # Trajectory Variables
    period     = 10 # seconds
    a          = 1
    b          = 0.5
    c          = 0.5
    n          = 1
    w1         = 2*PI/period
    w2         = w1/2
    w3         = w1
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_slf8)

    #===================#
    #   Flat Figure 8   #
    #===================#

def Flat_Figure_8():

    global K_flf8,cycles
    # Trajectory Variables
    period     = 10 # seconds
    a          = 1
    b          = 0.5
    c          = 0.0
    n          = 1
    w1         = 2*PI/period
    w2         = w1/2
    w3         = w1
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_flf8)

    #============#
    #   Circle   #
    #============#

def Circle():

    global K_crcl
    # Trajectory Variables
    period     = 8 # seconds
    a          = 0.8
    b          = 0.8
    c          = 0
    n          = 1
    w1         = 2*PI/period
    w2         = w1
    w3         = w1
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_crcl)

    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller(traj,K):
    global states,PI, euler_max, max_yaw_rate, max_alt_rate, pub_ctrl

    #rospy.loginfo("In Basic controller")
    Ctrl        = Controls()
    # Initiate Control Messages
    bodies = 1
    Ctrl.Obj = [Control()]*bodies
    Ctrl.header.stamp = states.header.stamp
    g = 9.81
    m = .450 # ARDrone mass

        #===================================#
        #    Get State Trajectory Errors    #
        #===================================#

    if states.Obj[0].visible:
        X = np.asmatrix(np.zeros((7,1)))
        X[0] = traj.Obj[0].x-states.Obj[0].x
        X[1] = traj.Obj[0].y-states.Obj[0].y
        X[2] = traj.Obj[0].z-states.Obj[0].z
        X[3] = traj.Obj[0].xdot-states.Obj[0].u
        X[4] = traj.Obj[0].ydot-states.Obj[0].v
        X[5] = traj.Obj[0].zdot-states.Obj[0].w
        X[6] = pi2pi(traj.Obj[0].psi)-states.Obj[0].psi*PI/180

        #============================================#
        #     Differential Flatness Control Input    #
        #============================================#

        # LQR input
        utilde = -K*X
        # required input
        u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot],[traj.Obj[0].psidot]])
        u = utilde-u_r+np.matrix([[0],[0],[9.81],[0]])

        #==================================#
        #     Rotate to Vehicle 1 Frame    #
        #==================================#

        psi = states.Obj[0].psi*PI/180
        rotZ = np.matrix([[cos(-psi), -sin(-psi), 0],[sin(-psi), cos(-psi), 0],[0, 0, 1]])
        Cart = np.matrix([[-1, 0, 0],[0, -1, 0],[0, 0, 1]]) # fix x and y directions
        u[:-1] = Cart*rotZ*u[:-1]

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

        T = sqrt(u[0:3].T*u[0:3])
        u[:-1] = np.divide(u[:-1],T)

        #==================#
        #   Set Controls   #
        #==================#

        # Controls for Ardrone
        # -phi = right... +phi = left
        # -theta = back... +theta = forward
        # -psi = right...  +psi = left
        ctrl        = Control()
        ctrl.name   = states.Obj[0].name
        ctrl.phi    = asin(u[1,-1])/euler_max
        ctrl.theta  = asin(u[0,-1])/euler_max
        ctrl.psi    = u[3,-1] /max_yaw_rate
        ctrl.T      = T*m
        Ctrl.Obj[0] = ctrl
        Ctrl.header = states.header
        #rospy.loginfo("latency = %f",states.header.stamp.to_sec()-rospy.get_time())
        pub_ctrl.publish(Ctrl)

def Datahandler():
    global mode, old_mode
    if mode == 1 or mode == 3 or mode == 5:
        Origin()
        if old_mode != mode:
            rospy.loginfo("Origin")
            old_mode = mode
    if mode == 2:
        Slanted_Figure_8()
        if old_mode != mode:
            rospy.loginfo("Slanted Figure 8 Trajectory")
            old_mode = mode
    if mode == 4:
        Flat_Figure_8()
        if old_mode != mode:
            rospy.loginfo("Flat Figure 8 Trajectory")
            old_mode = mode
    if mode == 6:
        mode = 6
        Circle()
        if old_mode != mode:
            rospy.loginfo("Circular Trajectory")
            old_mode = mode
    if rospy.get_param('controller_status',False):
        start_time = rospy.get_time()

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('LQR_controller')

    #=======================#
    #    quad parameters    #
    #=======================#

    euler_max    = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw",".3490659")) #in radians/sec
    max_alt_rate = float(rospy.get_param("control_vz_max","1000")) #in mm/sec
    switch_time = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates, queue_size = 1, buff_size = 2**24)
        sub_joy     = rospy.Subscriber('/joy' , Joy, GetJoy)
        Datahandler()
        r.sleep()

