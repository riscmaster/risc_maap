#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		IRIS_DF_Controller.py
    Organization:	RISC Lab, Utah State University
    Notes:
 ======================================================'''

import roslib; roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from std_msgs.msg import Bool
from roscopter.msg import Status

    #=====================#
    #    Gain Matrices    #
    #=====================#

K = np.matrix([[ 1.8,       0,       0, 1.4,       0,       0, 0],\
               [       0, 1.8,       0,       0, 1.4,       0, 0],\
               [       0,       0, 3,       0,       0, 5, 0],\
               [       0,       0,       0,       0,       0,       0,.5]])
 

    #========================#
    #        Globals         #
    #========================#

nominal_thrust = 0 # thrust necessary to maintain hover given battery level
phi_scale   = 3.053261127645355
phi_trim    = 0.0#0.058941904209906
theta_scale = 3.815398742249453
theta_trim  = 0.0#-0.091216767651723
ctrl_status     = False
states          = Cortex()
states.Obj      = [States()]*1
traj            = Trajectories()
traj.Obj        = [Trajectory()]*1
euler_max       = 45*np.pi/180
max_yaw_rate    = .3490659 #in radians/sec
rate            = 45      # Hz
image           = 0 
start_time      = 0

    #==================#
    #    Publishers    #
    #==================#

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTraj(S):

    global traj
    traj = S

    #=========================#
    #    Get Battery Status   #
    #=========================#

def GetBatt(S):

    global nominal_thrust
    B = S.battery_remaining
    # coefficients for fourth order fit
    # determined 11 May 2015 by Spencer Maughan and Ishmaal Erekson
    c0 =  0.491674747062374
    c1 = -0.024809293286468
    c2 =  0.000662710609466
    c3 = -0.000008160593348
    c4 =  0.000000033699651
    nominal_thrust = c0+c1*B+c2*B**2+c3*B**3+c4*B**4 

    #============================#
    #    Get Controller Status   #
    #============================#

def GetStatus(S):

    global ctrl_status
    ctrl_status = S.data

    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller():
    global states, euler_max, max_yaw_rate, pub_ctrl,K,traj

    Ctrl        = Controls()
    Ctrl.Obj    = [Control()]*1
    Ctrl.header.stamp = states.header.stamp
    g = 9.80665 # average value of earth's gravitational constant m/s^2
    m = 1.282 #  IRIS mass in kg

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
        X[6] = traj.Obj[0].psi-states.Obj[0].psi*np.pi/180

        #============================================#
        #     Differential Flatness Control Input    #
        #============================================#

        # LQR input
        utilde = -K*X
        # required input
        u_r = np.asmatrix(np.zeros((4,1)))
        u = utilde+u_r-np.matrix([[0],[0],[9.81],[0]])

        #==================================#
        #     Rotate to Vehicle 1 Frame    #
        #==================================#

        psi = states.Obj[0].psi*np.pi/180
        rotZ = np.matrix([[cos(psi), sin(psi), 0],[-sin(psi), cos(psi), 0],[0, 0, 1]])
        Cart = np.matrix([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
        u[:-1] = Cart*rotZ*u[:-1]

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

        T = sqrt(u[0:3].T*u[0:3])
        u[:-1] = np.divide(u[:-1],-T)

        #==================#
        #   Set Controls   #
        #==================#

        # Controls for Ardrone
        # -phi = right... +phi = left
        # -theta = back... +theta = forward
        # -psi = right...  +psi = left
        global phi_trim,theta_trim,phi_scale,theta_scale
        phi_d = (asin(u[1,-1]))
        theta_d = (-asin(u[0,-1]))
        ctrl        = Control()
        ctrl.name   = states.Obj[0].name
        ctrl.phi    = phi_trim + phi_scale*phi_d
        ctrl.theta  = theta_trim + theta_scale*theta_d
        ctrl.psi    = -u[3,-1]/max_yaw_rate
        global nominal_thrust
        T_d = nominal_thrust+(T-g)/g
        ctrl.T      = T_d
        Ctrl.Obj[0] = ctrl
        Ctrl.header = states.header
        #rospy.loginfo("latency = %f",states.header.stamp.to_sec()-rospy.get_time())
        pub_ctrl.publish(Ctrl)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('IRIS_DF_Controller')

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates, queue_size=1, buff_size=2**24)
        sub_traj  = rospy.Subscriber('/trajectory' , Trajectories, GetTraj, queue_size=1, buff_size=2**24)
        sub_Batt    = rospy.Subscriber('/apm/status' , Status, GetBatt)
        sub_status  = rospy.Subscriber('/controller_status' , Bool, GetStatus)
        Basic_Controller()
        r.sleep()

