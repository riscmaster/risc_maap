#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	July 2015
    File name: 		fixed_wing_like_flight.py
    Organization:	RISC Lab, Utah State University
    Notes:
         This file is intended to control a quadrotor in  
    such a way that it behaves similar to a fixed-wing aircraft 
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import scipy.linalg as la
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *

    #========================#
    #        Globals         #
    #========================#

states          = Cortex()
states.Obj      = [States()]*1
traj            = Trajectories()
traj.Obj        = [Trajectory()]*1
euler_max       = 0.349066 #in radians
max_yaw_rate    = .3490659 #in radians/sec
max_alt_rate    = 1000     # in mm/sec
rate            = 50      # Hz

    #==================#
    #    Publishers    #
    #==================#

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)

    #===================#
    #    Gain Matrix    #
    #===================#

kp = 1
kd = 1.6
percent_roll = .1

    #===================================#
    #    Radians between + or - pi/2    #
    #===================================#

def pi2pi(angle):
     while abs(angle) > np.pi:
         angle = angle - 2*np.pi*abs(angle)/angle
     return angle

    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTrajectory(X):
    global traj
    traj  = X

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller():
    global states, euler_max, max_yaw_rate, max_alt_rate, pub_ctrl, traj, K

    Ctrl        = Controls()
    # Initiate Control Messages
    bodies = len(states.Obj)
    Ctrl.Obj = [Control()]*bodies
    Ctrl.header.stamp = states.header.stamp
    g = 9.81
    m = .450 # ARDrone mass

    for i in range(bodies):

            #===================================#
            #    Get State Trajectory Errors    #
            #===================================#

        if states.Obj[i].visible and len(traj.Obj)>=len(states.Obj):
            X = np.asmatrix(np.zeros((7,1)))
            X[0] = traj.Obj[i].x-states.Obj[i].x
            X[1] = traj.Obj[i].y-states.Obj[i].y
            X[2] = traj.Obj[i].z-states.Obj[i].z
            X[3] = traj.Obj[i].xdot-states.Obj[i].u
            X[4] = traj.Obj[i].ydot-states.Obj[i].v
            X[5] = traj.Obj[i].zdot-states.Obj[i].w
            normal_calc = pi2pi(traj.Obj[i].psi)-states.Obj[i].psi*np.pi/180
            calc_p2pi   = pi2pi(traj.Obj[i].psi)-states.Obj[i].psi*np.pi/180+2*np.pi
            min_dist    = min([abs(normal_calc),abs(calc_p2pi)])
            direction   = ((min_dist==abs(normal_calc))*normal_calc + (min_dist==abs(calc_p2pi))*calc_p2pi)/min_dist
            X[6] = min_dist*direction

            #====================#
            #     Error Terms    #
            #====================#

            xy_err  = sqrt(X[0:2].T*X[0:2])
            xy_vel_err = sqrt(X[3:5].T*X[3:5])
            z_err   = X[2,-1]
            z_vel_err  = X[5,-1]
            psi_err = X[6,-1]

            #==================#
            #    Controller    #
            #==================#

            A = np.asmatrix(np.array([[traj.Obj[i].xddot],[traj.Obj[i].yddot]]))

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
            ctrl.phi    = asin(-u[1,-1])/euler_max
            ctrl.theta  = atan2(u[0,-1],u[2,-1])/euler_max
            ctrl.psi    = (traj.Obj[i].psidot + kp*psi_err)/max_yaw_rate
            ctrl.T      = T*m
            Ctrl.Obj[i] = ctrl
        Ctrl.header = states.header
        #rospy.loginfo("latency = %f",states.header.stamp.to_sec()-rospy.get_time())
        pub_ctrl.publish(Ctrl)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Differential_Flatness')

    #=======================#
    #    quad parameters    #
    #=======================#

    euler_max    = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw",".3490659")) #in radians/sec
    max_alt_rate = float(rospy.get_param("control_vz_max","1000")) #in mm/sec

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        sub_cortex   = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
        sub_traj     = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
        Basic_Controller()
        r.sleep()

