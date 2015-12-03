#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	February 2015
    File name: 		Pendulum_Control.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import cv2
import rospkg
import numpy as np
import scipy.linalg as la

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
states      = Cortex()
states.Obj  = [States()]*0
traj        = Trajectories()
traj.Obj    = [Trajectory()]*1
rate        = 200 # Hz
time_past   = 0
pitch_kg    = 0
roll_kg     = 0
pub         = rospy.Publisher('/controls', Controls, queue_size = 70)

    #==============================#
    #        Gain Matrices         #
    #==============================#

sat= .1
kp = 10#10
kd = 7#7
# K_quad = np.matrix([[kp,     0,   kd,      0,   0, 0, 0,   0,   0, 0, 0],\
#                     [   0,    kp,      0,    kd,   0, 0, 0,   0,   0, 0, 0],\
#                     [   0,     0,      0,      0,   0, 0, 0,  .01,   0, 0, 0],\
#                     [   0,     0,      0,      0,   0, 0, 0,   0,  .01, 0, 0],\
#                     [   0,     0,      0,      0,   0, 0, 0,   0,   0, 0, 0],\
#                     [   0,     0,      0,      0,   0, 0, 0,   0,   0, 0, 0]])
#==============================#
#   From Simulink Simulation   #
#==============================#

K_quad = np.matrix([[14.7479,       0, 6.0824,      0,   0, 0, 0,      0,      0,      0, 0],\
                    [      0, 12.5831,      0, 5.6421,   0, 0, 0,      0,      0,      0, 0],\
                    [      0,       0,      0,      0,   0, 0, 0, 0.0093,      0,      0, 0],\
                    [      0,       0,      0,      0,   0, 0, 0,      0, 0.0098,      0, 0],\
                    [      0,       0,      0,      0,   0, 0, 0,      0,      0, 0.0095, 0],\
                    [      0,       0,      0,      0,   0, 0, 0,      0,      0,      0, 0]])

#K_pen  = np.matrix([[    .005,     0,    0,     1 ,   0,      0],\
#                    [      0,   .005,    0,      0,   1,      0],\
#                    [      0,     0,  1.2,      0,   0,    2.2]])
# K_pen  = np.matrix([[      .1,     0,    0,     1,   0,      0],\
#                     [      0,     .1,    0,      0,   1,      0],\
#                     [      0,     0,  1.2,      0,   0,    2.2]])

#==============================#
#   From Simulink Simulation   #
#==============================#
K_pen  = np.matrix([[0.1690,      0,      0, 0.5875,        0,      0],\
                    [     0, 0.1690,      0,      0,   0.5845,      0],\
                    [     0,      0, 4.1833,      0,        0, 2.9777]])

    #======================#
    #    Adjust Radians    #
    #======================#

def pi2pi(angle):
     if abs(angle)>PI/2:
           if angle>0:
                angle = angle-PI
           else:
                angle = angle+PI
     return angle

    #====================#
    #    sgn function    #
    #====================#

def sgn(x):
     y = abs(x)/x
     return y


    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTrajectory(W):

    global traj
    traj = W
    Datahandler()

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler():
    global PI,K,traj,states,time_past,pub,image
    status = rospy.get_param('controller_status',True)
    rate   = 1/(rospy.get_time()-time_past)

    #====================================#
    #    quad parameters and constants   #
    #====================================#

    euler_max    = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw",".3490659")) #in radians/sec
    max_alt_rate = float(rospy.get_param("control_vz_max","1000")) #in mm/sec

    g = 9.81
    m = 1.378 # IRIS Mass   .450 # ARDrone mass

    #==================#
    #    Get States    #
    #==================#

    bodies      = len(states.Obj)
    X_pen       = np.asmatrix(np.zeros((6,1)))
    pen_states  = np.asmatrix(np.zeros((6,1)))
    quad_states = np.asmatrix(np.zeros((7,1)))
    pen_index   = 0
    quad_index  = 0

    for i in range(bodies):
        if states.Obj[i].name == 'Pendulum_tip':
            pen_index = i
            # Pendulum States
            pen_states[0] = states.Obj[i].x
            pen_states[1] = states.Obj[i].y
            pen_states[2] = states.Obj[i].z
            pen_states[3] = states.Obj[i].u
            pen_states[4] = states.Obj[i].v
            pen_states[5] = states.Obj[i].w

            # World Frame trajectory Errors
            X_pen[0] = traj.Obj[0].x-states.Obj[i].x
            X_pen[1] = traj.Obj[0].y-states.Obj[i].y
            X_pen[2] = traj.Obj[0].z-states.Obj[i].z
            X_pen[3] = traj.Obj[0].xdot-states.Obj[i].u
            X_pen[4] = traj.Obj[0].ydot-states.Obj[i].v
            X_pen[5] = traj.Obj[0].zdot-states.Obj[i].w

        if states.Obj[i].name == 'Condor' or states.Obj[i].name == 'Kahn' or states.Obj[i].name == "Simulated AR.Drone":

            quad_index = i
            # Quadrotor States of Interest
            quad_states[0] = states.Obj[i].x
            quad_states[1] = states.Obj[i].y
            quad_states[2] = states.Obj[i].z
            quad_states[3] = states.Obj[i].u
            quad_states[4] = states.Obj[i].v
            quad_states[5] = states.Obj[i].w
            quad_states[6] = states.Obj[i].psi

    # Relative Pendulum States
    #rospy.loginfo('x_v = %f, x_p = %f',pen_states[0],quad_states[0])
    r    = pen_states[0] - quad_states[0]
    s    = pen_states[1] - quad_states[1]
    rdot = pen_states[3] - quad_states[3]
    sdot = pen_states[4] - quad_states[4]
    L = np.linalg.norm(pen_states[0:3,0]-quad_states[0:3,0])

    #==========================#
    #    Get Desired States    #
    #==========================#

    # LQR input
    utilde = K_pen*X_pen
    # required input
    u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot]])
    u = utilde-u_r
    # map to pen2quad states desired
    rd = u[0,0]
    if abs(rd) > .1*L:
        rd = .1*L*sgn(rd)
    sd = u[1,0]
    if abs(sd) > .1*L:
        sd = .1*L*sgn(sd)
    zddotd = u[2,0]

    #==================#
    #    Get Errors    #
    #==================#
    #rospy.loginfo('%i',quad_index)

    X = np.asmatrix(np.zeros((11,1)))
    X[0]  = rd-r
    X[1]  = sd-s
    X[2]  = -rdot
    X[3]  = -sdot
    X[4]  = -states.Obj[quad_index].x
    X[5]  = -states.Obj[quad_index].y
    X[6]  = -states.Obj[quad_index].z
    X[7]  = -states.Obj[quad_index].u
    X[8]  = -states.Obj[quad_index].v
    X[9]  = -states.Obj[quad_index].w
    X[10] = pi2pi(traj.Obj[0].psi)-states.Obj[quad_index].psi*PI/180

    #============================================#
    #     Differential Flatness Control Input    #
    #============================================#

    # LQR input
    utilde    = K_quad*X
    u2        = np.asmatrix(np.zeros((4,1)))
    u2[0:2,0] = utilde[2:4,0]-utilde[0:2,0]
    u2[2,0]   = g-zddotd
    u2[3,0]   = utilde[5,0]

    # required input
    u = -u2

       #=============================#
       #     Rotate to Body Frame    #
       #=============================#

    psi  = quad_states[6,0]*PI/180
    rotZ = np.matrix([[cos(psi), -sin(psi), 0],[sin(psi), cos(psi), 0],[0, 0, 1]])
#    rospy.loginfo('psi = %f', psi)

    u[:-1] = rotZ*u[:-1]

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

    T = sqrt(u[0,-1]*u[0,-1]+u[1,-1]*u[1,-1]+u[2,-1]*u[2,-1])
    u[:-1] = np.divide(u[:-1],T)

    #===============================#
    #    Set up Controls Message    #
    #===============================#

    Ctrl              = Controls()
    Ctrl.Obj          = [Control()]*1
    Ctrl.header.stamp = states.header.stamp
    C1                = Control()

    #==================#
    #   Set Controls   #
    #==================#

    # Controls for Ardrone
    # -phi = right... +phi = left
    # -theta = back... +theta = forward
    # -psi = right...  +psi = left

    global pitch_kg, roll_kg
    C1.name  = states.Obj[quad_index].name
    C1.phi   = -asin(u[1,-1])/(euler_max*roll_kg)
    C1.theta = -asin(u[0,-1])/(euler_max*pitch_kg)
    C1.psi   = u[3,-1]/max_yaw_rate
    C1.T     = g*m#T*m
    Ctrl.Obj[0] = C1
    pub.publish(Ctrl)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Pendulum_Controller')
    time_past = rospy.get_time()
    roll_kg       = float ( rospy.get_param("~roll_kg","1.1714") )
    pitch_kg      = float ( rospy.get_param("~pitch_kg","1.10714") )

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
    sub  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
    rospy.spin()

