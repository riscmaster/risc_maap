#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		IRIS_IP_DF.py
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
from roscopter.msg import Status

    #========================#
    #        Globals         #
    #========================#

states         = Cortex()
states.Obj     = [States()]*0
traj           = Trajectories()
traj.Obj       = [Trajectory()]*1
nominal_thrust = -.1
rate           = 45 # Hz
time_past      = 0
pub            = rospy.Publisher('/ip_controls', Controls, queue_size = 70)

    #==============================#
    #        Gain Matrices         #
    #==============================#

#==============================#
#   From Simulink Simulation   #
#==============================#

K_quad = np.matrix([[14.7479,       0, 6.0824,      0,   0, 0, 0,      0,      0,      0, 0],\
                    [      0, 12.5831,      0, 5.6421,   0, 0, 0,      0,      0,      0, 0],\
                    [      0,       0,      0,      0,   0, 0, 0, 0.0093,      0,      0, 0],\
                    [      0,       0,      0,      0,   0, 0, 0,      0, 0.0098,      0, 0],\
                    [      0,       0,      0,      0,   0, 0, 0,      0,      0, 0.0095, 0],\
                    [      0,       0,      0,      0,   0, 0, 0,      0,      0,      0, 0]])

K_pen  = np.matrix([[0.1690,      0,      0, 0.5875,        0,      0],\
                    [     0, 0.1690,      0,      0,   0.5845,      0],\
                    [     0,      0, 4.1833,      0,        0, 2.9777]])

    #======================#
    #    Adjust Radians    #
    #======================#

def pi2pi(angle):
     if abs(angle)>np.pi/2:
           if angle>0:
                angle = angle-np.pi
           else:
                angle = angle+np.pi
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

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #========================#
    #    Get Battery Level   #
    #========================#

def GetBatt(S):

    global nominal_thrust
    B = S.battery_remaining
    c0 =  0.491674747062374
    c1 = -0.024809293286468
    c2 =  0.000662710609466
    c3 = -0.000008160593348
    c4 =  0.000000033699651
    nominal_thrust = c0+c1*B+c2*B**2+c3*B**3+c4*B**4


    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler():
    global PI,K,traj,states,time_past,pub,image
    status = rospy.get_param('controller_status',True)
    rate   = 1/(rospy.get_time()-time_past)

    g = 9.81

    #==================#
    #    Get States    #
    #==================#

    bodies      = len(states.Obj)
    if bodies != 0:
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

            if states.Obj[i].name == 'Condor' or states.Obj[i].name == 'IRIS'  or states.Obj[i].name == 'Kahn' or states.Obj[i].name == "Simulated AR.Drone":

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
        if L <1.2:
    
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
            X[10] = pi2pi(traj.Obj[0].psi)-states.Obj[quad_index].psi*np.pi/180
    
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
    
            psi  = quad_states[6,0]*np.pi/180
            rotZ = np.matrix([[cos(psi), -sin(psi), 0],[sin(psi), cos(psi), 0],[0, 0, 1]])
    #        rospy.loginfo('psi = %f', psi)
    
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
            global nominal_thrust
            C1.name  = states.Obj[quad_index].name
            C1.phi   = asin(u[1,-1])*6/np.pi
            C1.theta = asin(u[0,-1])*6/np.pi
            C1.psi   = u[3,-1]*6/np.pi
            C1.T     = nominal_thrust+(T-g)/g
            Ctrl.Obj[0] = C1
            pub.publish(Ctrl)
        else:

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
            C1.name  = states.Obj[quad_index].name
            C1.phi   = 0
            C1.theta = 0
            C1.psi   = 0
            C1.T     = nominal_thrust
            Ctrl.Obj[0] = C1
            pub.publish(Ctrl)


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Pendulum_Controller')
    time_past = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(45)
    while not rospy.is_shutdown():
        sub  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates,queue_size = 1, buff_size = 2**24)
        blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory,queue_size = 1, buff_size = 2**24)
        glub = rospy.Subscriber('/apm/status' , Status, GetBatt,queue_size = 1, buff_size = 2**24)
        Datahandler()
        r.sleep()

