#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	March 2015
    File name: 		IRIS_origin_hold.py
    Organization:	RISC Lab, Utah State University
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

    #========================#
    #        Globals         #
    #========================#

# set using experimental data on 7 May 2015 by Spencer Maughan and Ishmaal Erekson
t1 =  0.012126261080744
t2 = -0.000237741931107
t3 =  0.000001025182900
# set manually using values on transmitter
phi_trim        = -.05
theta_trim      = -.14

Batt            = 70
ctrl_status     = False
states          = Cortex()
states.Obj      = [States()]*1
euler_max       = 45*np.pi/180
max_yaw_rate    = .3490659 #in radians/sec
rate            = 45      # Hz
start_time      = 0

    #==================#
    #    Publishers    #
    #==================#

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)
pub_traj        = rospy.Publisher('/trajectory', Trajectories, queue_size = 1)

    #===================#
    #    Gain Matrix    #
    #===================#

K  = np.matrix([[    4,    0,    0,  6.1,    0,    0, 0],\
                [    0,    4.8,    0,    0,  7.1,    0, 0],\
                [    0,    0,   2,    0,    0, 3.3, 0],\
                [    0,    0,    0,    0,    0,    0, .5]])

    #========================#
    #    Integrator gains    #
    #========================#

ki_x = .5
ki_y = 1
ki_z = .5
ki_xdot = 0
ki_ydot = 0
ki_zdot = 0
ki_psi = 0

K_Int           = np.matrix([[ ki_x,    0,      0, ki_xdot,       0,      0,       0],\
                             [    0, ki_y,      0,       0, ki_ydot,      0,       0],\
                             [    0,    0,   ki_z,       0,       0,ki_zdot,       0],\
                             [    0,    0,      0,       0,       0,      0,  ki_psi]])

Integrator      = np.asmatrix(np.zeros((7,1)))

    #============================#
    #    Integrator Threshold    #
    #============================#

x_int_thresh    = 1
y_int_thresh    = 1
z_int_thresh    = 1
xdot_int_thresh = 1
ydot_int_thresh = 1
zdot_int_thresh = 1
psi_int_thresh  = 0
Int_thresh      = np.matrix([[x_int_thresh],[y_int_thresh],[z_int_thresh],\
                             [xdot_int_thresh],[ydot_int_thresh],[zdot_int_thresh],\
                             [psi_int_thresh]])

    #=====================#
    #    Integrator Cap   #
    #=====================#
def IntegratorCap(I):
    global Int_thresh
    good_terms = np.multiply(I,abs(I)<Int_thresh) # leave as is and set others to zero
    bad_terms  = abs(I)>Int_thresh # set bad terms to 1 and others to 0
    Int = good_terms + np.multiply(np.sign(I),np.multiply(bad_terms,Int_thresh))
    return Int

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #=========================#
    #    Get Battery Status   #
    #=========================#

def GetBatt(S):

    global Batt
    Batt = S.battery_remaining


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
    global states, euler_max, max_yaw_rate, pub_ctrl,K

    Ctrl        = Controls()
    Ctrl.Obj = [Control()]*1
    Ctrl.header.stamp = states.header.stamp
    g = 9.80665 # average value of earth's gravitational constant m/s^2
    m = 1.282 #  IRIS mass in kg

        #===================================#
        #    Get State Trajectory Errors    #
        #===================================#

    if states.Obj[0].visible:
        X = np.asmatrix(np.zeros((7,1)))
        X[0] = -states.Obj[0].x
        X[1] = -states.Obj[0].y
        X[2] = 1-states.Obj[0].z
        X[3] = -states.Obj[0].u
        X[4] = -states.Obj[0].v
        X[5] = -states.Obj[0].w
        X[6] = -states.Obj[0].psi*np.pi/180


        #======================================#
        #    Only Integrate When Autonomous    #
        #======================================#

        global Integrator,ctrl_status

        if ctrl_status:

        #=======================#
        #    Integrator Term    #
        #=======================#


            Integrator = Integrator + np.divide(X,rate)

        #======================#
        #    Integrator Cap    #
        #======================#

            global Int_thresh

            Integrator = IntegratorCap(Integrator)
        elif not ctrl_status:
            Integrator = np.asmatrix(np.zeros((7,1)))

        #============================================#
        #     Differential Flatness Control Input    #
        #============================================#

        global K_Int
        # LQR input
        utilde = -K*X - K_Int*Integrator
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
        global phi_trim,theta_trim
        ctrl        = Control()
        ctrl.name   = states.Obj[0].name
        ctrl.phi    = (asin(u[1,-1]))/euler_max+phi_trim
        ctrl.theta  = (-asin(u[0,-1]))/euler_max+theta_trim
        ctrl.psi    = -u[3,-1]/max_yaw_rate
        global Batt,t1,t2,t3
        Batt2 = Batt*Batt
        Batt3 = Batt*Batt*Batt
        T_d = t1*Batt+t2*Batt2+t3*Batt3+(T-g)/g
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
    rospy.init_node('IRIS_Origin_Hold')

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates, queue_size=1, buff_size=2**24)
        sub_Batt    = rospy.Subscriber('/apm/status' , Status, GetBatt)
        sub_status  = rospy.Subscriber('/controller_status' , Bool, GetStatus)
        Basic_Controller()
        r.sleep()

