#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	March 2015
    File name: 		IRIS_origin_hold_tuner.py
    Organization:	RISC Lab, Utah State University
    Notes:
	This file is meant for realtime tuning of waypoint PID gains.
 ======================================================'''

import roslib; roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time
import cv2
import rospkg

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from std_msgs.msg import Bool
from roscopter.msg import Status

    #==========================#
    #    Trackbar Variables    #
    #==========================#

# X Position
kpx = 300/2
kix = 100/2
kdx = 500/2

# Y Position
kpy = 300/2
kiy = 100/2
kdy = 500/2

# Z Position
kpz = 200
kiz = 54
kdz = 193


    #========================#
    #        Globals         #
    #========================#

nominal_thrust = 0 # thrust necessary to maintain hover given battery level
phi_scale   = 3.053261127645355
phi_trim    = 0.058941904209906
theta_scale = 3.815398742249453
theta_trim  = -0.091216767651723
ctrl_status     = False
states          = Cortex()
states.Obj      = [States()]*1
euler_max       = 45*np.pi/180
max_yaw_rate    = .3490659 #in radians/sec
rate            = 45      # Hz
image           = 0 
start_time      = 0

    #==================#
    #    Publishers    #
    #==================#

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)
pub_traj        = rospy.Publisher('/trajectory', Trajectories, queue_size = 1)

    #============================#
    #    Integrator Threshold    #
    #============================#

Integrator      = np.asmatrix(np.zeros((7,1)))
x_int_thresh    = 8
y_int_thresh    = 8
z_int_thresh    = 8
xdot_int_thresh = 1
ydot_int_thresh = 1
zdot_int_thresh = 1
psi_int_thresh  = 0
Int_thresh      = np.matrix([   [x_int_thresh],   [y_int_thresh],   [z_int_thresh],\
                             [xdot_int_thresh],[ydot_int_thresh],[zdot_int_thresh],\
                              [psi_int_thresh]])

    #=========================#
    #    Trackbar Functions   #
    #=========================#

# X Position
def Fkpx(x):
    global kpx
    kpx = x
def Fkix(x):
    global kix
    kix = x
def Fkdx(x):
    global kdx
    kdx = x

# Y Position
def Fkpy(x):
    global kpy
    kpy = x
def Fkiy(x):
    global kiy
    kiy = x
def Fkdy(x):
    global kdy
    kdy = x

# Z Position
def Fkpz(x):
    global kpz
    kpz = x
def Fkiz(x):
    global kiz
    kiz = x
def Fkdz(x):
    global kdz
    kdz = x

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

        #===================#
        #    Gain Matrix    #
        #===================#
        
        K = np.matrix([[ kpx/100,       0,       0, kdx/100,       0,       0, 0],\
                       [       0, kpy/100,       0,       0, kdy/100,       0, 0],\
                       [       0,       0, kpz/100,       0,       0, kdz/100, 0],\
                       [       0,       0,       0,       0,       0,       0,.5]])
        
        #========================#
        #    Integrator gains    #
        #========================#
        
        K_Int = np.matrix([[ kix/100,    0,   0, 0, 0, 0, 0],\
                           [    0, kiy/100,   0, 0, 0, 0, 0],\
                           [    0,   0, kiz/100, 0, 0, 0, 0],\
                           [    0,   0,   0, 0, 0, 0, 0]])

        #============================================#
        #     Differential Flatness Control Input    #
        #============================================#

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
        cv2.imshow("gains", image)
        cv2.waitKey(1)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('IRIS_Origin_Hold')

    #===============================#
    #    Get Image Using RosPack    #
    #===============================#

    rospack = rospkg.RosPack()
    path = rospack.get_path('risc_control')
    image = cv2.imread(path+'/mario.jpg')
    cv2.resize(image,(321,123))
    cv2.namedWindow("gains")

    #========================#
    #    Create Trackbars    #
    #========================#

    cv2.createTrackbar("kpx", "gains", kpx, 1000, Fkpx)
    cv2.createTrackbar("kix", "gains", kix, 1000, Fkix)
    cv2.createTrackbar("kdx", "gains", kdx, 1000, Fkdx)
    cv2.createTrackbar("kpy", "gains", kpy, 1000, Fkpy)
    cv2.createTrackbar("kiy", "gains", kiy, 1000, Fkiy)
    cv2.createTrackbar("kdy", "gains", kdy, 1000, Fkdy)
    cv2.createTrackbar("kpz", "gains", kpz, 1000, Fkpz)
    cv2.createTrackbar("kiz", "gains", kiz, 1000, Fkiz)
    cv2.createTrackbar("kdz", "gains", kdz, 1000, Fkdz)

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

