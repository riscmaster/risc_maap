#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		IRIS_exp.py
    Organization:	RISC Lab, Utah State University
    Notes:
        This file is meant for gathering data for throttle,
    Pitch and Roll model estimation.
    ======================================================'''

    #================================#
    #    Libraries/modules Needed    #
    #================================#

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
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from roscopter.msg import Status

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
states          = Cortex()
states.Obj      = [States()]*1
BatteryPercent  = 70
Thrust_Sat      = .3
euler_max       = 0.349066 #in radians
max_yaw_rate    = .3490659 #in radians/sec
max_alt_rate    = 1000     # in mm/sec
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

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)
pub_traj        = rospy.Publisher('/trajectory', Trajectories, queue_size = 1)

    #=======================================#
    #    Estimated Optimal Gain Matrices    #
    #=======================================#
kp = 1.2
kd = 2.6

K_way           = np.matrix([[  2.1,    0,   0,  2.9,    0,   0,   0],\
                             [    0,  2.1,   0,    0,  2.9,   0,   0],\
                             [    0,    0,   2,    0,    0,   3,   0],\
                             [    0,    0,   0,    0,    0,   0,  .3]])

    #========================#
    #    Integrator gains    #
    #========================#

ki_x = 0
ki_y = 0
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

x_int_thresh    = 2
y_int_thresh    = 2
z_int_thresh    = 2.5
xdot_int_thresh = 1
ydot_int_thresh = 1
zdot_int_thresh = 2.5
psi_int_thresh  = 0
Int_thresh      = np.matrix([[x_int_thresh],[y_int_thresh],[z_int_thresh],\
                             [xdot_int_thresh],[ydot_int_thresh],[zdot_int_thresh],\
                             [psi_int_thresh]])

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

    #=========================#
    #    Battery Percentage   #
    #=========================#
def GetBatt(I):
    global BatteryPercent 
    BatteryPercent = I.battery_remaining
    
    #=====================#
    #    Integrator Cap   #
    #=====================#
def IntegratorCap(I):
    global Int_thresh
    good_terms = np.multiply(I,abs(I)<Int_thresh) # leave as is and set others to zero
    bad_terms  = abs(I)>Int_thresh # set bad terms to 1 and others to 0
    Int = good_terms + np.multiply(np.sign(I),np.multiply(bad_terms,Int_thresh))
    return Int

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


    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #==========================#
    #   Various Trajectories   #
    #==========================#

def Origin():

    global K_way
    traj = GetTrajectory(10, 0,0,0,1,0,0,0,mode)
    Basic_Controller(traj, K_way)

def QuadrantI():

    global K_way
    traj = GetWaypoint(.75,.75,1,mode)
    Basic_Controller(traj, K_way)

def QuadrantII():

    global K_way
    traj = GetWaypoint(-.75,.75,1,mode)
    Basic_Controller(traj, K_way)

def QuadrantIII():

    global K_way
    traj = GetWaypoint(-.75,-.75,1,mode)
    Basic_Controller(traj, K_way)

def QuadrantIV():

    global K_way
    traj = GetWaypoint(.75,-.75,1,mode)
    Basic_Controller(traj, K_way)


def Datahandler():
    global mode, old_mode
    if mode == 1:
        Origin()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Origin")
            old_mode = mode
    if mode == 2 or mode == 6:
        QuadrantI()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Quadrant I")
            old_mode = mode
    if mode == 3:
        QuadrantII()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Quadrant II")
            old_mode = mode
    if mode == 4:
        mode = 4 # redefine mode to prevent control lockout (I don't know why this is necessary but it fixes it)
        QuadrantIII()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Quadrant III")
            old_mode = mode
    if mode == 5:
        mode = 5 # redefine mode to prevent control lockout (I don't know why this is necessary but it fixes it)
        QuadrantIV()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
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


    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller(traj,K):
    global states,PI, euler_max, max_yaw_rate, max_alt_rate, pub_ctrl

    g        = 9.80665
    m        = 1.282 # IRIS Mass Estimate (not used since thrust is given as a percentage)
    Ctrl     = Controls()
    Ctrl.Obj = [Control()]*1

    #===================================#
    #    Get State Trajectory Errors    #
    #===================================#

    if states.Obj[0].visible:
        X = np.asmatrix(np.zeros((7,1)))
        X[0] = traj.Obj[0].x    - states.Obj[0].x
        X[1] = traj.Obj[0].y    - states.Obj[0].y
        X[2] = traj.Obj[0].z    - states.Obj[0].z
        X[3] = traj.Obj[0].xdot - states.Obj[0].u
        X[4] = traj.Obj[0].ydot - states.Obj[0].v
        X[5] = traj.Obj[0].zdot - states.Obj[0].w
        X[6] = traj.Obj[0].psi  - states.Obj[0].psi*np.pi/180

            #======================================#
            #    Only Integrate When Autonomous    #
            #======================================#

        global Integrator
        if ctrl_status:
            #=======================#
            #    Integrator Term    #
            #=======================#

            global K_Int

            Integrator = Integrator + np.divide(X,rate)

            #======================#
            #    Integrator Cap    #
            #======================#

            global Int_thresh

            Integrator = IntegratorCap(Integrator)
        elif not ctrl_status:
            Integrator = np.asmatrix(np.zeros((7,1)))
        print states.Obj[0]

        #====================================#
        #         Differential Flatness      #
        #     Input Acceleration Desired     #
        #          in Vehicle Frame          #
        #====================================#

        # LQR input
        utilde = -K*X - K_Int*Integrator
        # required input
        u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot],[traj.Obj[0].psiddot]])
        u = utilde+u_r+np.matrix([[0],[0],[9.81],[0]])

        #==================================#
        #     Rotate to Vehicle 1 Frame    #
        #==================================#

        psi = states.Obj[0].psi*np.pi/180 # in radians

        rotZ = np.matrix([[cos(psi), sin(psi), 0],[-sin(psi), cos(psi), 0],[0, 0, 1]])
        Cart = np.matrix([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
        u[:-1] = Cart*rotZ*u[:-1]

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

        T = sqrt(u[0:3].T*u[0:3])
        z = np.divide(u[:-1],-T)

        #==================#
        #   Set Controls   #
        #==================#
        global BatteryPercent, Thrust_Sat
        phi_d = asin(z[1,-1])
        theta_d = -asin(z[0,-1])
        T_map = T-1

        ctrl        = Control()
        ctrl.name   = states.Obj[0].name
        ctrl.phi    = phi_d/euler_max 
        ctrl.theta  = theta_d/euler_max
        ctrl.psi    = -u[3,-1]/max_yaw_rate
        ctrl.T      = 1-T_map/g
        Ctrl.Obj[0] = ctrl
        Ctrl.header = states.header
        pub_ctrl.publish(Ctrl)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('DF_Demo')

    #=======================#
    #    quad parameters    #
    #=======================#

    euler_max    = float(rospy.get_param("euler_angle_max","0.78537")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw","0.3490659")) #in radians/sec
    #ki_x         = float(rospy.get_param("ki_x","0"))
    #ki_y         = float(rospy.get_param("ki_y","0"))
    #ki_z         = float(rospy.get_param("ki_z","0"))
    #ki_xdot      = float(rospy.get_param("ki_xdot","0"))
    #ki_ydot      = float(rospy.get_param("ki_ydot","0"))
    #ki_zdot      = float(rospy.get_param("ki_zdot","0"))
    #ki_psi       = float(rospy.get_param("ki_psi","0"))
    #ThrustCap    = float(rospy.get_param("ThrustCap",".4"))

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        sub2 = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
        sub3 = rospy.Subscriber('/joy' , Joy, GetJoy)
        sub4 = rospy.Subscriber('/controller_status' , Bool, GetStatus)
        sub5 = rospy.Subscriber('/apm/status' , Status, GetBatt)
        Datahandler()
        r.sleep()

