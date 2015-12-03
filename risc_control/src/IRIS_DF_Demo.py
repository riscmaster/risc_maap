#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	March 2015
    File name: 		IRIS_DF_Demo.py
    Organization:	RISC Lab, Utah State University
    Notes:
        This file is meant for demonstrating Differential Flatness
    as described in "Aggressive Maneuvers" by Jeff Ferrin. It is
    set up to allow toggling between trajectories using the joystick.
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

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
ThrustCap 	= .3
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
ctrl_status     = False
cases           = ['Origin','Slanted Figure Eight','Origin',\
                   'Flat Figure Eight','Origin','Circle','Origin','Toroid Knot']
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

K_way           = np.matrix([[  4.2,    0,   0,  5.8,    0,   0,   0],\
                             [    0,  4.2,   0,    0,  5.8,   0,   0],\
                             [    0,    0,   2,    0,    0,   3,   0],\
                             [    0,    0,   0,    0,    0,   0,  .3]])
K_slf8          = np.matrix([[ 3.2,    0,   0, 4.6,    0,   0,   0],\
                             [    0, 3.2,   0,    0, 4.6,   0,   0],\
                             [    0,    0,   2,    0,    0,   3,   0],\
                             [    0,    0,   0,    0,    0,   0,  .3]])
K_flf8          = np.matrix([[ 3.2,    0,   0, 4.2,    0,   0,   0],\
                             [    0, 3.2,   0,    0, 4.2,   0,   0],\
                             [    0,    0,   2,    0,    0,   3,   0],\
                             [    0,    0,   0,    0,    0,   0,  .3]])
K_crcl          = np.matrix([[ kp,    0,   0, kd,    0,   0,   0],\
                             [    0, kp,   0,    0, kd,   0,   0],\
                             [    0,    0,   2,    0,    0,   3,   0],\
                             [    0,    0,   0,    0,    0,   0,  .3]])
K_toroid        = np.matrix([[ kp,    0,   0, kd,    0,   0,   0],\
                             [    0, kp,   0,    0, kd,   0,   0],\
                             [    0,    0,   2,    0,    0,   3,   0],\
                             [    0,    0,   0,    0,    0,   0,  .3]])

    #========================#
    #    Integrator gains    #
    #========================#

ki_x = .5
ki_y = .5
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
        if mode > 8:
            mode = 1
        if mode < 1:
            mode = 8

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

def Slanted_Figure_8():

    global K_slf8,cycles
    # Trajectory Variables
    period     = 12 # seconds
    a          = 1
    b          = 0.5
    c          = 0.5
    n          = 1
    w1         = 2*PI/period
    w2         = w1/2
    w3         = w1
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_slf8)

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

def Circle():

    global K_crcl
    # Trajectory Variables
    period     = 10 # seconds
    a          = 0.8
    b          = 0.8
    c          = 0
    n          = 1
    w1         = 2*PI/period
    w2         = w1
    w3         = w1
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_crcl)

def Toroid():

    global K_toroid,mode
    # Trajectory Variables
    period     = 15 # seconds
    a          = 0.65
    b          = 0.25
    c          = 0.25
    n          = 1
    q          = 3 # number of loops
    p          = q-1
    w1         = 2*PI/period
    traj = GetToroid(period, a, b, c, n, q, p, w1,mode)
    Basic_Controller(traj, K_toroid)


def Datahandler():
    global mode, old_mode
    if mode == 1 or mode == 3 or mode == 5 or mode == 7:
        Origin()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Origin")
            old_mode = mode
    if mode == 2:
        Slanted_Figure_8()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Slanted Figure 8 Trajectory")
            old_mode = mode
    if mode == 4:
        Flat_Figure_8()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Flat Figure 8 Trajectory")
            old_mode = mode
    if mode == 6:
        mode = 6 # redefine mode to prevent control lockout (I don't know why this is necessary but it fixes it)
        Circle()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Circular Trajectory")
            old_mode = mode
    if mode == 8:
        mode = 8 # redefine mode to prevent control lockout (I don't know why this is necessary but it fixes it)
        Toroid()
        if old_mode != mode:
            global Integrator
            Integrator      = np.asmatrix(np.zeros((7,1)))
            rospy.loginfo("Toroid Knot Trajectory")
            old_mode = mode
    if rospy.get_param('controller_status',False):
        start_time = rospy.get_time()

    #=================#
    #    Get Toroid   #
    #=================#

def GetToroid(period,a,b,c,n,q,p,w1,case):

    global start_time, cases, pub_traj
    time_now    = rospy.get_time()
    t           = time_now-start_time
    theta       = w1*t
    WP          = Trajectories()
    WP.Obj      = [Trajectory()]*1

    #=======================#
    #    Polar Trajectory   #
    #=======================#
    r = a+b*cos(q*theta/p)
    z = n+c*sin(q*theta/p)
    rdot = -b*q*sin(q*theta/p)
    zdot = c*q*cos(q*theta/p)
    rddot = -b*q*q*cos(q*theta/p)/(p*p)
    zddot = -c*q*q*sin(q*theta/p)/(p*p)

    #=================#
    #    Trajectory   #
    #=================#

    traj = Trajectory()
    traj.name = cases[case-1]
    # Position
    traj.x       = r*cos(theta)
    traj.y       = r*sin(theta)
    traj.z       = z
    traj.psi     = 0
    # Velocity
    traj.xdot    = rdot*cos(theta) - r*sin(theta)
    traj.ydot    = rdot*sin(theta) + r*cos(theta)
    traj.zdot    = zdot
    traj.psidot  = 0
    # Acceleration
    traj.xddot   = rddot*cos(theta)-2*rdot*sin(theta)-r*cos(theta)
    traj.yddot   = rddot*sin(theta)+2*rdot*cos(theta)-r*sin(theta)
    traj.zddot   = zddot
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

        ctrl        = Control()
        ctrl.name   = states.Obj[0].name
        ctrl.phi    = asin(z[1,-1])/euler_max
        ctrl.theta  = -asin(z[0,-1])/euler_max
        ctrl.psi    = -u[3,-1]/max_yaw_rate

        thrust = 1-T/g
        global ThrustCap
        if thrust > ThrustCap:
            thrust = ThrustCap
        if thrust < -ThrustCap:
            thrust = -ThrustCap

        ctrl.T      = thrust
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
        sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
        sub_joy     = rospy.Subscriber('/joy' , Joy, GetJoy)
        sub_status  = rospy.Subscriber('/controller_status' , Bool, GetStatus)
        Datahandler()
        r.sleep()

