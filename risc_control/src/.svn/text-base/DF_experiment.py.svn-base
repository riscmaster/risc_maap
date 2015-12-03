#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	November 2014
    File name: 		DF_experiment.py
    Organization:	RISC Lab, Utah State University
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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
navdata         = Navdata()
states          = Cortex()
states.Obj      = [States()]*0
traj            = Trajectories()
traj.Obj        = [Trajectory()]*1
euler_max       = 0.349066 #in radians
max_yaw_rate    = .3490659 #in radians/sec
max_alt_rate    = 1000     # in mm/sec
rate            = 200      # Hz
cycles          = 2      # number of cycles for
time_past       = 0
image 		= 0
start_time      = 0
mode            = 0 # mode of 7 listed under cases
cases           = ['Takeoff','Fly to Origin','Slanted Figure Eight',\
                   'Flat Figure Eight','Circle', 'Fly to Origin', 'Land']
    #==================#
    #    Publishers    #
    #==================#

pubTakeoff      = rospy.Publisher('/ardrone/takeoff',Empty, queue_size = 5)
pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 200)
pub_traj        = rospy.Publisher('/trajectory', Trajectories, queue_size = 200)

    #=====================#
    #    Gain Matrices    #
    #=====================#

K_way           = np.matrix([[ 4.15,    0, 0, 3.15,    0,      0, 0],\
                             [    0, 4.15, 0,    0, 3.15,      0, 0],\
                             [    0,    0, 3,    0,    0, 0.0001, 0],\
                             [    0,    0, 0,    0,    0,      0, 1]])
K_slf8          = np.matrix([[ 4.15,    0, 0, 3.15,    0,      0, 0],\
                             [    0, 4.15, 0,    0, 3.15,      0, 0],\
                             [    0,    0, 3,    0,    0, 0.0001, 0],\
                             [    0,    0, 0,    0,    0,      0, 1]])
K_flf8          = np.matrix([[ 4.15,    0, 0, 3.15,    0,      0, 0],\
                             [    0, 4.15, 0,    0, 3.15,      0, 0],\
                             [    0,    0, 3,    0,    0, 0.0001, 0],\
                             [    0,    0, 0,    0,    0,      0, 1]])
K_crcl          = np.matrix([[ 4.15,    0, 0, 3.15,    0,      0, 0],\
                             [    0, 4.15, 0,    0, 3.15,      0, 0],\
                             [    0,    0, 3,    0,    0, 0.0001, 0],\
                             [    0,    0, 0,    0,    0,      0, 1]])

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

    #======================#
    #    Signal Complete   #
    #======================#

def Complete():
    rospy.loginfo("Experiment Complete")

    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTrajectory(period,a,b,c,n,w1,w2,w3,case):
    global start_time, time_past, cases
    time_now    = rospy.get_time()
    t           = time_now-start_time
    WP          = Trajectories()
    WP.Obj      = [Trajectory()]*1

    #=================#
    #    Trajectory   #
    #=================#

    traj = Trajectory()
    traj.name    = cases[case]
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
    traj.zddot   = -c*w3*w3*sin(w3*t)+9.81
    traj.psiddot = 0

    WP.Obj = [traj]
    return WP

    #=======================#
    #    NaN returns zero   #
    #=======================#

def IfNaN(X):

    if X != X:
         Y = 0
    else:
         Y = X
    return Y

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

    #==================#
    #    Get Navdata   #
    #==================#

def GetNavdata(S):

    global navdata
    navdata = S

    #=============#
    #   Takeoff   #
    #=============#

def Takeoff():

    global navdata, mode
    if navdata.state == 2:
        pubTakeoff.publish(Empty())
    traj = GetTrajectory(10, 0,0,0,1,0,0,0,mode)
    if navdata.state == 3:
        mode = 1

    #==========#
    #   Land   #
    #==========#

def Land():

    global navdata, mode
    if navdata.state == 3:
        pubLand.publish(Empty())
    traj = GetTrajectory(10, 0,0,0,1,0,0,0,mode)
    if navdata.state == 2:
        rospy.signal_shutdown(complete)


    #============#
    #   Origin   #
    #============#

def Origin():

    global navdata, mode, start_time, states, time_past, K_way
    x = states.Obj[0].x
    y = states.Obj[0].y
    z = states.Obj[0].z
    if sqrt(x*x+y*y) < .05 and z > .95 and z < 1.05:
        time_past = rospy.get_time() - start_time
    else:
        start_time = rospy.get_time()
    traj = GetTrajectory(10, 0,0,0,1,0,0,0,mode)
    Basic_Controller(traj, K_way)
    if time_past > 3:
        mode = mode + 1
        start_time = rospy.get_time()

    #======================#
    #   Slanted Figure 8   #
    #======================#

def Slanted_Figure_8():

    global mode, start_time, time_past, K_slf8,cycles
    # Trajectory Variables
    period     = 10 # seconds
    a          = 1.5
    b          = 0.75
    c          = 0.5
    n          = .75
    w1         = 2*PI/period
    w2         = w1/2
    w3         = w1
    time_past = rospy.get_time() - start_time
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_slf8)
    if time_past > cycles*period:
        mode = mode + 1
        start_time = rospy.get_time()

    #===================#
    #   Flat Figure 8   #
    #===================#

def Flat_Figure_8():

    global mode, start_time, time_past, K_flf8,cycles
    # Trajectory Variables
    period     = 10 # seconds
    a          = 1.5
    b          = 0.75
    c          = 0.0
    n          = .75
    w1         = 2*PI/period
    w2         = w1/2
    w3         = w1
    time_past = rospy.get_time() - start_time
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_flf8)
    if time_past > cycles*period:
        mode = mode + 1
        start_time = rospy.get_time()

    #============#
    #   Circle   #
    #============#

def Circle():

    global mode, start_time, time_past, K_crcl,cycles
    # Trajectory Variables
    period     = 8 # seconds
    a          = 0.8
    b          = 0.8
    c          = 0
    n          = 1
    w1         = 2*PI/period
    w2         = w1
    w3         = w1
    time_past = rospy.get_time() - start_time
    traj = GetTrajectory(period, a, b, c, n, w1, w2, w3,mode)
    Basic_Controller(traj, K_crcl)
    if time_past > cycles*period:
        mode = mode + 1
        start_time = rospy.get_time()

    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller(traj,K):
    global states,PI, euler_max, max_yaw_rate, max_alt_rate

    Ctrl        = Controls()
    # Initiate Control Messages
    bodies = 1
    Ctrl.Obj = [Control()]*bodies
    Ctrl.header.stamp = states.header.stamp
    C1 = Control()
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
        u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot],[traj.Obj[0].psiddot]])
        u = utilde-u_r

        #=============================#
        #     Rotate to Body Frame    #
        #=============================#

        psi = states.Obj[0].psi*PI/180
        rotZ = np.matrix([[cos(psi), -sin(psi), 0],[sin(psi), cos(psi), 0],[0, 0, 1]])

        u[:-1] = rotZ*u[:-1]

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

        T = sqrt(u[0,-1]*u[0,-1]+u[1,-1]*u[1,-1]+u[2,-1]*u[2,-1])*m
        u[:-1] = np.divide(u[:-1],-T)

        #==================#
        #   Set Controls   #
        #==================#

        # Controls for Ardrone
        # -phi = right... +phi = left
        # -theta = back... +theta = forward
        # -psi = right...  +psi = left
        ctrl.name   = states.Obj[0].name
        ctrl.phi    = atan2(u[1,-1],u[2,-1])*euler_max
        ctrl.theta  = atan2(u[0,-1],u[2,-1])*euler_max
        ctrl.psi    = u[3,-1]/max_yaw_rate
        ctrl.alt    = -3*utilde[2,-1]/g
        Ctrl.Obj[0] = ctrl
        pub.publish(Ctrl)

def Datahandler():
    global PI,K_way,K_slf8,K_flf8,K_crcl,states,time_past,pub,image, mode
    status = rospy.get_param('controller_status',False)
    if status:
        if mode == 0:
            Takeoff()
        if mode == 1 or mode == 5:
            Origin()
        if mode == 2:
            Slanted_Figure_8()
        if mode == 3:
            Flat_Figure_8()
        if mode == 4:
            Circle()
        if mode == 6:
            Land()
    else:
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

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        sub_navdata = rospy.Subscriber('/ardrone/navdata' , Navdata, GetNavdata)
        sub_cortex  = rospy.Subscriber('/cortex' , Cortex, GetStates)
        Datahandler()
        r.sleep()

