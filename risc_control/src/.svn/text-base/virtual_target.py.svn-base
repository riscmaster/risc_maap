#!/usr/bin/env python

'''======================================================
                    ICUAS 2015 Trajectory
   ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import cv2
import time
import numpy as np
    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Joy

    #========================#
    #        Globals         #
    #========================#

# enable time, pi and publisher
start_time = 0
mode       = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 1)
pub_vel    = rospy.Publisher('target_velocity',Float64,queue_size = 200)


# Select Trajectory Variables
period     = 10 # seconds
a          = .8
b          = .8
c          = 0
n          = 1
vt         = 0.2 # m/s
#V_veh = 0.3
Veh_init_X = 0.5643
Veh_init_Y = 1.2213
phase      = PI/2 


    #======================================#
    #    Parameters never to be changed    #
    #======================================#

Des_X      = 0.5643    # Please Never Change
Des_Y      = 1.2213   # Please Never Change


# initial position of the quad
STATES = States()
start_state = States()
init = True


    #===============#
    #    Get Joy    #
    #===============#

def GetJoy(joy):
    global start_time, mode
    if mode == 0:
        start_time = rospy.get_time()
    mode    = mode + joy.buttons[1]
    #print mode

    #=========================#
    #    Get Initial State    #
    #=========================#

def GetStates(X):
    global STATES,start_state, init, mode
    STATES = X.Obj[0]
    if init and mode != 0:
        start_state = X.Obj[0]
        init = False
    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, PI, pub, period, a, b, c, n, w1, w2, w3, vt, mode,Veh_init_X,Veh_init_Y,X_offset,Y_offset,phase,STATES,start_state,Des_X,Des_Y
    WP 		= Trajectories()
    WP.Obj 	= [Trajectory()]*1
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    Xv_init     = start_state.x
    Yv_init     = start_state.y
    
    #Rot_Phase   = np.matrix([[cos(phase),-sin(phase)],[sin(phase),cos(phase)]])

    #print x
    if mode == 0:

        #=================#
        #    Trajectory   #
        #=================#

        traj = Trajectory()
        traj.name = "WP"
        # Position
        #XY_veh_rotated   = Rot_Phase*(np.matrix([[Veh_init_X],[Veh_init_Y]]))
        traj.x       = Veh_init_X#XY_veh_rotated[0,-1]
        traj.y       = Veh_init_Y#XY_veh_rotated[1,-1]
        traj.z       = 1
        traj.psi     = -PI/2
        #-3.14/2
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
        
    else:


        w            = vt/a
        Correction_X = (Xv_init-Des_X)
        Correction_Y = (Yv_init-Des_Y)


        #=================#
        #    Trajectory   #
        #=================#

        traj = Trajectory()
        traj.name = "NGC"
        # Position
                
        traj.x       = Correction_X+a*cos(phase+w*t)       
        traj.y       = Correction_Y+b*sin(phase+w*t)      
        traj.z       = n+c*sin(w*t)
        traj.psi     = 0
        # Velocity
        traj.xdot    = -a*w*sin(w*t)
        traj.ydot    = b*w*cos(w*t)
        traj.zdot    = c*w*cos(w*t)
        traj.psidot  = 0
        # Acceleration
        traj.xddot   = -a*w*w*cos(w*t)
        traj.yddot   = -b*w*w*sin(w*t)
        traj.zddot   = -c*w*w*sin(w*t)
        traj.psiddot = 0
        
    #==================#
    #     Publish      #
    #==================#

    WP.Obj = [traj]
    pub.publish(WP)
    pub_vel.publish(vt)


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('virtual_target')
    start_time = rospy.get_time()
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#
    #sub_states = rospy.Subscriber('/cortex_raw', Cortex, GetStates)
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        sub_states = rospy.Subscriber('/cortex_raw', Cortex, GetStates)
        sub = rospy.Subscriber('/joy', Joy, GetJoy)
        Datahandler()
        r.sleep()
    # and only progresses to here once the application has been shutdown
    rospy.loginfo("Virtual Target Node Has Shutdown.")
    rospy.signal_shutdown(0)
