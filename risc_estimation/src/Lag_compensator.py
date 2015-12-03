#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	November 2014
    File name: 		Lag_compensator.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import cv2

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from geometry_msgs.msg import PointStamped

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
states          = Cortex()
states.Obj      = [States()]*0
Control_msg     = Controls()
Control_msg.Obj  = [Control()]*4
rate            = 200 # Hz
pub             = rospy.Publisher('/cortex_estimate', Cortex, queue_size = None)
delay           = 40 # ms
Ctrl1           = np.zeros((4,ceil(delay*rate/1000)))
Ctrl2           = np.zeros((4,ceil(delay*rate/1000)))
Ctrl3           = np.zeros((4,ceil(delay*rate/1000)))
Ctrl4           = np.zeros((4,ceil(delay*rate/1000)))
Ctrl            =  [Ctrl1, Ctrl2, Ctrl3, Ctrl4]

    #====================#
    #    Get Controls    #
    #====================#

def GetControls(C):
    global Control_msg
    Control_msg = C
    #rospy.loginfo('getting controls')
   
    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states,Ctrl,Control_msg
    states = S
    bodies = len(S.Obj)
    for i in range(bodies):
         v = np.array([[Control_msg.Obj[i].phi],[Control_msg.Obj[i].theta],[Control_msg.Obj[i].psi],[Control_msg.Obj[i].alt]])
         new_ctrl = np.zeros(Ctrl[i].shape)
         new_ctrl[:,0:-1] = Ctrl[i][:,1:]
         new_ctrl[:,-1] = v[:,-1]
         Ctrl[i] = new_ctrl 
    #rospy.loginfo("in states")
    
    #======================#
    #    Rotaion matrix    #
    #======================#

def RotZ(psi):
    mat = np.array([[cos(psi*PI/180), -sin(psi*PI/180)],[sin(psi*PI/180), cos(psi*PI/180)]])
    return mat
    
    #========================#
    #    Old States to New   #
    #========================#

def NewStates(STATES,CONTROLS,PARAMS):
    global PI
    euler_max    = 0.349066   # float(rospy.get_param("euler_angle_max","0.349066"))
    max_yaw_rate = 0.349066  # float(rospy.get_param("control_yaw","0.7854"))
    max_alt_rate = 1          # float(rospy.get_param("control_vz_max","0.1"))
    # PARAMS = np.array([p0, p1, p2, p3, r0 ,r1, r2 ,r3, y0, y1, a0, a1])
    steps  = len(CONTROLS[0])
    n_state = STATES 
    psi_rot_m = RotZ(STATES.psi) # input is degrees
    v1_velocity = np.dot(psi_rot_m,np.array([[STATES.v],[STATES.u]]))
    # First Order Discrete Linear Model, (this was estimated at 200 Hz and cannot be expected to hold at varying rates)
    for i in range(steps):
         n_state.theta = PARAMS[0]*euler_max*CONTROLS[1][i]+PARAMS[1]*n_state.theta 
         n_state.phi   = PARAMS[4]*euler_max*CONTROLS[0][i]+PARAMS[5]*n_state.phi 
         n_state.r     = (PARAMS[8]*max_yaw_rate*CONTROLS[2][i]+PARAMS[9]*n_state.r*PI/180)*180/PI 
         n_state.psi   = (n_state.r*.005)*PI/180+n_state.psi
         rot_m         = RotZ(-STATES.psi) # input is degrees
         v1_velocity   = np.array([[PARAMS[2]*n_state.theta+PARAMS[3]*v1_velocity[0,-1]],[PARAMS[6]*n_state.phi+PARAMS[7]*v1_velocity[1,-1]]])
         world_vel     = np.dot(rot_m,v1_velocity)
         n_state.v     = world_vel[0,-1]
         n_state.u     = world_vel[1,-1]
         n_state.x     = n_state.u*.005 + n_state.x 
         n_state.y     = n_state.v*.005 + n_state.y 
         n_state.w     = PARAMS[10]*max_alt_rate*CONTROLS[3][i]+PARAMS[11]*n_state.w 
         n_state.z     = n_state.w*.005 + n_state.z 
    return n_state

    #=========================#
    #    Basic Compensator    #
    #=========================#

def Datahandler():
    global PI,states,rate,Ctrl,pub

        #=======================#
        #    quad parameters    #
        #=======================#

    #   Initialize asuming Condor linear approximation   #
    # Using the parameters slows it down
    p0 = 0.0286 # float(rospy.get_param("condor_p0","0.0286")) 
    p1 = 0.9730 # float(rospy.get_param("condor_p1","0.9730")) 
    p2 = 0.001  # float(rospy.get_param("condor_p2","0.001")) 
    p3 = 0.9962 # float(rospy.get_param("condor_p3","0.9962")) 
                # 
    r0 = 0.0308 # float(rospy.get_param("condor_r0","0.0308")) 
    r1 = 0.9694 # float(rospy.get_param("condor_r1","0.9694")) 
    r2 = 0.0007 # float(rospy.get_param("condor_r2","0.000725")) 
    r3 = 0.9982 # float(rospy.get_param("condor_r3","0.9982")) 
                # 
    y0 = 0.0113 # float(rospy.get_param("condor_y0","0.0113")) 
    y1 = 0.9885 # float(rospy.get_param("condor_y1","0.9885"))
                # 
    a0 = 0.0083 # float(rospy.get_param("condor_a0","0.0083")) 
    a1 = 0.9960 # float(rospy.get_param("condor_a1","0.9960"))
    params = np.array([p0, p1, p2, p3, r0 ,r1, r2 ,r3, y0, y1, a0, a1])

    # Build Messages
    bodies = len(states.Obj)
    newstates       = Cortex()
    newstates.Obj   = [States()]*bodies
    for j in range(bodies):
         newstates.Obj[j] = NewStates(states.Obj[j],Ctrl[j],params)
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'cortex'
    newstates.header = h 
    pub.publish(newstates)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Compensator')

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub  = rospy.Subscriber('/cortex' , Cortex, GetStates)
         stub  = rospy.Subscriber('/controls' , Controls, GetControls)
         Datahandler()
         r.sleep()
