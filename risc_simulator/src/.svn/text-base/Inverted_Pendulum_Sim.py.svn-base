#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	February 2015
    File name: 		Inverted_Pendulum_Sim.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import cv2
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from std_msgs.msg import Bool# to make header
from geometry_msgs.msg import PointStamped

    #========================#
    #        Globals         #
    #========================#

PI 	           = 3.141592653589793
ardrone            = States()
states             = States()
states.x           = 0.0
states.y           = 0.0
states.u           = 0.0
states.v           = 0.0
g                  = 9.81
L                  = 1
pen_in             = PointStamped()
rate               = 200 # Hz
pub_states         = rospy.Publisher('/cortex_raw', Cortex, queue_size = 1)
start_time	   = 0
controller_status  = Bool()
controller_status.data = False

    #==================#
    #    Get States    #
    #==================#

def GetStates(I):
    global ardrone
    ardrone = I

    #=============================#
    #    Get Controller Status    #
    #=============================#

def GetControllerStatus(I):
    global controller_status
    controller_status = I

    #=================#
    #    Get Input    #
    #=================#

def GetInput(I):
    global pen_in
    pen_in = I
    #rospy.loginfo('getting input')

    #=================#
    #    Integrater   #
    #=================#

def Inverted_Pendulum(y):
    global pen_in,g,L
    r    = y[0]
    s    = y[1]
    rdot = y[2]
    sdot = y[3]
    xddot = pen_in.point.x
    yddot = pen_in.point.y
    zddot = pen_in.point.z
    # variables to simplify expressions
    zeta = sqrt(L*L-r*r-s*s)
    zeta2 = L*L-r*r-s*s

    A = (zeta2/(zeta2 + r*r))*( -xddot + zddot*r/zeta - rdot*rdot*r/zeta2 -  sdot*sdot*r/zeta2 - rdot*rdot*r*r*r/(zeta2*zeta2) - 2*rdot*sdot*r*r*s / (zeta2*zeta2) - sdot*sdot*s*s*r/(zeta2*zeta2) + g*r/zeta)

    B = (zeta2/(zeta2 + s*s))*( -yddot + zddot*s/zeta - sdot*sdot*s/zeta2 - rdot*rdot*s/zeta2 - sdot*sdot*s*s*s/(zeta2*zeta2) - 2*sdot*rdot*s*s*r/(zeta2*zeta2)	- rdot*rdot*r*r*s/(zeta2*zeta2) + g*s/zeta)

    dxdt =[0,0,0,0]
    dxdt[0] = y[2]
    dxdt[1] = y[3]
    dxdt[2] = (L*L-r*r)*(A*(L*L-s*s)-B*r*s)/((L*L-s*s)*(L*L-s*s)-r*r*s*s)
    dxdt[3] = B - dxdt[2]*r*s/(L*L-r*r)
    return dxdt

    #==================#
    #    ODE Driver    #
    #==================#

def Driver():
    global PI,rate,pub,start_time,states,L,controller_status, ardrone
    # Build Messages
    if controller_status.data:
        y_start = [states.x,states.y,states.u,states.v]
        if sqrt(states.x*states.x+states.y*states.y) < .99*L:
             dy = Inverted_Pendulum(y_start)
             states.x = states.x+dy[0]/rate
             states.y = states.y+dy[1]/rate
             states.u = states.u+dy[2]/rate
             states.v = states.v+dy[3]/rate
        else:
             rospy.logwarn("Your Pendulum Has Fallen... \n Please Try again.")
             while not rospy.is_shutdown():
                 rospy.sleep(.05)
    zeta = sqrt(L*L-states.x*states.x-states.y*states.y)
    Pen = States()
    Pen.name = "Pendulum_tip"
    Pen.visible = True
    Pen.x = ardrone.x+states.x
    Pen.y = ardrone.y+states.y
    Pen.z = ardrone.z+zeta
    Pen.u = ardrone.u+states.u
    Pen.v = ardrone.v+states.v
    Pen.w = ardrone.w-(states.x*states.u+states.y*states.v)/zeta

    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'cortex'
    cortex = Cortex()
    cortex.Obj = [States()]*2
    cortex.header = h
    cortex.Obj[0] = ardrone
    cortex.Obj[1] = Pen

    pub_states.publish(cortex)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Pendulum')
    start_time = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub  = rospy.Subscriber('/pen_input' , PointStamped, GetInput)
         sub_states  = rospy.Subscriber('/quad/states' , States, GetStates)
         sub_controller  = rospy.Subscriber('/controller_status' , Bool, GetControllerStatus)
         Driver()
         r.sleep()
