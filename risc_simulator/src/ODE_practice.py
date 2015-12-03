#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	November 2014
    File name: 		DF_Plots.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
from scipy import integrate as IN
import cv2
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from geometry_msgs.msg import Point

    #========================#
    #        Globals         #
    #========================#

PI 	           = 3.141592653589793
states             = PendulumStates()
states.r           = 0.0
states.s           = .00001
states.rdot        = 0.0
states.sdot        = 0.0
g                  = 9.81
L                  = 10
pen_in             = Point()
rate               = 200 # Hz
pub                = rospy.Publisher('/pendulum', PendulumStates, queue_size = None)
start_time	   = 0

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
    xddot = pen_in.x
    yddot = pen_in.y
    zddot = pen_in.z
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
    global PI,rate,pub,start_time,states
    # Build Messages
    #r = IN.ode(Inverted_Pendulum).set_integrator('vode', method='bdf')
    #t_start = rospy.get_time()-start_time
    y_start = [states.r,states.s,states.rdot,states.sdot]
    #r.set_initial_value(y_start,t_start)
    #r.integrate(r.t + 1/rate)
    if sqrt(states.r*states.r+states.s*states.s) < 99*L:
         dy = Inverted_Pendulum(y_start)
         states.r = states.r+dy[0]/rate
         states.s = states.s+dy[1]/rate
         states.rdot = states.rdot+dy[2]/rate
         states.sdot = states.sdot+dy[3]/rate
   # if r.successful():
   #      rospy.loginfo('integration successful')
   #      rospy.loginfo('%f',t_start)
    else:
         rospy.logwarn("Your pendulum Has Fallen!")
    h = std_msgs.msg.Header()
    h.seq = states.header.seq + 1
    h.stamp = rospy.Time.now()
    h.frame_id = 'reference_frame'
    states.header = h 
    pub.publish(states)

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
         sub  = rospy.Subscriber('/pen_input' , Point, GetInput)
         Driver()
         r.sleep()
