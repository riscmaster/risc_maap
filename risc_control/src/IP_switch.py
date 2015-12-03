#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		IP_switch.py
    Organization:	RISC Lab, Utah State University
    Notes:
    ======================================================'''

    #================================#
    #    Libraries/modules Needed    #
    #================================#

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

    #========================#
    #        Globals         #
    #========================#

IP_control = False
ipctrl   = Control()
wpctrl   = Control()
h = Header()
rate       = 45      # Hz

    #==================#
    #    Publishers    #
    #==================#

pub = rospy.Publisher('/controls', Controls, queue_size = 1)

    #==============#
    #    Get Joy   #
    #==============#

def GetJoy(joy):
    global IP_control
    if (joy.buttons[5] == 1):
        IP_control = True
    else:
        IP_control = False

def GetIPControls(C):
    global ipctrl
    ipctrl = C.Obj[0]

def GetWPControls(C):
    global wpctrl,h
    wpctrl = C.Obj[0]
    h = C.header

def PublishControls():
    global ipctrl, wpctrl, IP_control, h
    cntrls = Controls()
    cntrls.Obj = [Control()]*1
    if IP_control:
        cntrls.Obj[0] = ipctrl
    if not IP_control:
        cntrls.Obj[0] = wpctrl
    cntrls.header = h
    pub.publish(cntrls)
    
    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('IP_switch')
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        sub1 = rospy.Subscriber('/joy' , Joy, GetJoy,queue_size = 1, buff_size = 2**24)
        sub2 = rospy.Subscriber('/ip_controls' , Controls, GetIPControls,queue_size = 1, buff_size = 2**24)
        sub3 = rospy.Subscriber('/wp_controls' , Controls, GetWPControls,queue_size = 1, buff_size = 2**24)
        PublishControls()
        r.sleep()

