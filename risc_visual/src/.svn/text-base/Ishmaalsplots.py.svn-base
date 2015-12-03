#!/usr/bin/env python

'''======================================================
    Created by:  	Ishmaal Erekson
    Last updated: 	January 2015
    File name: 		Ishmaalsplots.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pylab as p
import matplotlib.pyplot as plt
import IshyPlots as pl
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import * # states,controls,trajectory
from ardrone_autonomy.msg import * # on board state estimates

    #========================#
    #        Globals         #
    #========================#

rate               = 60 # Hz
start_time         = 0
states             = Cortex()
states.Obj         = [States()]*1
t	           = 0


    #===================================#
    #        Plotting Variables         #
    #===================================#

Init_mat = np.asmatrix(np.zeros((1,3)))

    #==================#
    #    Get States    #
    #==================#

def GetStates(I):
    global states
    states = I

    #=============#
    #    Plots    #
    #=============#

def Plots():
    global states,Init_mat
    x_act      = states.Obj[0].x
    y_act      = states.Obj[0].y
    z_act      = states.Obj[0].z
    new_stack = np.asmatrix(np.array([x_act, y_act, z_act]))
    Init_mat = np.append(Init_mat,new_stack,0)

def Plotaroosky():
    global Init_mat
    pl.pl3d('Plotalicious',Init_mat[0:,0],Init_mat[0:,1],Init_mat[0:,2])
    plt.show()

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Ishmaalsplots')
    start_time = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub_states = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
         Plots()
         t = rospy.get_time() - start_time
         rospy.loginfo("time = %f",t)
         if t > 20:
             Plotaroosky()
             start_time = rospy.get_time()
         r.sleep()
    
    
