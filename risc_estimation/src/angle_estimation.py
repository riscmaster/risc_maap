#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	March 2015
    File name: 		angle_estimation.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import tf
from numpy import *
import rospkg


    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from geometry_msgs.msg import PointStamped

    #========================#
    #        Globals         #
    #========================#

PI         = 3.141592653589793
Threshold  = 1000
Lm         = Landmarks()
Lm.Obj     = [Landmark()]*0
states     = Cortex()
fov_azim   = 75940 # based off of albatross
fov_elev   = 35609 # based off of albatross

plub = rospy.Publisher('/angles', Observed_angles, queue_size = 30)

    #============================#
    #    Datahandler Function    #
    #============================#

def AngleEstimate(rois):
    global fov_azim, fov_elev
    h = 360
    w = 640
    angle_h_view = fov_elev/1000
    angle_w_view = fov_azim/1000

    bodies = len(rois.Obj)
    Ang = Observed_angles()
    Ang.Obj = [Angles()]*bodies
    # Loop through all Bodies
    for j in range(bodies):
    # Loop through all quads
         quads = len(rois.Obj[j].quads)
         Ang.Obj[j].quads = [Angle()]*quads
         if rois.Obj[j].name == "albatross":
             angle_h_view = 35609/1000
             angle_w_view = 75940/1000
         for q in range(quads):
              Ang.Obj[j].quads[q].azim =  (rois.Obj[j].quads[q].x-w/2)*angle_w_view/w
              Ang.Obj[j].quads[q].elev = -(rois.Obj[j].quads[q].y-h/2)*angle_h_view/h
              Ang.Obj[j].quads[q].name = rois.Obj[j].quads[q].name
              Ang.Obj[j].quads[q].visible = rois.Obj[j].quads[q].visible
    # Loop through all landmarks
         landmarks = len(rois.Obj[j].landmarks)
         Ang.Obj[j].landmarks = [Angle()]*landmarks
         for i in range(landmarks):
              X = Angle()
              X.azim =  (rois.Obj[j].landmarks[i].x-w/2)*angle_w_view/w
              X.elev = -(rois.Obj[j].landmarks[i].y-h/2)*angle_h_view/h
              X.name = rois.Obj[j].landmarks[i].name
              X.visible = rois.Obj[j].landmarks[i].visible
              Ang.Obj[j].landmarks[i] = X
         Ang.Obj[j].name = rois.Obj[j].name
    Ang.header = rois.header
    global plub
    plub.publish(Ang)



    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys

    rospy.init_node('angle_estimation')


    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    while not rospy.is_shutdown():
        stub  = rospy.Subscriber('/land_rois' , Observed_rois, AngleEstimate)
        rospy.spin()
