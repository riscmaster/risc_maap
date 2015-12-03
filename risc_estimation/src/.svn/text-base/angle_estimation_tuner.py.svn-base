#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	July 2014
    File name: 		angle_estimation_tuner.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import cv2
import tf
import rospkg
from numpy import *


    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from geometry_msgs.msg import PointStamped

    #========================#
    #        Globals         #
    #========================#

PI 		    = 3.141592653589793
angle_w =1000
angle_h =1000
image = 0
Threshold 	= 2000
Lm 	    	= Landmarks()
Lm.Obj      = [Landmark()]*0
states 		= Cortex()

    #==========================#
    #    Trackbar Functions    #
    #==========================#

# angle
def Fangle_w(x):
    global angle_w
    angle_w = x

def Fangle_h(x):
    global angle_h
    angle_h = x

    #====================#
    #    Get Landmarks   #
    #====================#

def getLandmarks(W):

    global Lm
    Lm = W

    #========================#
    #    Get Cortex States   #
    #========================#

def getStates(S):

    global states,Lm
    states = S
    if len(Lm.Obj) > 0 and len(states.Obj)>0:
            AngleTruth()

    #============================#
    #    Angle truth Function    #
    #============================#

def AngleTruth():
    global Lm ,states,PI
    bodies = len(states.Obj)
    Ang = Observed_angles()
    Ang.Obj = [Angles()]*bodies
    land_num = len(Lm.Obj)
    if len(Lm.Obj) > 0:
        # Loop through all Bodies
        for j in range(bodies):
            Ang.Obj[j].landmarks = [Angle()]*land_num
            Ang.Obj[j].name = states.Obj[j].name
            # Loop through all Landmarks
            for k in range(land_num):

            #==================================#
            #    Get Cortex Frame Positions    #
            #==================================#

                X = PointStamped()
                X.point.x = Lm.Obj[k].x
                X.point.y = Lm.Obj[k].y
                X.point.z = Lm.Obj[k].z
                X.header.frame_id = "/cortex"

                if X.point.x < 3:

            #========================================#
            #    Convert to  Body Frame Positions    #
            #========================================#

                    now = rospy.Time.now()
                    listener.waitForTransform("/cortex", states.Obj[j].name+"/camera", now, rospy.Duration(4.0))
                    X = listener.transformPoint(states.Obj[j].name+"/camera", X)

            #===========================#
            #    Get Observed Angles    #
            #===========================#

                    A = Angle()
		    A.visible = True
                    if Lm.Obj[k].x == 0 and Lm.Obj[k].y == 0 and Lm.Obj[k].z == 0:
                          A.visible = False
                    A.azim = arctan2(X.point.x,X.point.y)*180/PI
                    A.elev = -arctan2(X.point.z,X.point.y)*180/PI
                    A.name = Lm.Obj[k].name
                    Ang.Obj[j].landmarks[k] = A

    Ang.header.stamp = states.header.stamp
    pub.publish(Ang)

    #============================#
    #    Datahandler Function    #
    #============================#

def AngleEstimate(rois):
    global angle,image
    cv2.imshow("angle", image)
    cv2.waitKey(3)

    h = 360
    w = 640
    angle_h_view = 70.7492398*h/w + (.01*(angle_h-1000))
    angle_w_view = 70.7492398 + (.01*(angle_w-1000))
    bodies = len(rois.Obj)
    Ang = Observed_angles()
    Ang.Obj = [Angles()]*bodies
    if len(Lm.Obj) > 0:
        # Loop through all Bodies
        for j in range(bodies):
            Ang.Obj[j].name = rois.Obj[j].name
	    land_num = len(rois.Obj[j].roi)
            Ang.Obj[j].landmarks = [Angle()]*land_num
            # Loop through all Landmarks
            for k in range(land_num):

            #============================#
            #    Get Estimated Angles    #
            #============================#

                    A = Angle()
		    A.visible = rois.Obj[j].roi[k].visible
                    A.azim = ((rois.Obj[j].roi[k].x-w/2)-.5)*angle_w_view/w
                    A.elev = ((rois.Obj[j].roi[k].y-h/2)-.5)*angle_h_view/h
                    A.name = rois.Obj[j].roi[k].name
                    Ang.Obj[j].landmarks[k] = A

    Ang.header.stamp = rois.header.stamp
    plub.publish(Ang)



    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys

    rospy.init_node('angle_estimation')
    rospack = rospkg.RosPack()
    path = rospack.get_path('quad_command')
    image = cv2.imread(path+"/mario.jpg")
    cv2.resize(image,(321,123))
    cv2.namedWindow("angle")
    cv2.createTrackbar("height 1000=0", "angle", angle_h, Threshold, Fangle_h)
    cv2.createTrackbar("width 1000=0", "angle", angle_w, Threshold, Fangle_w)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    listener = tf.TransformListener()
    pub = rospy.Publisher('/true_angles', Observed_angles, queue_size = None)
    plub = rospy.Publisher('/estimated_angles', Observed_angles, queue_size = None)
    blub = rospy.Subscriber('/landmarks' , Landmarks, getLandmarks)
    sub  = rospy.Subscriber('/cortex' , Cortex, getStates)
    stub  = rospy.Subscriber('/ROIs' , Observed_rois, AngleEstimate)
    rospy.spin()
