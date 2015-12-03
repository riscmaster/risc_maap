#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	August 2014
    File name: 		sharmas_controller.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import cv2
import tf
import rospkg

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from geometry_msgs.msg import PointStamped


    #==========================#
    #    Trackbar Variables    #
    #==========================#

# gains
Kx = 100 
Ky = 100 
Kz = 100
K_theta = 50 
K_phi = 40 
K_psi = 1500 

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
states          = Cortex()
states.Obj      = [States()]*0
wp 	    	= Waypoints()
wp_old 	    	= Waypoints()
wp.Obj          = [Waypoint()]*0
wp_old.Obj      = [Waypoint()]*2
rate            = 200 # Hz
time_past       = 0 
image 		= 0



    #===================#
    #    Cap a value    #
    #===================#
def cap(value, cap):
     if abs(value)>cap:
          value /= abs(value)
     return value

    #======================#
    #    Adjust Radians    #
    #======================#

def pi2pi(angle):
     if abs(angle)>PI/2:
           if angle>0:
                angle = angle-PI
           else:
                angle = angle+PI
     return angle

    #====================#
    #    Get Waypoints   #
    #====================#

def waypoint(W):

    global wp
    wp = W
    #rospy.loginfo("in waypoint")
    Datahandler()

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S
    #rospy.loginfo("in states")

    #==========================#
    #    Trackbar Functions    #
    #==========================#

def FKx(x):
    global Kx
    Kx = x
def FKy(x):
    global Ky
    RKy = x
def FKz(x):
    global Kz
    Kz = x
def FK_theta(x):
    global K_theta
    K_theta = x
def FK_phi(x):
    global K_phi
    K_phi = x
def FK_psi(x):
    global K_psi
    K_psi = x

    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler():
    global PI,wp_old,states,wp,rate,time_past
    status = rospy.get_param('controller_status',True)
    cv2.imshow("gains", image)
    cv2.waitKey(3)

    if status:
        rate = 1/(rospy.get_time()-time_past)
 #       rospy.loginfo("rate = %f",rate)
        #=======================#
        #    quad parameters    #
        #=======================#

        euler_max = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
        max_yaw_rate = float(rospy.get_param("control_yaw","3.490659")) #in radians/sec
        max_alt_rate = float(rospy.get_param("control_vz_max","1000")) #in mm/sec

        Ctrl        = Controls()
        # Initiate Control Messages
        bodies = len(states.Obj)
        Ctrl.Obj = [Control()]*bodies
        Ctrl.header.stamp = states.header.stamp
        C1 = Control()
        C2 = Control()
        C3 = Control()
        C4 = Control()
        cmat = [C1,C2,C3,C4]
        g = 9.81

        if len(wp.Obj) != 0:
            # Loop through all Bodies
            for j in range(bodies):

            #=============================================#
            #    Get World Frame Pose of Waypoints    #
            #=============================================#
                 
                if states.Obj[j].visible:
                     PsiError = states.Obj[j].psi - wp.Obj[j].heading
                     if PsiError>180:
                         PsiError = PsiError -360
                     if PsiError<-180:
                         PsiError = PsiError +360
                     PsiVelError = states.Obj[j].r-(wp.Obj[j].heading-wp_old.Obj[j].heading)/rate
                     if PsiVelError>180:
                         PsiVelError = PsiVelError -360
                     if PsiVelError<-180:
                         PsiVelError = PsiVelError +360

            #     rospy.loginfo("error in degrees is %f",psi)
                     PosError = PointStamped()
                     PosError.point.x = wp.Obj[j].x
                     PosError.point.y = wp.Obj[j].y
                     PosError.point.z = wp.Obj[j].z
                     PosError.header.frame_id = "/cortex"

                     VelError = PointStamped()
                     VelError.point.x = ((wp.Obj[j].x-wp_old.Obj[j].x)/rate) - states.Obj[j].u
                     VelError.point.y = ((wp.Obj[j].y-wp_old.Obj[j].y)/rate) - states.Obj[j].v
                     VelError.point.z = ((wp.Obj[j].z-wp_old.Obj[j].z)/rate) - states.Obj[j].w
                     VelError.header.frame_id = "/cortex"

            #=============================#
            #     Rotate to Body Frame    #
            #=============================#

                     PosError = listener.transformPoint(states.Obj[j].name,PosError)
                     VelError = listener.transformPoint(states.Obj[j].name,VelError)

                     # Position
                     x = PosError.point.x
                     y = PosError.point.y
                     z = PosError.point.z
                     psi = PsiError*PI/180

                     # Velocities
                     u = VelError.point.x
                     v = VelError.point.y
                     w = VelError.point.z
                     r = PsiVelError*PI/180

            #=======================#
            #    PID Controllers    #
            #=======================#
                     global K_theta,Ky,K_phi,Kx,Kz,K_psi

                     ay    = K_theta*.01*(Ky*.01*y+v)
                     ax    = -K_phi*.01*(Kx*.01*x+u)
                     az    = Kz*.01*z
                     vYaw  = K_psi*.01*psi

            #==============================#
            #    Check for Cortex Error    #
            #==============================#

                     cmat[j].name  = states.Obj[j].name
                     cmat[j].theta = cap(ay,1)
                     cmat[j].phi   = cap(ax,1)
                     cmat[j].psi   = cap(vYaw,1)
                     cmat[j].alt   = cap(az,1)
                     Ctrl.Obj[j]   = cmat[j]
            pub.publish(Ctrl)
            wp_old = wp
        time_past = rospy.get_time()


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('waypoint_controller')
    listener = tf.TransformListener()

    #=====================================#
    #    Set global time values for PID   #
    #=====================================#

    global time_past
    time_past = rospy.get_time()

    #=======================#
    #    Create Trackbars   #
    #=======================#

    rospack = rospkg.RosPack()
    path = rospack.get_path('risc_visual')
    image = cv2.imread(path+"/mountains.jpg")
    #cv2.resize(image,(321,123))
    cv2.namedWindow("gains")
    cv2.createTrackbar("Kx", "gains", Kx, Threshold, FKx)
    cv2.createTrackbar("Ky", "gains", Ky, Threshold, FKy)
    cv2.createTrackbar("Kz", "gains", Kz, Threshold, FKz)
    cv2.createTrackbar("K_psi", "gains", K_psi, 2*Threshold, FK_psi)
    cv2.createTrackbar("K_phi", "gains", K_phi, Threshold, FK_phi)
    cv2.createTrackbar("K_theta", "gains", K_theta, Threshold, FK_theta)
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    pub = rospy.Publisher('/controls', Controls, queue_size = None)
    sub  = rospy.Subscriber('/cortex' , Cortex, GetStates)
    blub = rospy.Subscriber('/waypoints' , Waypoints, waypoint)
    rospy.spin()

