#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	July 2014
    File name: 		waypoints.py
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

#  Tuned intially Using Closed Loop Ziegler Nichols Method
#  Use Kp value only and turn it up until you are Marginally stable
#  This is when you have steady oscillation about the desired value at a constant Amplitude
#  At this point record the kp value(Ku) and the period of oscillation(Tu).
#  The classic PID tuning Follows
#  kp = .6*Ku
#  ki = 1.2*Ku/Tu
#  kd = .6*Ku*Tu/8

# Roll
R_kp =135.6
R_ki =5.12#5.59# 298 #250
R_kd =89.8#78.6# 265 #625

# Pitch
P_kp =127.8# 588
P_ki =4.09# 350
P_kd =99.8# 875

# Yaw
#Y_kp = 531 # 800
#Y_kd = 47.06 # 0
#Y_ki = 149.86# 0
Y_kpp = 50 # 800
Y_kpm = 100 # 0
period = 167# 0

# Altitude
A_kp = 50
A_kd = 0
A_ki = 0

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
wp 	    	= Waypoints()
wp.Obj          = [Waypoint()]*0
rate            = 200 # Hz
image 		= 0

    #==========================#
    #    Pitch PI Controller   #
    #==========================#

def PID_Pitch(err,p):

    global P_kp, P_ki, P_kd
    kp = .01*P_kp # .25
    ki = .1*P_ki # 0.2
    kd = .01*P_kd # 0.0
    u  = (kp*err-kd*p+ki*err*.16667)
    return u

    #=========================#
    #    Roll PI Controller   #
    #=========================#

def PID_Roll(err,p):

    global R_kp,R_ki,R_kd
    kp = .01*R_kp
    ki = .1*R_ki
    kd = .01*R_kd
    u  = (kp*err-kd*p+ki*err*.16667)
    return u

    #========================#
    #    Yaw PI Controller   #
    #========================#
# Ku = 8.85, Tu = 2.257 seconds
# Kp = 5.31, Ki = 4.706, Kd = 1.498
def PID_Yaw(err,p):

    #global Y_kp,Y_ki,Y_kd
    global Y_kpp,Y_kpm,period
    kp = .1*Y_kpp
    if err<0:
        kp = .1*Y_kpm
    ki = 2*kp/(.01*period) 
    kd = kp*period/8 
    u  = ((kp*err-kd*p+ki*err*1/200))
    return u

    #=============================#
    #    Altitude PI Controller   #
    #=============================#

def PID_Alt(err,p):

    global A_kp,A_ki,A_kd
    kp = .1*A_kp
    ki = .01*A_ki
    kd = .001*A_kd
    u  = (kp*err-kd*p+ki*err*.16667)
    return u

    #====================#
    #    Get Waypoints   #
    #====================#

def waypoint(W):

    global wp
    wp = W

    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global wp,states
    
    if len(wp.Obj)>0 and len(S.Obj)>0:
        Datahandler(S,wp)

    #==========================#
    #    Trackbar Functions    #
    #==========================#

# Roll
def FR_kp(x):
    global R_kp
    R_kp = x
def FR_kd(x):
    global R_kd
    R_kd = x
def FR_ki(x):
    global R_ki
    R_ki = x

# Pitch
def FP_kp(x):
    global P_kp
    P_kp = x
def FP_kd(x):
    global P_kd
    P_kd = x
def FP_ki(x):
    global P_ki
    P_ki = x

# Yaw
def FY_kpp(x):
    global Y_kpp
    Y_kpp = x
def FY_kpm(x):
    global Y_kpm
    Y_kpm = x
def Fperiod(x):
    global period
    period = x

def FY_kd(x):
    global Y_kd
    Y_kd = x
def FY_ki(x):
    global Y_ki
    Y_ki = x

# Altitude
def FA_kp(x):
    global A_kp
    A_kp = x
def FA_kd(x):
    global A_kd
    A_kd = x
def FA_ki(x):
    global A_ki
    A_ki = x

    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler(states,wp):
    global PI

    #=======================#
    #    quad parameters    #
    #=======================#
    
    euler_max = rospy.get__param("euler_angle_max") #in radians
    max_yaw_rate = rospy.get__param("control_yaw") #in radians/sec
    max_alt_rate = rospy.get__param("control_vz_max") #in mm/sec

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

    if len(wp.Obj) > 0:
        # Loop through all Bodies
        for j in range(bodies):
            cv2.imshow("gains", image)
            cv2.waitKey(3)

        #=============================================#
        #    Get World Frame Pose of Waypoints    #
        #=============================================#
            if states.Obj[j].visible:
                 psi = states.Obj[j].psi - wp.Obj[j].heading
                 if psi>180:
                     psi = psi -360
                 if psi<-180:
                     psi = psi +360
            #     rospy.loginfo("error in degrees is %f",psi)
                 PosError = PointStamped()
                 PosError.point.x = wp.Obj[j].x
                 PosError.point.y = wp.Obj[j].y
                 PosError.point.z = wp.Obj[j].z
                 PosError.header.frame_id = "/cortex"

                 Vel = PointStamped()
                 Vel.point.x = wp.Obj[j].x
                 Vel.point.y = wp.Obj[j].y
                 Vel.point.z = wp.Obj[j].z
                 Vel.header.frame_id = "/cortex"

        #=============================#
        #     Rotate to Body Frame    #
        #=============================#

                 PosError = listener.transformPoint(states.Obj[j].name,PosError)
                 Vel      = listener.transformPoint(states.Obj[j].name,Vel)

                 # Position
                 r = PosError.point.x
                 p = PosError.point.y

                 # Velocities
                 u = Vel.point.x
                 v = Vel.point.y

        #=======================#
        #    PID Controllers    #
        #=======================#

                 ay  = PID_Roll(r,u)
                 ax  = PID_Pitch(p,v)
                 vz  = PID_Yaw(psi*PI/180,states.Obj[j].r*PI/180)*rate
                 alt = PID_Alt(wp.Obj[j].z-states.Obj[j].z,states.Obj[j].w)

        #==============================#
        #    Check for Cortex Error    #
        #==============================#

                 cmat[j].name  = states.Obj[j].name
                 cmat[j].theta = ax/g
                 cmat[j].phi   = -ay/g
                 cmat[j].psi   = vz/max_yaw_rate
                 cmat[j].alt   = alt/g
                 Ctrl.Obj[j] = cmat[j]
        pub.publish(Ctrl)
        wp.Obj = [Waypoint()]*0
        states.Obj = [States()]*0


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('waypoint_controller')
    listener = tf.TransformListener()

    #=======================#
    #    Create Trackbars   #
    #=======================#

    rospack = rospkg.RosPack()
    path = rospack.get_path('quad_command')
    image = cv2.imread(path+"/mario.jpg")
    cv2.resize(image,(321,123))
    cv2.namedWindow("gains")
#    cv2.createTrackbar("R_kp", "gains", R_kp, Threshold, FR_kp)
#    cv2.createTrackbar("R_ki", "gains", R_ki, Threshold, FR_ki)
#    cv2.createTrackbar("R_kd", "gains", R_kd, Threshold, FR_kd)
#    cv2.createTrackbar("P_kp", "gains", P_kp, Threshold, FP_kp)
#    cv2.createTrackbar("P_ki", "gains", P_ki, Threshold, FP_ki)
#    cv2.createTrackbar("P_kd", "gains", P_kd, Threshold, FP_kd)
    cv2.createTrackbar("Y_kpp", "gains", Y_kpp, Threshold, FY_kpp)
    cv2.createTrackbar("Y_kpm", "gains", Y_kpm, Threshold, FY_kpm)
    cv2.createTrackbar("period", "gains", period, Threshold, Fperiod)
#    cv2.createTrackbar("A_kp", "gains", A_kp, Threshold, FA_kp)
#    cv2.createTrackbar("A_ki", "gains", A_ki, Threshold, FA_ki)
#    cv2.createTrackbar("A_kd", "gains", A_kd, Threshold, FA_kd)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    pub = rospy.Publisher('/controls', Controls, queue_size = None)
    blub = rospy.Subscriber('/waypoints' , Waypoints, waypoint)
    sub  = rospy.Subscriber('/cortex' , Cortex, GetStates)
    rospy.spin()
    rospy.signal_shutdown('Uhh...')
    sys.exit(status)

