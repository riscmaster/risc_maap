#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	August 2014
    File name: 		altPIV.py
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

# Roll and Pitch
RP_kp = 60 
RP_ki = 50
RP_kd = 6
R_err_old = 0
R_I_err_old = 0
P_err_old = 0
P_I_err_old = 0

# Yaw
Y_kp = 600
Y_ki = 10
Y_kd = 0
Y_err_old = 0
Y_I_err_old = 0


# Altitude
A_kp =  755
A_kd =  225
A_ki =  90
A_err_old = 0
A_I_err_old = 0

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
rise_fac        = 2.5
max_rise        = .8
max_drop        = -.25
error           = 100 
agr             = 100 


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

    #=======================#
    #    Pitch Controller   #
    #=======================#

def PID_Pitch(err,p):

    global RP_kp, RP_ki, RP_kd,P_err_old,rate,P_I_err_old
    kp = .001*RP_kp 
    ki = .001*RP_ki 
    kd = .001*RP_kd 
    # kill integral if it cross waypoint
    if err<0 and P_I_err_old >0:
         P_I_err_old = 0
    if err>0 and P_I_err_old <0:
         P_I_err_old = 0
    # disable integral stacking while tuning
    if ki == 0:
        P_I_err_old = 0
    # integrate
    I = (P_err_old+err)/(2*rate)+P_I_err_old
    # differentiate
    D = p
    # get control term and cap it
    u  = (kp*err+kd*D+ki*I)
    if abs(u)>1:
        u /= abs(u)
    # update stored errors and send
    P_err_old = p
    P_I_err_old = I
    return u

    #======================#
    #    Roll Controller   #
    #======================#

def PID_Roll(err,p):

    global RP_kp,RP_ki,RP_kd,R_err_old,rate,R_I_err_old
    kp = .001*RP_kp
    ki = .001*RP_ki
    kd = .001*RP_kd
    # kill integral if it cross waypoint
    if err<0 and R_I_err_old >0:
         R_I_err_old = 0
    if err>0 and R_I_err_old <0:
         R_I_err_old = 0
    # disable integral stacking while tuning
    if ki == 0:
        R_I_err_old = 0
    # integrate
    I = (R_err_old+err)/(2*rate)+R_I_err_old
    # differentiate
    D = p
    # get control term and cap it
    u  = (kp*err+kd*D+ki*I)
    if abs(u)>1:
        u /= abs(u)
    # update stored errors and send
    R_err_old = p
    R_I_err_old = I
    return u

    #=====================#
    #    Yaw Controller   #
    #=====================#

def PID_Yaw(err,p):

    global Y_kp,Y_ki,Y_kd,Y_err_old,rate,Y_I_err_old
    kp = .001*Y_kp
    ki = .001*Y_ki
    kd = .001*Y_kd
    # kill integral if it cross waypoint
    if err<0 and Y_I_err_old >0:
         Y_I_err_old = 0
    if err>0 and Y_I_err_old <0:
         Y_I_err_old = 0
    # disable integral stacking while tuning
    if ki == 0:
        Y_I_err_old = 0
    # integrate
    I = (Y_err_old+err)/(2*rate)+Y_I_err_old
    # differentiate
    D = p
    # get control term and cap it
    u  = (kp*err-kd*p+ki*I/PI)
    if abs(u)>1:
        u /= abs(u)
    # update stored errors and send
    Y_I_err_old = I;
    Y_err_old = p;

    return u

    #=============================#
    #    Altitude PI Controller   #
    #=============================#

def PID_Alt(err,p):

    global A_kp,A_ki,A_kd,A_err_old,rate,A_I_err_old,rise_fac,max_rise,max_drop
    kp = .001*A_kp
    ki = .001*A_ki
    kd = .001*A_kd
    # kill integral if it cross waypoint
    if err<0 and A_I_err_old >0:
         A_I_err_old = 0
    if err>0 and A_I_err_old <0:
         A_I_err_old = 0
    # disable integral stacking while tuning
    if ki == 0:
        A_I_err_old = 0
    # integrate
    I = (A_err_old+err)/(2*rate)+A_I_err_old
    # differentiate
    D = p
    # get control term and cap it
    u  = (kp*err-kd*D+ki*I)
    if err>0:
        u = u*rise_fac
    if u>.8:
        u = max_rise
    if u<-.25:
        u = max_drop 
    # update stored errors and send
    A_I_err_old = I;
    A_err_old = p;

    return u

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

# Roll
def FRP_kp(x):
    global RP_kp
    RP_kp = x
def FRP_kd(x):
    global RP_kd
    RP_kd = x
def FRP_ki(x):
    global RP_ki
    RP_ki = x

# Yaw
def FY_kp(x):
    global Y_kp
    Y_kp = x
def FY_kd(x):
    global Y_kd
    Y_kd = x
def FY_ki(x):
    global Y_ki
    Y_ki = x


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
# error allowed
def Ferror(x):
    global error
    error = x
def Fagr(x):
    global agr
    agr = x


    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler():
    global PI,wp_old,states,wp,P_I_err_old,R_I_err_old,Y_I_err_old,A_I_err_old,P_err_old,R_err_old,Y_err_old,A_err_old,rate,time_past,error
    status = rospy.get_param('controller_status',True)
    ok = error*.001
    cv2.imshow("gains", image)
    cv2.waitKey(3)
    if not status:
        P_err_old = 0
        R_err_old = 0
        Y_err_old = 0
        A_err_old = 0
        P_I_err_old = 0
        R_I_err_old = 0
        Y_I_err_old = 0
        A_I_err_old = 0

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
                     global agr
                     m = agr*.01
                     ay    = m*PID_Pitch(y,v)
                     ax    = m*PID_Roll(x,u)
                     az    = PID_Alt(z,w)
                     vYaw  = PID_Yaw(psi,r)

            #==============================#
            #    Check for Cortex Error    #
            #==============================#

                     cmat[j].name  = states.Obj[j].name
                     cmat[j].theta = -pi2pi(atan2(ay,(az-g)))/euler_max 
                     cmat[j].phi   = pi2pi(atan2(ax*cos(cmat[j].theta*euler_max),(az-g)))/euler_max 
                     cmat[j].psi   = vYaw 
                     cmat[j].alt   = az 
                     Ctrl.Obj[j]   = cmat[j]
                      # if location is within ok then hover
                     if x < ok and y < ok and z<ok and (wp.Obj[j].heading-wp_old.Obj[j].heading)==0:

                          cmat[j].theta = 0 
                          cmat[j].phi   = 0 
                          cmat[j].psi   = vYaw 
                          cmat[j].alt   = az 
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
    cv2.createTrackbar("Y_kp", "gains", Y_kp, Threshold, FY_kp)
    cv2.createTrackbar("Y_ki", "gains", Y_ki, Threshold, FY_ki)
    cv2.createTrackbar("Y_kd", "gains", Y_kd, Threshold, FY_kd)
#    cv2.createTrackbar("P_kp", "gains", P_kp, Threshold, FP_kp)
#    cv2.createTrackbar("P_ki", "gains", P_ki, Threshold, FP_ki)
#    cv2.createTrackbar("P_kd", "gains", P_kd, Threshold, FP_kd)
    cv2.createTrackbar("RP_kp", "gains", RP_kp, Threshold, FRP_kp)
    cv2.createTrackbar("RP_ki", "gains", RP_ki, Threshold, FRP_ki)
    cv2.createTrackbar("RP_kd", "gains", RP_kd, Threshold, FRP_kd)
    cv2.createTrackbar("A_kp", "gains", A_kp, Threshold, FA_kp)
    cv2.createTrackbar("A_ki", "gains", A_ki, Threshold, FA_ki)
    cv2.createTrackbar("A_kd", "gains", A_kd, Threshold, FA_kd)
    cv2.createTrackbar("allowed error", "gains", error, Threshold, Ferror)
    cv2.createTrackbar("aggressiveness", "gains", agr, Threshold, Fagr)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    pub = rospy.Publisher('/controls', Controls, queue_size = None)
    sub  = rospy.Subscriber('/cortex' , Cortex, GetStates)
    blub = rospy.Subscriber('/waypoints' , Waypoints, waypoint)
    rospy.spin()

