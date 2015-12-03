#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	August 2014
    File name: 		PIV_controller.py
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
R_kp =14000
R_ki =0
R_kd =1
R_err_old = 0
R_I_err_old = 0

# Pitch
P_kp = 14000
P_ki = 0
P_kd = 1
P_err_old = 0
P_I_err_old = 0


# Yaw
Y_kp = 22500
Y_ki = 0
Y_kd = 0
Y_err_old = 0
Y_I_err_old = 0


# Altitude
A_kp =  100
A_kd =  1
A_ki =  0
A_err_old = 0
A_I_err_old = 0

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000000
states          = Cortex()
states.Obj      = [States()]*0
traj 	    	= Trajectories()
traj.Obj        = [Trajectory()]*0
rate            = 200 # Hz
time_past       = 0 
image 		= 0

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

    #==========================#
    #    Pitch PI Controller   #
    #==========================#

def PID_Pitch(err,p):

    global P_kp, P_ki, P_kd,P_err_old,rate,P_I_err_old
    kp = .000001*P_kp # .25
    ki = .000001*P_ki # 0.2
    kd = .000001*P_kd # 0.0
    current = rospy.get_time()
    I = err/rate+P_I_err_old
    D = (p-P_err_old)/rate
#    if abs(err)<.01:
#        I = 0
    if ki == 0:
        I = 0
    if I > 1:
        I = 1
    if I < -1:
        I = -1
    u  = (kp*err+kd*D+ki*I)
    P_err_old = p
    P_I_err_old = I
    return u

    #=========================#
    #    Roll PI Controller   #
    #=========================#

def PID_Roll(err,p):

    global R_kp,R_ki,R_kd,R_err_old,rate,R_I_err_old
    kp = .000001*R_kp
    ki = .000001*R_ki
    kd = .000001*R_kd
    current = rospy.get_time()
    I = err/rate+R_I_err_old
    D = (p-R_err_old)/rate
   # if abs(err)<.01:
   #     I = 0
    if ki == 0:
        I = 0
    if I > 1:
        I = 1
    if I < -1:
        I = -1
    #rospy.loginfo("roll %f",I)
    u  = (kp*err+kd*D+ki*I)
    R_err_old = p
    R_I_err_old = I
    return u

    #========================#
    #    Yaw PI Controller   #
    #========================#
# Ku = 8.85, Tu = 2.257 seconds
# Kp = 5.31, Ki = 4.706, Kd = 1.498
def PID_Yaw(err,p):

    #global Y_kp,Y_ki,Y_kd
    global Y_kp,Y_ki,Y_kd,Y_err_old,rate,Y_I_err_old
    kp = .001*Y_kp
    ki = .001*Y_ki
    kd = .001*Y_kd
   # if abs(err)<.01:
   #     I = 0
    I = Y_I_err_old+err/rate
    if ki == 0:
        I = 0
    if I > PI:
        I = PI
    if I < -PI:
        I = -PI
    D = (p-Y_err_old)/rate
    #rospy.loginfo("yaw %f",I)
    u  = (kp*err-kd*p+ki*I/PI)
    Y_I_err_old = I;
    Y_err_old = p;

    return u

    #=============================#
    #    Altitude PI Controller   #
    #=============================#

def PID_Alt(err,p):

    global A_kp,A_ki,A_kd,A_err_old,rate,A_I_err_old
    kp = .000001*A_kp
    ki = .000001*A_ki
    kd = .000001*A_kd
    I = err/(rate)+A_err_old
    D = (p-A_err_old)/rate
  #  if abs(err)<.01:
  #      I = 0
    if ki == 0:
        I = 0
    if I > 1:
        I = 1
    if I < -1:
        I = -1
    u  = (kp*err-kd*D+ki*I)
    A_I_err_old = I;
    A_err_old = p;

    return u

    #====================#
    #    Get Waypoints   #
    #====================#

def GetTrajectory(W):

    global traj
#    traj = W
    traj.Obj = [Trajectory()]*1
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
def FR_kp(x):
    global R_kp
    R_kp = x
def FR_kd(x):
    global R_kd
    R_kd = x
def FR_ki(x):
    global R_ki
    R_ki = x

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

    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler():
    global PI,states,traj,rate,time_past
    #status = rospy.get_param('controller_status',True)

    cv2.imshow("gains", image)
    cv2.waitKey(3)
    #if not status:
    #    global P_I_err_old,R_I_err_old,Y_I_err_old,A_I_err_old,P_err_old,R_err_old,Y_err_old,A_err_old
    #    P_err_old = 0
    #    R_err_old = 0
    #    Y_err_old = 0
    #    A_err_old = 0
    #    P_I_err_old = 0
    #    R_I_err_old = 0
    #    Y_I_err_old = 0
    #    A_I_err_old = 0

    #if status:
    rate = 1/(rospy.get_time()-time_past)
 #   rospy.loginfo("rate = %f",rate)
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

    if len(traj.Obj) != 0:
        # Loop through all Bodies
        for j in range(bodies):

        #=============================================#
        #    Get World Frame Pose of Waypoints    #
        #=============================================#
             
            if states.Obj[j].visible:
                 PsiError = states.Obj[j].psi - traj.Obj[j].psi
                 if PsiError>180:
                     PsiError = PsiError -360
                 if PsiError<-180:
                     PsiError = PsiError +360
                 PsiVelError = states.Obj[j].r - traj.Obj[j].psi
                 if PsiVelError>180:
                     PsiVelError = PsiVelError -360
                 if PsiVelError<-180:
                     PsiVelError = PsiVelError +360

        #     rospy.loginfo("error in degrees is %f",psi)
                 PosError = PointStamped()
                 PosError.point.x = traj.Obj[j].x - states.Obj[j].x
                 PosError.point.y = traj.Obj[j].y - states.Obj[j].y
                 PosError.point.z = traj.Obj[j].z - states.Obj[j].z
                 PosError.header.frame_id = "/cortex"

                 VelError = PointStamped()
                 VelError.point.x = traj.Obj[j].xdot - states.Obj[j].u
                 VelError.point.y = traj.Obj[j].ydot - states.Obj[j].v
                 VelError.point.z = traj.Obj[j].zdot - states.Obj[j].w
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

                 ay  = -PID_Roll(y,v)
                # rospy.loginfo("ay = %f",ay)
                 ax  = -PID_Pitch(x,u)
                # rospy.loginfo("ax = %f",ay)
                 az  = PID_Alt(z,w)+9.81
                # rospy.loginfo("az = %f",ay)
                 vYaw  = PID_Yaw(psi,r)
                 thrust = sqrt(ax*ax + ay*ay + az*az)
                 ay = ay/thrust
                 az = az/thrust
                 ax = ax/thrust

        #====================#
        #    Set Commands    #
        #====================#

                 cmat[j].name  = states.Obj[j].name
                 cmat[j].theta = pi2pi(asin(-ax))/euler_max
                 cmat[j].phi   = pi2pi(atan2(ay,az))/euler_max
                 cmat[j].psi   = vYaw/max_yaw_rate
                 cmat[j].alt   = (az*thrust-9.81)*rate/max_alt_rate
                 Ctrl.Obj[j] = cmat[j]
        pub.publish(Ctrl)
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
    cv2.createTrackbar("P_kp", "gains", P_kp, Threshold, FP_kp)
    cv2.createTrackbar("P_ki", "gains", P_ki, Threshold, FP_ki)
    cv2.createTrackbar("P_kd", "gains", P_kd, Threshold, FP_kd)
    cv2.createTrackbar("R_kp", "gains", R_kp, Threshold, FR_kp)
    cv2.createTrackbar("R_ki", "gains", R_ki, Threshold, FR_ki)
    cv2.createTrackbar("R_kd", "gains", R_kd, Threshold, FR_kd)
    cv2.createTrackbar("A_kp", "gains", A_kp, Threshold, FA_kp)
    cv2.createTrackbar("A_ki", "gains", A_ki, Threshold, FA_ki)
    cv2.createTrackbar("A_kd", "gains", A_kd, Threshold, FA_kd)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    pub = rospy.Publisher('/controls', Controls, queue_size = None)
    sub  = rospy.Subscriber('/cortex' , Cortex, GetStates)
    blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
    rospy.spin()

