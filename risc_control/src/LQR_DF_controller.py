#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	November 2014
    File name: 		LQR_DF_controller.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import cv2
import rospkg
import numpy as np
import scipy.linalg as la

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from geometry_msgs.msg import PointStamped

#
#    #==========================#
#    #    Trackbar Variables    #
#    #==========================#
## as starting values choose
#qx = 10
#qy = 10
#qz = 10
#qxdot = 1
#qydot = 1
#qzdot = 1
#qpsi = 1
#rphi = 50
#rtheta = 50
#rpsi = 50
#ralt = 50
#
    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
states          = Cortex()
states.Obj      = [States()]*0
traj            = Trajectories()
traj.Obj        = [Trajectory()]*1
rate            = 200 # Hz
time_past       = 0
image 		= 0
pub             = rospy.Publisher('/controls', Controls, queue_size = 200)
#A               = np.array([[np.zeros((3,3)), np.eye(3,dtype=int),np.zeros((3,1))],[np.zeros((4,7))]])
#B               = np.array([[np.zeros((3,4))],[np.eye(4,dtype=int)]])
#q               = np.array([qx, qy, qz, qxdot, qydot, qzdot,qpsi])
#r               = np.array([rphi, rtheta, rpsi, ralt])
#Q               = np.diag(q,0)
#R               = np.diag(r,0)
K               = np.matrix([[ 4.15, 0, 0, 3.15, 0, 0, 0],[0, 4.15, 0, 0, 3.15, 0, 0],[0, 0, 3, 0, 0, 0.0001, 0],[0, 0, 0, 0, 0, 0, 1]])


    #=====================#
    #    LQR Functions    #
    #=====================#


#def lqr(A,B,Q,R):
#    # Solve the continuous time lqr controller.
#    #
#    # dx/dt = A x + B u
#    #
#    # cost = integral x.T*Q*x + u.T*R*u
#
#    # first, try to solve the ricatti equation
#    X = np.matrix(la.solve_continuous_are(A, B, Q, R))
#
#    #compute the LQR gain
#    K = np.matrix(la.inv(R)*(B.T*X))
#
#    #eigVals, eigVecs = scipy.linalg.eig(A-B*K)
#
#    return K #, X, eigVals
#
#def dlqr(A,B,Q,R):
#    # Solve the discrete time lqr controller.
#    #
#    #
#    # x[k+1] = A x[k] + B u[k]
#    #
#    # cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
#
#    # first, try to solve the ricatti equation
#    X = np.matrix(la.solve_discrete_are(A, B, Q, R))
#
#    #compute the LQR gain
#    K = np.matrix(la.inv(B.T*X*B+R)*(B.T*X*A))
#
#    # eigVals, eigVecs = scipy.linalg.eig(A-B*K)
#
#    return K # , X, eigVals

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

    #=====================#
    #    Get Trajectory   #
    #=====================#

def GetTrajectory(W):

    global traj
    traj = W
    #rospy.loginfo("got trajectory")
    Datahandler()

    #=======================#
    #    NaN returns zero   #
    #=======================#

def IfNaN(X):

    if X != X:
         Y = 0
    else:
         Y = X
    return Y

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

def Fqx(x):
    global qx,Q,R,A,B,K
    qx = x
    Q[0][0]= qx
    K = dlqr(A,B,Q,R)
def Fqy(x):
    global qy,Q,R,A,B,K
    qy = x
    Q[1][1]= qy
    K = dlqr(A,B,Q,R)
def Fqz(x):
    global qz,Q,R,A,B,K
    qz = x
    Q[2][2]= qz
    K = dlqr(A,B,Q,R)
def Fqxdot(x):
    global qxdot,Q,R,A,B,K
    qxdot = x
    Q[3][3]= qxdot
    K = dlqr(A,B,Q,R)
def Fqydot(x):
    global qydot,Q,R,A,B,K
    qydot = x
    Q[4][4]= qydot
    K = dlqr(A,B,Q,R)
def Fqzdot(x):
    global qzdot,Q,R,A,B,K
    qzdot = x
    Q[5][5]= qzdot
    K = dlqr(A,B,Q,R)
def Fqpsi(x):
    global qpsi,Q,R,A,B,K
    qpsi = x
    Q[6][6]= qpsi
    K = dlqr(A,B,Q,R)
def Frphi(x):
    global rphi,Q,R,A,B,K
    rphi = x
    R[0][0]= rphi
    K = dlqr(A,B,Q,R)
def Frtheta(x):
    global rtheta,Q,R,A,B,K
    rtheta = x
    R[1][1]= rtheta
    K = dlqr(A,B,Q,R)
def Frpsi(x):
    global rpsi,Q,R,A,B,K
    rpsi = x
    R[2][2]= rpsi
    K = dlqr(A,B,Q,R)
def Fralt(x):
    global ralt,Q,R,A,B,K
    ralt = x
    R[3][3]= ralt
    K = dlqr(A,B,Q,R)

    #========================#
    #    Basic Controller    #
    #========================#

def Datahandler():
    global PI,K,traj,states,time_past,pub,image
    status = rospy.get_param('controller_status',True)
#    cv2.imshow("gains", image)
#    cv2.waitKey(3)
    rate = 1/(rospy.get_time()-time_past)

    #=======================#
    #    quad parameters    #
    #=======================#

    euler_max = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw",".3490659")) #in radians/sec
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
    m = .450 # ARDrone mass

    if len(traj.Obj) != 0:
        # Loop through all Bodies
        for j in range(bodies):

        #===================================#
        #    Get State Trajectory Errors    #
        #===================================#

             if states.Obj[j].visible:
                 X = np.asmatrix(np.zeros((7,1)))
                 X[0] = traj.Obj[j].x-states.Obj[j].x
                 X[1] = traj.Obj[j].y-states.Obj[j].y
                 X[2] = traj.Obj[j].z-states.Obj[j].z
                 X[3] = traj.Obj[j].xdot-states.Obj[j].u
                 X[4] = traj.Obj[j].ydot-states.Obj[j].v
                 X[5] = traj.Obj[j].zdot-states.Obj[j].w
                 X[6] = pi2pi(traj.Obj[j].psi)-states.Obj[j].psi*PI/180
        #============================================#
        #     Differential Flatness Control Input    #
        #============================================#

                # LQR input
                 utilde = -K*X
                # required input
                 u_r = np.matrix([[traj.Obj[j].xddot],[traj.Obj[j].yddot],[traj.Obj[j].zddot],[traj.Obj[j].psiddot]])
                 u = utilde-u_r

        #=============================#
        #     Rotate to Body Frame    #
        #=============================#

                 psi = states.Obj[j].psi*PI/180
                 rotZ = np.matrix([[cos(psi), -sin(psi), 0],[sin(psi), cos(psi), 0],[0, 0, 1]])
#                 rospy.loginfo('psi = %f', psi)

                 u[:-1] = rotZ*u[:-1]
                 # put desired acceleration into NED V1 frame


        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

                 T = sqrt(u[0,-1]*u[0,-1]+u[1,-1]*u[1,-1]+u[2,-1]*u[2,-1])*m
                 u[:-1] = np.divide(u[:-1],-T)
             #rospy.loginfo('z after = [%f]',u[2,-1])

        #==================#
        #   Set Controls   #
        #==================#

                # Controls for Ardrone
                # -phi = right... +phi = left
                # -theta = back... +theta = forward
                # -psi = right...  +psi = left
                 cmat[j].name  = states.Obj[j].name
                 cmat[j].phi   = atan2(u[1,-1],u[2,-1])*euler_max
                 cmat[j].theta = atan2(u[0,-1],u[2,-1])*euler_max
                 cmat[j].psi   = u[3,-1]/max_yaw_rate #PI*states.Obj[j].r/180+u[3]/rate
                 cmat[j].alt   = -3*utilde[2,-1]/g
                 Ctrl.Obj[j] = cmat[j]
        pub.publish(Ctrl)
    time_past = rospy.get_time()


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('LQR_controller')
    time_past = rospy.get_time()

    #=======================#
    #    Create Trackbars   #
    #=======================#

#    rospack = rospkg.RosPack()
#    path = rospack.get_path('risc_visual')
#    image = cv2.imread(path+"/mountains.jpg")
#    #cv2.resize(image,(321,123))
#    cv2.namedWindow("gains")
#    cv2.createTrackbar("qx", "gains", qx, Threshold, Fqx)
#    cv2.createTrackbar("qy", "gains", qy, Threshold, Fqy)
#    cv2.createTrackbar("qz", "gains", qz, Threshold, Fqz)
#    cv2.createTrackbar("qxdot", "gains", qxdot, Threshold/10, Fqxdot)
#    cv2.createTrackbar("qydot", "gains", qydot, Threshold/10, Fqydot)
#    cv2.createTrackbar("qzdot", "gains", qzdot, Threshold/10, Fqzdot)
#    cv2.createTrackbar("qpsi", "gains", qpsi, Threshold/10, Fqpsi)
#    cv2.createTrackbar("rphi", "gains", rphi, Threshold/10, Frphi)
#    cv2.createTrackbar("rtheta", "gains", rtheta, Threshold/10, Frtheta)
#    cv2.createTrackbar("ralt", "gains", ralt, Threshold/10, Fralt)
#    cv2.createTrackbar("rpsi", "gains", rpsi, Threshold/10, Frpsi)

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
    sub  = rospy.Subscriber('/cortex' , Cortex, GetStates)
    rospy.spin()

