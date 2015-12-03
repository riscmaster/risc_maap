#!/usr/bin/env python

'''======================================================
    Created by:  	Ishmaal Erekson
    Last updated: 	March 2015
    File name: 		Pendulum_EKF.py
    Organization:	RISC Lab, Utah State University

    Notes:
 ======================================================'''

import roslib roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from geometry_msgs.msg import PointStamped

    #===================#
    #      Globals      #
    #===================#

    #---------------#
    #   Constants   #
    #---------------#

flag          = 0             # denotes whether an initial measurement has been recieved
g             = 9.8067        # gravitational acceleration m/s^2
L             = 1             # Length of Pendulum in meters
m             = .45           # kg
rate          = 45            # Hz Control loop runs at 45 Hz
quad_acc      = PointStamped()

# Standard deviations of System
Q  = (.004)**2*np.asmatrix(np.eye(4))

# Standard deviations of Cortex Data
Rk = (.002)**2*np.asmatrix(np.eye(4))

# Initial Covariance
P_est = np.asmatrix(np.eye(4))

    #----------#
    #   Time   #
    #----------#

start_time    = 0
delay         = 0
h = std_msgs.msg.Header()
object_message_time = 0

    #-----------#
    #   Input   #
    #-----------#

ctrl_size     = int (ceil(delay*rate))
if ctrl_size == 0:
    ctrl_size = 1
ctrl_in       = [Control()]*ctrl_size

    #-------------------------------------#
    #   Initialize Global State Variables #
    #-------------------------------------#

states_in = States()
quad_est  = np.asmatrix(np.zeros((12,1)))
pen_est   = np.asmatrix(np.zeros((4,1)))

    #----------------#
    #   Publishers   #
    #----------------#

pub = rospy.Publisher('/Pen_ekf', PenWithCov, queue_size = rate)

    #------------------------------#
    #   Matrix to list converter   #
    #------------------------------#

def mat2list(mat)
    return np.asarray(a).reshape(-1).tolist()

    #==============================#
    #    Executed Upon Shutdown    #
    #==============================#

def shutdownFunction():

    rospy.loginfo("Extended Kalman Filter Shutting Down...")


    #===========================#
    #    Get State Estimates    #
    #===========================#

def Pend_Jacobian(x_est):
    global g, L, acc_est
    xv     = x_est[1]
    yv     = x_est[2]
    dxv    = x_est[3]
    dyv    = x_est[4]
    ddx    = acc_est.point.x
    ddy    = acc_est.point.y
    ddz    = acc_est.point.z
   
    zeta   = sqrt(L**2 - xv**2 - yv**2)
    alpha  = zeta**2/(zeta**2 + xv**2)*(-ddx + ddz*xv/zeta - dxv**2*xv/zeta**2 - dyv**2*xv/zeta**2 - dxv**2*xv**3/zeta**4 - 2*dxv*dyv*xv**2*yv/zeta**4 - dyv**2*yv**2*xv/zeta**4 + P.g*xv/zeta) 
    beta   = zeta**2/(zeta**2 + yv**2)*(-ddy + ddz*yv/zeta - dyv**2*yv/zeta**2 - dxv**2*yv/zeta**2 - dyv**2*yv**3/zeta**4 - 2*dyv*dxv*yv**2*xv/zeta**4 - dxv**2*xv**2*yv/zeta**4 + P.g*yv/zeta) 
         
    ddxv   = (alpha*(zeta**2 + xv**2)*(zeta**2 + yv**2) - beta*xv*yv*(zeta**2 + yv**2))/(zeta**2*(zeta**2 + xv**2 + yv**2)) 
    daxv   = (2*xv*ddx - dxv**2 - dyv**2)/(zeta**2 + xv**2) + (ddz + P.g)*(zeta**2 - xv**2)/(zeta*(zeta**2 + xv**2)) - (dxv*xv + dyv*yv)*(2*dxv*xv**3 + 2*dyv*yv*xv**2 + 3*dxv*xv*zeta**2 + dyv*yv*zeta**2)/(zeta**4*(zeta**2 + xv**2))  
    dayv   = (2*xv*yv)*(xv*ddx - dxv**2 - dyv**2)/(zeta**2 + xv**2)**2 + xv*yv*(ddz + P.g)*(zeta**2 - xv**2)/(zeta*(zeta**2+xv**2)**2) - ((4*zeta**2*yv + 2*yv*xv**2)*(dxv**2*xv**3 + 2*dxv*dyv*xv**2*yv + dyv**2*yv**2*xv) + zeta**2*(zeta**2 + xv**2)*(2*dxv*dyv*xv**2 + 2*dyv**2*yv*xv))/(zeta**4*(zeta**2+xv**2)**2) 
    dadxv  = -2*dxv*xv/(zeta**2 + xv**2) - 2*dxv*xv**3/(zeta**4 + zeta**2 * xv**2) - 2*dyv*xv**2*yv/(zeta**4 + zeta**2*xv**2) 
    dadyv  = -2*dyv*xv/(zeta**2 + xv**2) - 2*dxv*xv**2*yv/(zeta**4 + zeta**2 * xv**2) - 2*dyv*xv*yv**2/(zeta**4 + zeta**2*xv**2) 
    
    dbxv   = 2*xv*yv*(ddy*yv - dyv**2 - dxv**2)/(zeta**2 + yv**2)**2 + xv*yv*(ddz + P.g)*(zeta**2 - yv**2)/(zeta*(zeta**2 + yv**2)**2) - 2*yv*(dxv*xv + dyv*yv)*(dxv*xv**2*yv**2 + 2*dxv*xv**2*zeta**2 + dyv*xv*yv**3 + 2*dyv*xv*yv*zeta**2 + dxv*yv**2*zeta**2 + dxv*zeta**4)/(zeta**4*(zeta**2 + yv**2)**2) 
    dbyv   = (2*ddy*yv - dyv**2 - dxv**2)/(zeta**2 + yv**2) + (ddz + P.g)*(zeta**2 - yv**2)/(zeta*(zeta**2 + yv**2)) - (dxv*xv + dyv*yv)*(2*dyv*yv**3 + 2*dxv*xv*yv**2 + 3*dyv*yv*zeta**2 + dxv*xv*zeta**2)/(zeta**4*(zeta**2 + yv**2)) 
    dbdxv  = -2*yv*(dxv*(zeta**2 + xv**2) + dyv*yv*xv)/(zeta**2*(zeta**2 + yv**2)) 
    dbdyv  = -2*yv*(dyv*(zeta**2 + yv**2) + dxv*yv*xv)/(zeta**2*(zeta**2 + yv**2)) 

    # Partials of ddx_v
    ddxxv  = daxv*(zeta**2 + xv**2)*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) + alpha*2*xv*yv**2*(zeta**2 + xv**2)/(zeta**4*(zeta**2 + xv**2 + yv**2)) - dbxv*xv*yv*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) - beta*yv*(zeta**4 + zeta**2*yv**2 - 2*zeta**2*xv**2 + 2*zeta**2*xv*yv + 2*xv*yv**3)/(zeta**4*(zeta**2 + xv**2 + yv**2)) 
    ddxyv  = dayv*(zeta**2 + xv**2)*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) + alpha*2*(zeta**2 + xv**2)*(xv**2*yv - zeta**2*xv + zeta**2*yv)/(zeta**4*(zeta**2 + xv**2 + yv**2)) - dbyv*xv*yv*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) - beta*(zeta**2 + yv**2)*(zeta**2*xv + 2*xv*yv**2)/(zeta**4*(zeta**2 + xv**2 + yv**2)) 
    ddxdxv = dadxv*(zeta**2 + xv**2)*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) - dbdxv*xv*yv*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) 
    ddxdyv = dadyv*(zeta**2 + xv**2)*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) - dbdyv*xv*yv*(zeta**2 + yv**2)/(zeta**2*(zeta**2 + xv**2 + yv**2)) 

    # Partials of ddy_v
    ddyxv  = dbxv - ddxxv*(xv*yv)/(zeta**2 + yv**2) - ddxv*(zeta**2*yv + yv**3 + 2*xv**2*yv)/(zeta**2 + yv**2)**2 
    ddyyv  = dbyv - ddxyv*(xv*yv)/(zeta**2 + yv**2) - ddxv*xv/(zeta**2 + yv**2) 
    ddydxv = dbdxv - ddxdxv*(xv*yv)/(zeta**2 + yv**2) 
    ddydyv = dbdyv - ddxdyv*(xv*yv)/(zeta**2 + yv**2) 
    
    F = np.matrix([[0,0,1,0],[0,0,0,1],[ddxxv,ddxyv,ddxdxv,ddxdyv],[ddyxv,ddyyv,ddydxv, ddydyv]])
    return F

    #======================#
    #    Equality Check    #
    #======================#

def StatesEqual(s1, s2):
    if (s1.x != s2.x) or (s1.y != s2.y) or (s1.z != s2.z) or (s1.u != s2.u) or (s1.v != s2.v) or (s1.w != s2.w) or (s1.phi != s2.phi) or (s1.theta != s2.theta) or (s1.psi != s2.psi) or (s1.p != s2.p) or (s1.q != s2.q) or (s1.r != s2.r):
        return 0
    else:
        return 1

    #===========================#
    #    Get State Estimates    #
    #===========================#

def GetStates(I):
    global states_in,quad_est,flag,pen_est
    # check to see if cortex is frozen
    Equal = StatesEqual(states_in,  I.Obj[0])
    if not Equal:
        states_in = I.Obj[0]
        if I.obj[0].visible and I.obj[1].visible
            pen_vis =  True
        else
            pen_vis = False

        if flag == 0 and I.Obj[0].visible: # initialize estimate
            rospy.loginfo("%s's state initialized",I.Obj[0].name)
            quad_est[0]  = I.Obj[0].x
            quad_est[1]  = I.Obj[0].y
            quad_est[2]  = I.Obj[0].z
            quad_est[3]  = I.Obj[0].u
            quad_est[4]  = I.Obj[0].v
            quad_est[5]  = I.Obj[0].w
            quad_est[6]  = I.Obj[0].phi*np.pi/180
            quad_est[7]  = I.Obj[0].theta*np.pi/180
            quad_est[8]  = I.Obj[0].psi*np.pi/180
            quad_est[9]  = I.Obj[0].p*np.pi/180
            quad_est[10] = I.Obj[0].q*np.pi/180
            quad_est[11] = I.Obj[0].r*np.pi/180
            pen_est[0]   = I.Obj[1].x - quad_est[0]
            pen_est[1]   = I.Obj[1].y - quad_est[1]
            pen_est[2]   = I.Obj[1].u - quad_est[3]
            pen_est[3]   = I.Obj[1].v - quad_est[4]
            flag = 1
    elif Equal:
        pass # redundant data  (do not use)
    else:
        rospy.log("strange stuff")

def GetAcc(I):
    global acc_est
    acc_est = I

    #===============================#
    # Pendulum Equations of Motion  #
    #===============================#

def Pendulum():
    global g, L, acc_est, pen_est

    xv     = pen_est[0,-1]
    yv     = pen_est[1,-1]
    dxv    = pen_est[2,-1]
    dyv    = pen_est[3,-1]

    #------------------------#
    #   Quad Accelerations   #
    #------------------------#

    ddx    = acc_est.point.x
    ddy    = acc_est.point.y
    ddz    = acc_est.point.z

    #--------------#
    #   Velocity   #
    #--------------#

    dxdt = np.asmatrix(np.zeros((4,1)))
    dxdt[0] = dxv
    dxdt[1] = dyv

    #-----------------------------#
    #   Pen State Accelerations   #
    #-----------------------------#

    zeta   = sqrt(L**2 - xv**2 - yv**2)
    alpha  = zeta**2/(zeta**2 + xv**2)*(-ddx + ddz*xv/zeta - dxv**2*xv/zeta**2 - dyv**2*xv/zeta**2 - dxv**2*xv**3/zeta**4 - 2*dxv*dyv*xv**2*yv**2/zeta**4 - dyv**2*yv**2*xv/zeta**4 + g*xv/zeta) 
    beta   = zeta**2/(zeta**2 + yv**2)*(-ddy + ddz*yv/zeta - dyv**2*yv/zeta**2 - dxv**2*yv/zeta**2 - dyv**2*yv**3/zeta**4 - 2*dyv*dxv*yv**2*xv**2/zeta**4 - dxv**2*xv**2*yv/zeta**4 + g*yv/zeta)
    dxdt[2] = (alpha*(zeta**2 + xv**2)*(zeta**2 + yv**2) - beta*xv*yv*(zeta**2 + yv**2))/(zeta**2*(zeta**2 + xv**2 + yv**2))
    dxdt[3] = beta - dxdt[2]*xv*yv/(zeta**2 + yv**2)

    return dxdt

    #==================#
    #    EKF Driver    #
    #==================#

def Driver():
    global rate,pub,start_time, states_in, quad_est, quad_acc, pen_est, P_est,eye

    if flag == 1:
        for i in range(len(ctrl_in)):

        #===============#
        #    Predict    #
        #===============#

            dy = Pendulum()
            pen_xhat = pen_est + dy/rate 
            F = np.asmatrix(np.eye(4)) + Pend_Jacobian(pen_est)/rate
            global Q
            Phat = F*P_est*F.T + Q/rate 

            if i == 0 and states_in.visible and states_in == states_in: # then use the measurement

        #==============#
        #    Update    #
        #==============#
                deg2rad = np.pi/180

                # measurement
                zk = np.matrix([[pen_states_in.x],[pen_states_in.y],[pen_states_in.u],[pen_states_in.v]])
                
                # residual/innovation
                yk = zk - xhat
                
                # residual/innovation Covariance
                global Rk
                Sk = Phat + Rk 
                
                # Kalman Gain
                Kk = Phat*np.linalg.inv(Sk) 
                x_est = xhat + Kk*yk
                P_est = (eye-Kk)*Phat 
            else:
                x_est = xhat
                P_est = Phat

        #--------------#
        # Make Message #
        #--------------#

        pendulum = PenWithCov()
        pendulum.visible = pen_vis
        global h
        h.seq =  h.seq + 1
        h.stamp = rospy.Time.now()
        h.frame_id = 'Pendulum_With_Covariance'
        pendulum.header = h
        pendulum.x    = x_est[0]
        pendulum.y    = x_est[1]
        pendulum.xdot = x_est[3]
        pendulum.ydot = x_est[4]
        pendulum.cov  = mat2list(P_est)

        #---------#
        # Publish #
        #---------#
        pub_cov.publish(pendulum)

    else:
        global object_message_time
        if (rospy.get_time()-object_message_time)>2:
            rospy.logwarn("no object available...")
            object_message_time = rospy.get_time()

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Pendulum_EKF')
    start_time = rospy.get_time()

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         states   = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)        #Pen and Quad INERTIAL
         quad_acc = rospy.Subscriber('/quad_accelerations', PointStamped, GetAcc)  #Pendulum Accelerations
         Driver()
         r.sleep()
    rospy.signal_shutdown(shutdownFunction)
