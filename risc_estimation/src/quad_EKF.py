#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		quad_EKF.py
    Organization:	RISC Lab, Utah State University

    Notes:
 ======================================================'''

import roslib; roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped

    #===================#
    #      Globals      #
    #===================#

    #---------------#
    #   Constants   #
    #---------------#

g             = 9.8067
m             = 1.378 #.45 #kg
rate          = 22.5 # Hz Control loop runs at 45 Hz
euler_max     = 45*np.pi/180 # radians
yaw_rate_max  = 45*np.pi/180 # radians/sec
acc           = PointStamped()
# Standard deviations of System
Q             = (.004)**2*np.asmatrix(np.eye(12))
# Standard deviations of Cortex Data
Rk_cort       = (.003)**2*np.ones(6)
Rk_angl       = (1.*np.pi/180)**2*np.ones(6)
Rk_tot        = np.concatenate((Rk_cort,Rk_angl),axis = 0)
Rk = np.asmatrix(np.diag(Rk_tot))
# Initial Covariance
P_est         = 3*np.asmatrix(np.eye(12))
eye           = np.asmatrix(np.eye(12))

    #----------#
    #   Time   #
    #----------#

h = std_msgs.msg.Header()

    #-----------#
    #   Input   #
    #-----------#

ctrl_in       = Control()

    #-------------------------------------#
    #   Initiialize Global State Variable #
    #-------------------------------------#

states_in = States()
x_est     = np.asmatrix(np.zeros((12,1)))

    #----------------#
    #   Publishers   #
    #----------------#

pub           = rospy.Publisher('/cortex_ekf', Cortex, queue_size = 1)
pub_acc       = rospy.Publisher('/ekf_acc', PointStamped, queue_size = rate)
pub_cov       = rospy.Publisher('/position_cov', TwistStamped, queue_size = 1)

    #----------------------------------------------------#
    #           ARDrone Inertial Coefficients            #
    #        and Attitude PID gains without hull         #
    #----------------------------------------------------#

# These can be modified through the launch file
Ixx      = 0.0 #Kg*m^2
Iyy      = 0.0 #Kg*m^2
Izz      = 0.0 #Kg*m^2

roll_kp     = 0.0
roll_kd     = 0.0
roll_ki     = 0.0
pitch_kp    = 0.0
pitch_kd    = 0.0
pitch_ki    = 0.0
yaw_kp      = 0.0
yaw_kg      = 0.0
status = False

    #=============================#
    #    New Measurement Check    #
    #=============================#

def CheckInnovation(x_hat,states_in,P):
    global x_est
    new_measurement = True
    deg2rad = np.pi/180
    x_m = np.matrix([[states_in.x],[-states_in.y],[-states_in.z],[states_in.u],[-states_in.v],[-states_in.w],[states_in.phi*deg2rad],[states_in.theta*deg2rad],[states_in.psi*deg2rad],[states_in.p*deg2rad],[states_in.q*deg2rad],[states_in.r*deg2rad]])
    #if (np.absolute(x_m-x_hat)<5*np.diag(P)).all():
    #    new_measurement = True
    #else:
    #    new_measurement = False
    if (x_m != x_m).all():
        new_measurement = False
    if (np.linalg.norm(x_m[0:3])>3):
        new_measurement = False
    return new_measurement

    #==============================#
    #    Executed Upon Shutdown    #
    #==============================#

def shutdownFunction():

    rospy.loginfo("Extended Kalman Filter Shutting Down...")

    #=============================#
    #    Get Controller Status    #
    #=============================#

def GetStatus(X):
    global status
    status = X.data

    #================================================#
    #    Rotation Matrix from Body to World Frame    #
    #================================================#

def rotB2W(phi, theta, psi):
    cT = cos(theta)
    cF = cos(phi)
    cP = cos(psi)
    sT = sin(theta)
    sF = sin(phi)
    sP = sin(psi)
    ROT = np.asmatrix(np.array([[cT*cP, sF*sT*cP-cF*sP, cF*sT*cP+sF*sP],\
                                [cT*sP, sF*sT*sP+cF*cP, cF*sT*sP-sF*cP],\
                                [  -sT,          sF*cT,          cF*cT]]))
    return ROT

    #===========================#
    #    Get State Estimates    #
    #===========================#

def Quad_Jacobian(x_est):
#    useful definitions
    global g
    u     = x_est[3]
    v     = x_est[4]
    w     = x_est[5]
    phi   = x_est[6]
    theta = x_est[7]
    psi   = x_est[8]
    p     = x_est[9]
    q     = x_est[10]
    r     = x_est[11]
    cT    = cos(theta)
    cF    = cos(phi)
    cP    = cos(psi)
    sT    = sin(theta)
    sF    = sin(phi)
    sP    = sin(psi)
    tT    = sT/cT
    tF    = sF/cF
    tP    = sP/cP
    secT  = 1/cT
    sss   = sF*sT*sP
    ssc   = sF*sT*cP
    scc   = sF*cT*cP
    ccc   = cF*cT*cP
    ccs   = cF*cT*sP
    css   = cF*sT*sP
    csc   = cF*sT*cP
    scs   = sF*cT*sP
    # x/north partials
    dxdu  = cT*cP
    dxdv  = ssc-cF*sP
    dxdw  = csc-sF*sP
    dxdF  = (csc+sF*sP)*v + (cF*sP - ssc)*w
    dxdT  = -sT*cP*u + scc*v + ccc*w
    dxdP  = -cT*sP*u + (-sss-cF*cP)*v -css*w + sF*cP*w
    # y/east partials
    dydu  = cT*cP
    dydv  = sss+cF*cP
    dydw  = css-sF*cP
    dydF  = css*v - sF*cP*v - sss*w + sF*sP*w
    dydT  = -sT*sP*u +scs*v + ccs*w
    dydP  = cT*cP*u + sF*sT*cP*v - cF*sP*v + csc*w + sF*sP*w
    # z/down partials
    dzdu  = sT
    dzdv  = -sF*cT
    dzdw  = -cF*cT
    dzdF  = -cF*cT*v + sF*cT*w
    dzdT  = cT*u + sF*sT*v + cF*sT*w
    dzdP  = 0
    #xdot partials
    dxddu  = 0
    dxddv  = r
    dxddw  = -q
    dxddF  = 0
    dxddT  = -g*cT
    dxddP  = 0
    dxddp  = 0
    dxddq  = -w
    dxddr  = v
    #ydot partials
    dyddu  = -r
    dyddv  = 0
    dyddw  = p
    dyddF  = g*cT*cF
    dyddT  = -g*sT*sF
    dyddP  = 0
    dyddp  = w
    dyddq  = 0
    dyddr  = -u
    #zdot partials
    dzddu  = q
    dzddv  = -p
    dzddw  = 0
    dzddF  = -g*cT*sF
    dzddT  = -g*sT*cF
    dzddP  = 0
    dzddp  = -v
    dzddq  = u
    dzddr  = 0
    #Phi partials
    dFdu  = 0
    dFdv  = 0
    dFdw  = 0
    dFdF  = cF*tT*q - sF*tT*r
    dFdT  = sF*secT*secT*q + cF*secT*secT*r
    dFdP  = 0
    dFdp  = 1
    dFdq  = sF*tT
    dFdr  = cF*tT
    #Theta partials
    dTdu  = 0
    dTdv  = 0
    dTdw  = 0
    dTdF  = -sF*q-cF*r
    dTdT  = 0
    dTdP  = 0
    dTdp  = 0
    dTdq  = cF
    dTdr  = -sF
    #Psi partials
    dPdu  = 0
    dPdv  = 0
    dPdw  = 0
    dPdF  = cF*secT*q - sF*secT*r
    dPdT  = (sF*q+cF*r)*tT*secT
    dPdP  = 0
    dPdp  = 0
    dPdq  = sF*secT
    dPdr  = cF*secT

    F     = np.asmatrix(np.array([\
            [ 0, 0, 0,  dxdu,  dxdv,  dxdw,  dxdF,  dxdT,  dxdP,     0,     0,     0],\
            [ 0, 0, 0,  dydu,  dydv,  dydw,  dydF,  dydT,  dydP,     0,     0,     0],\
            [ 0, 0, 0,  dzdu,  dzdv,  dzdw,  dzdF,  dzdT,  dzdP,     0,     0,     0],\
            [ 0, 0, 0, dxddu, dxddv, dxddw, dxddF, dxddT, dxddP, dxddp, dxddq, dxddr],\
            [ 0, 0, 0, dyddu, dyddv, dyddw, dyddF, dyddT, dyddP, dyddp, dyddq, dyddr],\
            [ 0, 0, 0, dzddu, dzddv, dzddw, dzddF, dzddT, dzddP, dzddp, dzddq, dzddr],\
            [ 0, 0, 0,  dFdu,  dFdv,  dFdw,  dFdF,  dFdT,  dFdP,  dFdp,  dFdq,  dFdr],\
            [ 0, 0, 0,  dTdu,  dTdv,  dTdw,  dTdF,  dTdT,  dTdP,  dTdp,  dTdq,  dTdr],\
            [ 0, 0, 0,  dPdu,  dPdv,  dPdw,  dPdF,  dPdT,  dPdP,  dPdp,  dPdq,  dPdr],\
            [0,0,0,0,0,0,0,0,0,1,0,0],\
            [0,0,0,0,0,0,0,0,0,0,1,0],\
            [0,0,0,0,0,0,0,0,0,0,0,1],\
            ]))
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
    global states_in
    states_in = I.Obj[0]

    #=================#
    #    Get Input    #
    #=================#

def GetInput(I):
    global ctrl_in
    ctrl_in = I.Obj[0]

    #=====================#
    #    PID Controller   #
    #=====================#

def pidControl(kp,kd,ki,act,des,vel):
    global rate
    err = des-act
    u = kp*err - kd*vel + ki*err/rate
    return u


    #===============================#
    #    Quad Equations of Motion   #
    #===============================#

def Quad(i):
    global g,ctrl_in,Ixx,Iyy,Izz,euler_max,yaw_rate_max,alt_rate_max,ctrl_size,m,x_est, T_kp

    x     = x_est[0,-1]
    y     = x_est[1,-1]
    z     = x_est[2,-1]
    u     = x_est[3,-1]
    v     = x_est[4,-1]
    w     = x_est[5,-1]
    phi   = x_est[6,-1]
    theta = x_est[7,-1]
    psi   = x_est[8,-1]
    p     = x_est[9,-1]
    q     = x_est[10,-1]
    r     = x_est[11,-1]

    #------------#
    #   Inputs   #
    #------------#

    phi_c   = ctrl_in.phi*euler_max # roll
    theta_c = ctrl_in.theta*euler_max # pitch
    r_c     = ctrl_in.psi*yaw_rate_max # yaw_rate
    T       =  (ctrl_in.T + 1)*g
    #--------------#
    #   Velocity   #
    #--------------#

    dxdt = np.asmatrix(np.zeros((12,1)))
    dxdt[0] = u
    dxdt[1] = v
    dxdt[2] = w

    #------------------#
    #   Acceleration   #
    #------------------#

    global roll_ki,pitch_ki,acc
    body_frame_acceleration = np.matrix([[0],[0],[-T]])  # DIVIDE BY MASS?
    dxdt[3:6] = rotB2W(phi, theta, psi)*body_frame_acceleration*np.asmatrix(np.diag([[1],[1],[-1]])) - np.matrix([[0],[0],[g]])
    acc.point.x = dxdt[3,-1]
    acc.point.y = dxdt[4,-1]
    acc.point.z = dxdt[5,-1]

    #----------------------#
    #   Angular Velocity   #
    #----------------------#

    # Gyro to Body Rotation
    G2B = np.matrix([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],\
                     [0,            cos(phi),           -sin(phi)],\
                     [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])
    dxdt[6:9] = G2B*np.matrix([[p],[q],[r]])

    #--------------------------#
    #   Angular Acceleration   #
    #--------------------------#

    global roll_kp,roll_kd,pitch_kp,pitch_kd,yaw_kp,yaw_ki
    tauPhi   = pidControl( roll_kp, roll_kd, roll_ki,  phi,  phi_c, p)
    tauTheta = pidControl(pitch_kp,pitch_kd,pitch_ki,theta,theta_c, q)
    tauPsi   = pidControl(yaw_kp,0,0,r,r_c,0)

    dxdt[9] =  (q*r*(Iyy-Izz)/Ixx) + tauPhi/Ixx
    dxdt[10] = (p*r*(Izz-Ixx)/Iyy) + tauTheta/Iyy
    dxdt[11] = (p*q*(Ixx-Iyy)/Izz) + tauPsi/Izz

    return dxdt

    #==================#
    #    EKF Driver    #
    #==================#

def Driver():
    global rate,pub,start_time,states_in, x_est, P_est,eye,Q,Rk,status

    if status:

        #===============#
        #    Predict    #
        #===============#


        dy = Quad(0)
        xhat = x_est + dy/rate
        F = eye + Quad_Jacobian(x_est)/rate
        Phat = F*P_est*F.T + Q/rate
        New_measurement = CheckInnovation(xhat,states_in,Phat)

        if New_measurement: # then use the measurement

        #==============#
        #    Update    #
        #==============#
            deg2rad = np.pi/180
            # measurement
            zk = np.matrix([[states_in.x],[-states_in.y],[-states_in.z],[states_in.u],[-states_in.v],[-states_in.w],[states_in.phi*deg2rad],[states_in.theta*deg2rad],[states_in.psi*deg2rad],[states_in.p*deg2rad],[states_in.q*deg2rad],[states_in.r*deg2rad]])
            # residual/innovation
            yk = zk - xhat
            # residual/innovation Covariance
            Sk = Phat + Rk
            # Kalman Gain
            Kk = Phat*np.linalg.inv(Sk)
            x_est = xhat + Kk*yk
            P_est = (eye-Kk)*Phat
        else:
            x_est = xhat
            P_est = Phat

        #=================================================#
        #    Check if Estimate is actually within Room    #
        #=================================================#

        if np.linalg.norm(x_est[0:3]) >6 or (x_est != x_est).all():
            x_est = np.asmatrix(np.zeros((12,1)))
            x_est[0:3] = np.matrix([[0],[0],[-2]])
            P_est = 3*np.asmatrix(np.eye(12))
        #--------------#
        # Make Message #
        #--------------#

    cortex = Cortex()
    cortex.Obj = [States()]*1
    states = States()
    states.name = states_in.name
    states.visible = states_in.visible
    global h
    h.seq =  h.seq + 1
    h.stamp = rospy.Time.now()
    h.frame_id = 'cortex'
    cortex.header = h
    rad2deg = 180/np.pi
    # Invert y and z position and velocity from NED to Cartesian
    states.x      = x_est[0]
    states.y      = -x_est[1]
    states.z      = -x_est[2]
    states.u      = x_est[3]
    states.v      = -x_est[4]
    states.w      = -x_est[5]
    states.phi    = rad2deg*x_est[6]
    states.theta  = rad2deg*x_est[7]
    states.psi    = rad2deg*x_est[8]
    states.p      = rad2deg*x_est[9]
    states.q      = rad2deg*x_est[10]
    states.r      = rad2deg*x_est[11]
    cortex.Obj[0] = states

    global acc
    acc.header = h

    cov = TwistStamped()
    cov.header = h
    cov.twist.linear.x = x_est[0]
    cov.twist.linear.y = -x_est[1]
    cov.twist.linear.z = -x_est[2]
    cov.twist.angular.x = P_est[0,0]
    cov.twist.angular.y = P_est[1,1]
    cov.twist.angular.z = P_est[2,2]

    #---------#
    # Publish #
    #---------#
    pub_cov.publish(cov)
    pub_acc.publish(acc)
    pub.publish(cortex)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('EKF')
    start_time = rospy.get_time()

    #------------------------------------------#
    #    Set up Gains and Saturation values    #
    #------------------------------------------#

    # Saturation Values
    euler_max     = float ( rospy.get_param("~euler_angle_max","0.785398" ) )
    yaw_rate_max  = float ( rospy.get_param("~control_yaw","0.785398" ) )
    # These are unscientific guesses until we can make reasonable measurements
    Ixx           = float ( rospy.get_param("~Ixx","0.05") )
    Iyy           = float ( rospy.get_param("~Iyy","0.025") )
    Izz           = float ( rospy.get_param("~Izz","0.075") )
    # Experimental System ID done by Spencer Maughan December 2014
    # Details can be found in a paper currently being written
    roll_kp       = float ( rospy.get_param("~roll_kp",".2") )
    roll_ki       = float ( rospy.get_param("~roll_ki","0.1") )
    roll_kd       = float ( rospy.get_param("~roll_kd",".4") )
    pitch_kp      = float ( rospy.get_param("~pitch_kp","0.0944") )
    pitch_ki      = float ( rospy.get_param("~pitch_ki","0.1") )
    pitch_kd      = float ( rospy.get_param("~pitch_kd","0.404206293307772") )
    yaw_kp        = float ( rospy.get_param("~yaw_kp","0.5") )
    yaw_kg        = float ( rospy.get_param("~yaw_kg","1") )
    object_message_time = rospy.get_time()

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
         rospy.Subscriber('/controls' , Controls, GetInput)
         rospy.Subscriber('/controller_status' , Bool, GetStatus)
         Driver()
         r.sleep()
    rospy.signal_shutdown(shutdownFunction)
