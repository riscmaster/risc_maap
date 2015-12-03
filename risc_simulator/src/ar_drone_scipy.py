#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		ar_drone_scipy.py
    Organization:	RISC Lab, Utah State University

    Notes:
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import cv2
import time
import Queue
from scipy import integrate

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist      # for sending commands to the drone

    #===================#
    #      Globals      #
    #===================#

    #---------------#
    #   Constants   #
    #---------------#

PI 	      = 3.141592653589793
g             = 9.8067
m             = .45 #kg
step          = 10 # times per loop
rate          = 150 # Hz
euler_max     = 0.349066 # radians
yaw_rate_max  = 0.3490659 # radians/sec
alt_rate_max  = 1000 # mm/sec

    #----------#
    #   Time   #
    #----------#

start_time    = 0
delay         = 0.0
h = std_msgs.msg.Header()

    #-----------#
    #   Input   #
    #-----------#

ctrl_size     = int (ceil(delay*rate))
if ctrl_size <= 1:
    ctrl_size = 2
ctrl_in       = [Control()]*ctrl_size
command       = False

    #-------------------------#
    #   Initiial Conditions   #
    #-------------------------#

states        = np.array([0,0,1,0,0,0,0,0,0,0,0,0])

    #----------------#
    #   Publishers   #
    #----------------#

pub           = rospy.Publisher('/cortex_raw', Cortex, queue_size = 1)
pub_acc       = rospy.Publisher('/ardrone_acc', PointStamped, queue_size = 1)

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
roll_kg     = 0.0
pitch_kp    = 0.0
pitch_kd    = 0.0
pitch_kg    = 0.0
yaw_kp      = 0.0
yaw_kg      = 0.0
w_old       = 0.0

    #==============================#
    #    Executed Upon Shutdown    #
    #==============================#

def shutdownFunction():

    rospy.loginfo("AR.Drone Simulation Shutting Down...")


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

    #=================#
    #    Get Input    #
    #=================#

def GetInput(I):
    global ctrl_in,ctrl_size,command
    command = True
    C = Control()
    C.theta = I.linear.x
    C.phi   = I.linear.y
    C.T     = I.linear.z
    C.psi   = I.angular.z
    #ctrl_in = ctrl_in[len(ctrl_in)-ctrl_size+1:len(ctrl_in)]
    if len(ctrl_in) == ctrl_size-1:
        ctrl_in.append(C)


    #=====================#
    #    PID Controller   #
    #=====================#

def pidControl(kp,kd,kg,act,des,vel):
    err = kg*des-act
    u = kp*err - kd*vel
    return u


    #===================================#
    #    AR.Drone Equations of Motion   #
    #===================================#

def AR_Drone(X,t=0):
    global C,g,Ixx,Iyy,Izz,euler_max,yaw_rate_max,alt_rate_max,ctrl_size,m,w_old, T_kp
    x=X[0]
    y=X[1]
    z=X[2]
    u=X[3]
    v=X[4]
    w=X[5]
    phi=X[6]
    theta=X[7]
    psi=X[8]
    p=X[9]
    q=X[10]
    r=X[11]
    #------------#
    #   Inputs   #
    #------------#

    phi_c   = -C.phi*euler_max # roll
    theta_c = -C.theta*euler_max # pitch
    r_c     = -C.psi*yaw_rate_max # yaw_rate
    zdot_c  = C.T


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

    global roll_kg,pitch_kg
    Z_body_acceleration = ((zdot_c - w)/.3+g)/(cos(roll_kg*phi_c)*cos(pitch_kg*theta_c))
    body_frame_acceleration = np.matrix([[0],[0],[Z_body_acceleration]])

    dxdt[3:6] = np.multiply((rotB2W(-phi,theta,psi)*body_frame_acceleration-np.matrix([[0],[0],[g]])),np.matrix([[-1],[-1],[1]]))
    acc = PointStamped()
    acc.point.x = dxdt[3,-1]
    acc.point.y = dxdt[4,-1]
    acc.point.z = -dxdt[5,-1]
    pub_acc.publish(acc)

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
    tauPhi   = pidControl( roll_kp, roll_kd, roll_kg,  phi,  phi_c, p)
    tauTheta = pidControl(pitch_kp,pitch_kd,pitch_kg,theta,theta_c, q)
    tauPsi   = pidControl(yaw_kp,0,yaw_kg,r,r_c,0)

    dxdt[9]  = (q*r*(Iyy-Izz)/Ixx) + tauPhi/Ixx
    dxdt[10] = (p*r*(Izz-Ixx)/Iyy) + tauTheta/Iyy
    dxdt[11] = (p*q*(Ixx-Iyy)/Izz) + tauPsi/Izz

    return np.asarray(dxdt)

    #==================#
    #    ODE Driver    #
    #==================#

def Driver():
    global PI,rate,pub,start_time,states,step

    #---------------#
    # one unit step #
    #---------------#

    t = np.linspace(0,1/rate,step)
    print t
    global ctrl_in,C
    C = ctrl_in.pop(0)
    X,infodict = integrate.odeint(AR_Drone,states,t,full_output=True)

    #------------------------#
    # propogate given the dy #
    #------------------------#

    n = .002 # meters of noise
    a = .01 # Degrees of noise
    states[0]     = X[0] +np.random.normal(0,n*n,1)[0]
    states[1]     = X[1] +np.random.normal(0,n*n,1)[0]
    states[2]     = X[2] +np.random.normal(0,n*n,1)[0]
    states[3]     = X[3] +np.random.normal(0,n*.3,1)[0]
    states[4]     = X[4] +np.random.normal(0,n*.3,1)[0]
    states[5]     = X[5] +np.random.normal(0,n*.3,1)[0]
    states[6]     = X[6] +np.random.normal(0,a*a*PI/180,1)[0]
    states[7]     = X[7] +np.random.normal(0,a*a*PI/180,1)[0]
    states[8]     = X[8] +np.random.normal(0,a*a*PI/180,1)[0]
    states[9]     = X[9] +np.random.normal(0,a*a*PI/180,1)[0]
    states[10]    = X[10]+np.random.normal(0,a*a*PI/180,1)[0]
    states[11]    = X[11]+np.random.normal(0,a*a*PI/180,1)[0]

    # Saturation Bounds
    if states[2] < 0.05:
        states[2] = 0.05
    if states[2] > 3:
        states[2] = 3

    #-----------------------------------------------------#
    # Crash if the quad moves out of the RISC MAAP region #
    #-----------------------------------------------------#

    if abs(states[0]) > 2.492954-.02 or abs(states[1]) > 2.05178 - .02 or states[2] > 3:
         rospy.logwarn("Quadrotor has left the RISC MAAP and crashed \n Push the back button to reset.")
         reset = False
         time = 0
         while not reset:
             reset = bool ( rospy.get_param("RESET") )
             if time - floor(time) == 0:
                 print int (10 - time)
             time = time+.5
             if reset:
                 global states
                 states[0]     = float ( rospy.get_param("~Initial_x","0.0") )
                 states[1]     = float ( rospy.get_param("~Initial_y","0.0") )
                 states[2]     = float ( rospy.get_param("~Initial_z","1.0") )
                 states[3]     = float ( rospy.get_param("~Initial_u","0.0") )
                 states[4]     = float ( rospy.get_param("~Initial_v","0.0") )
                 states[5]     = float ( rospy.get_param("~Initial_w","0.0") )
                 states[6]     = float ( rospy.get_param("~Initial_phi","0.0") )
                 states[7]     = float ( rospy.get_param("~Initial_theta","0.0") )
                 states[8]     = float ( rospy.get_param("~Initial_psi","0.0") )
                 states[9]     = float ( rospy.get_param("~Initial_p","0.0") )
                 states[10]    = float ( rospy.get_param("~Initial_q","0.0") )
                 states[11]    = float ( rospy.get_param("~Initial_r","0.0") )
                 rospy.loginfo("Reset")

             if time >= 10:
                 rospy.signal_shutdown(shutdownFunction)
                 break
             rospy.sleep(.5)

    #-------------------------------------------#
    # Make Header with time stamp and frame id  #
    #-------------------------------------------#

    S = States()
    S.x     = states[0]
    S.y     = states[1]
    S.z     = states[2]
    S.u     = states[3]
    S.v     = states[4]
    S.w     = states[5]
    S.phi   = states[6]*180/np.pi
    S.theta = states[7]*180/np.pi
    S.psi   = states[8]*180/np.pi
    S.p     = states[9]*180/np.pi
    S.q     = states[10]*180/np.pi
    S.r     = states[11]*180/np.pi
    S.name = "condor"#"Simulated AR.Drone"
    S.visible = True
    cortex = Cortex()
    cortex.Obj = [S]
    global h
    h.seq =  h.seq + 1
    h.stamp = rospy.Time.now()
    h.frame_id = 'cortex'
    cortex.header = h

    #---------#
    # Publish #
    #---------#

    pub.publish(cortex)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('ardrone_sim')
    start_time = rospy.get_time()

    #-------------------------------------------------------------#
    #    Set up Initial Conditions,Gains and Saturation values    #
    #-------------------------------------------------------------#

    global states
    states[0]     = float ( rospy.get_param("~Initial_x","0.0") )
    states[1]     = float ( rospy.get_param("~Initial_y","0.0") )
    states[2]     = float ( rospy.get_param("~Initial_z","1.0") )
    states[3]     = float ( rospy.get_param("~Initial_u","0.0") )
    states[4]     = float ( rospy.get_param("~Initial_v","0.0") )
    states[5]     = float ( rospy.get_param("~Initial_w","0.0") )
    states[6]     = float ( rospy.get_param("~Initial_phi","0.0") )
    states[7]     = float ( rospy.get_param("~Initial_theta","0.0") )
    states[8]     = float ( rospy.get_param("~Initial_psi","0.0") )
    states[9]     = float ( rospy.get_param("~Initial_p","0.0") )
    states[10]    = float ( rospy.get_param("~Initial_q","0.0") )
    states[11]    = float ( rospy.get_param("~Initial_r","0.0") )
    # Saturation Values
    euler_max     = float ( rospy.get_param("~euler_angle_max","0.349066" ) )
    yaw_rate_max  = float ( rospy.get_param("~control_yaw","0.349065" ) )
    alt_rate_max  = float ( rospy.get_param("~control_vz_max","1000" ) )
    # Grey-Box System Identification of a Quadrotor Unmanned Aerial Vehicle"
    # by Qianying Li, Master's Thesis Delft University 2014
    Ixx           = float ( rospy.get_param("~Ixx","0.002237568") )
    Iyy           = float ( rospy.get_param("~Iyy","0.002985236") )
    Izz           = float ( rospy.get_param("~Izz","0.00480374") )
    # Experimental System ID done by Spencer Maughan December 2014
    # Details can be found in a paper currently being written
    roll_kp       = float ( rospy.get_param("~roll_kp","0.0757") )
    roll_kg       = float ( rospy.get_param("~roll_kg","1.1714") )
    roll_kd       = float ( rospy.get_param("~roll_kd","0.0192") )
    pitch_kp      = float ( rospy.get_param("~pitch_kp","0.0944") )
    pitch_kg      = float ( rospy.get_param("~pitch_kg","1.10714") )
    pitch_kd      = float ( rospy.get_param("~pitch_kd","0.0205") )
    yaw_kp        = float ( rospy.get_param("~yaw_kp","1.2") )
    yaw_kg        = float ( rospy.get_param("~yaw_kg","1") )
    alt_kp_up     = float ( rospy.get_param("~alt_kp_up","1.1271") )
    alt_kg_up     = float ( rospy.get_param("~alt_kg_up","1.1499") )
    alt_kp_dn     = float ( rospy.get_param("~alt_kp_dn","1.1271") )
    alt_kg_dn     = float ( rospy.get_param("~alt_kg_dn","1.1499") )
    alt_kp        = float ( rospy.get_param("~alt_kp","1.1271") )
    alt_kg        = float ( rospy.get_param("~alt_kg","1.1499") )

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub  = rospy.Subscriber('/cmd_vel' , Twist, GetInput)
         if not command or len(ctrl_in) < ctrl_size:
              C = ctrl_in[len(ctrl_in)-1]
              ctrl_in.append(C)
         command = False
         Driver()
         r.sleep()
    rospy.signal_shutdown(shutdownFunction)
