#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	February 2014
    File name: 		ardrone_state_filter.py
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
rate          = 200 # Hz
euler_max     = 0.349066 # radians
yaw_rate_max  = 0.3490659 # radians/sec
alt_rate_max  = 1000 # mm/sec

    #----------#
    #   Time   #
    #----------#

start_time    = 0
delay         = 0.3
h = std_msgs.msg.Header()

    #-----------#
    #   Input   #
    #-----------#

ctrl_size     = int (ceil(delay*rate))
if ctrl_size == 0:
    ctrl_size = 1
ctrl_in       = [Control()]*ctrl_size

    #-------------------------#
    #   Initiial Conditions   #
    #-------------------------#

states        = States()
states.x      = 0.0
states.y      = 0.0
states.z      = 0.0
states.u      = 0.0
states.v      = 0.0
states.w      = 0.0
states.phi    = 0.0
states.theta  = 0.0
states.psi    = 0.0
states.p      = 0.0
states.q      = 0.0
states.r      = 0.0

    #----------------#
    #   Publishers   #
    #----------------#

pub           = rospy.Publisher('/cortex_raw', Cortex, queue_size = 200)
pub_acc       = rospy.Publisher('/ardrone_acc', PointStamped, queue_size = 200)

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
    global ctrl_in,ctrl_size
    C = Control()
    C.theta = I.linear.x
    C.phi   = I.linear.y
    C.T   = I.linear.z
    C.psi   = I.angular.z
    print len(ctrl_in)
    if len(ctrl_in) == 1:
        ctrl_in = [C]
    else:
        del ctrl_in[0]
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

def AR_Drone(x,y,z,u,v,w,phi,theta,psi,p,q,r,u):
    global g,Ixx,Iyy,Izz,euler_max,yaw_rate_max,alt_rate_max,ctrl_size,m,w_old, T_kp

    #------------#
    #   Inputs   #
    #------------#

    phi_c   = -u.phi*euler_max # roll
    theta_c = -u.theta*euler_max # pitch
    r_c     = -u.psi*yaw_rate_max # yaw_rate
    zdot_c  =  u.T

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
    acc.point.z = dxdt[5,-1]
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

    dxdt[9] =  (q*r*(Iyy-Izz)/Ixx) + tauPhi/Ixx
    dxdt[10] = (p*r*(Izz-Ixx)/Iyy) + tauTheta/Iyy
    dxdt[11] = (p*q*(Ixx-Iyy)/Izz) + tauPsi/Izz

    return dxdt

    #==================#
    #    ODE Driver    #
    #==================#

def Driver():
    global PI,rate,pub,start_time,states

    for i in range(len(ctrl_in))
        #---------------#
        # one unit step #
        #---------------#

        dy = AR_Drone(states.x,states.y,states.z,states.u,states.v,states.w,PI*states.phi/180,PI*states.theta/180,PI*states.psi/180,PI*states.p/180,PI*states.q/180,PI*states.r/180)

        #------------------------#
        # propogate given the dy #
        #------------------------#

        states.x      = states.x      + dy[0,0]/rate +np.random.normal(0,.1,1)[0]
        states.y      = states.y      + dy[1,0]/rate +np.random.normal(0,.1,1)[0]
        states.z      = states.z      + dy[2,0]/rate +np.random.normal(0,.1,1)[0]
        states.u      = (states.u     + dy[3,0]/rate)+np.random.normal(0,.1,1)[0]
        states.v      = (states.v     + dy[4,0]/rate)+np.random.normal(0,.1,1)[0]
        states.w      = (states.w     + dy[5,0]/rate)+np.random.normal(0,.1,1)[0]
        states.phi    = 180*(states.phi*PI/180   + dy[6,0]/rate)/PI+np.random.normal(0,.004,1)[0]
        states.theta  = 180*(states.theta*PI/180 + dy[7,0]/rate)/PI+np.random.normal(0,.004,1)[0]
        states.psi    = 180*(states.psi*PI/180   + dy[8,0]/rate)/PI+np.random.normal(0,.004,1)[0]
        states.p      = 180*(states.p*PI/180     + dy[9,0]/rate)/PI+np.random.normal(0,.005,1)[0]
        states.q      = 180*(states.q*PI/180     + dy[10,0]/rate)/PI+np.random.normal(0,.005,1)[0]
        states.r      = 180*(states.r*PI/180     + dy[11,0]/rate)/PI+np.random.normal(0,.005,1)[0]

        # Saturation Bounds

        if states.z < 0.05:
            states.z = 0.05
        if states.z > 3:
            states.z = 3

    #-------------------------------------------#
    # Make Header with time stamp and frame id  #
    #-------------------------------------------#

    states.name = "condor"#"Simulated AR.Drone"
    states.visible = True
    cortex = Cortex()
    cortex.Obj = [states]
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

    states.x      = float ( rospy.get_param("~Initial_x","0.0") )
    states.y      = float ( rospy.get_param("~Initial_y","0.0") )
    states.z      = float ( rospy.get_param("~Initial_z","1.0") )
    states.u      = float ( rospy.get_param("~Initial_u","0.0") )
    states.v      = float ( rospy.get_param("~Initial_v","0.0") )
    states.w      = float ( rospy.get_param("~Initial_w","0.0") )
    states.phi    = float ( rospy.get_param("~Initial_phi","0.0") )
    states.theta  = float ( rospy.get_param("~Initial_theta","0.0") )
    states.psi    = float ( rospy.get_param("~Initial_psi","0.0") )
    states.p      = float ( rospy.get_param("~Initial_p","0.0") )
    states.q      = float ( rospy.get_param("~Initial_q","0.0") )
    states.r      = float ( rospy.get_param("~Initial_r","0.0") )
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

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         print "in"
         sub  = rospy.Subscriber('/cmd_vel' , Twist, GetInput)
         #sub_states  = rospy.Subscriber('/cortex_raw' , Cortex, GetInput)
         Driver()
         r.sleep()
    rospy.signal_shutdown(shutdownFunction)
