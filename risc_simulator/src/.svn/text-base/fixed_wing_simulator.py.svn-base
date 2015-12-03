#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	January 2015
    File name: 		fixed_wing_simulator.py
    Organization:	RISC Lab, Utah State University

    Notes:
          The Equations of Motion for this model were taken
    from:
    Beard & McLain, “Small Unmanned Aircraft,” Princeton University Press, 2012, Chapter 3      

 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import numpy as np
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
import std_msgs.msg # to make header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist      # for sending commands to the drone

    #===================#
    #      Globals      #
    #===================#

    #---------------#
    #   Constants   #
    #---------------#

PI 	      = 3.141592653589793
g             = 9.8067
rate          = 200 # Hz

    #----------#
    #   Time   #
    #----------#

start_time    = 0
h = std_msgs.msg.Header()

    #-----------#
    #   Input   #
    #-----------#

ctrl_in       = Twist()

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

pub           = rospy.Publisher('/cortex', Cortex, queue_size = 200)

    #--------------------------------------------#
    #           Inertial Coefficients            #
    #--------------------------------------------#

# These can be modified through the launch file
mass     = 1.56   #kg
Jxx      = 0.1147 #Kg*m^2
Jyy      = 0.0576 #Kg*m^2
Jzz      = 0.1712 #Kg*m^2
Jxz      = 0.0015 #Kg*m^2

    #==============================#
    #    Executed Upon Shutdown    #
    #==============================#

def shutdownFunction():

    rospy.loginfo("Fixed Wing Simulation Shutting Down...")


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
    ROT = np.matrix([[cT*cP, sF*sT*cP-cF*sP, cF*sT*cP+sF*sP],\
                     [cT*sP, sF*sT*sP+cF*cP, cF*sT*sP-sF*cP],\
                     [  -sT,          sF*cT,          cF*cT]])
    return ROT
    #=================#
    #    Get Input    #
    #=================#

def GetInput(I):
    global ctrl_in
    ctrl_in = I

    #=====================================#
    #    Fixed wing Equations of Motion   #
    #=====================================#

def FixedWing(x,y,z,u,v,w,phi,theta,psi,p,q,r):
    global ctrl_in,g,Jxx,Jyy,Jzz,Jxz,mass

    #------------#
    #   Inputs   #
    #------------#

    fx  = ctrl_in.linear.x
    fy  = ctrl_in.linear.y
    fz  = ctrl_in.linear.z
    ell = ctrl_in.angular.x
    m   = ctrl_in.angular.y
    n   = ctrl_in.angular.z

    Gamma  = Jxx*Jzz-Jxz*Jxz
    Gamma1 = (Jxz*(Jxx-Jyy+Jzz))/Gamma
    Gamma2 = (Jzz*(Jzz-Jyy)+Jxz*Jxz)/Gamma
    Gamma3 = Jxx/Gamma
    Gamma4 = Jxz/Gamma
    Gamma5 = (Jzz-Jxx)/Gamma
    Gamma7 = (Jxx*(Jxx-Jyy)+Jxz*Jxz)/Gamma
    Gamma8 = Jxx/Gamma
    
    #--------------#
    #   Velocity   #
    #--------------#

    dxdt = np.asmatrix(np.zeros((12,1)))
    pndot = cos(theta)*cos(psi)*u\
            + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v\
            + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w
                                
    pedot = cos(theta)*sin(psi)*u\
            + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v\
            + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w
                                        
    pddot = -sin(theta)*u\
            + sin(phi)*cos(theta)*v\
            + cos(phi)*cos(theta)*w
    dxdt[0] = pndot
    dxdt[1] = pedot
    dxdt[2] = pddot

    #------------------#
    #   Acceleration   #
    #------------------#

    udot      = r*v - q*w + (1/mass)*fx
    vdot      = p*w - r*u + (1/mass)*fy 
    wdot      = q*u - p*v + (1/mass)*fz
    dxdt[3:6] = np.matrix([[udot],[vdot],[wdot]])

    #----------------------#
    #   Angular Velocity   #
    #----------------------#

    phidot    = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
    thetadot  = cos(phi)*q - sin(phi)*r
    psidot    = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r
    dxdt[6:9] = np.matrix([[phidot],[thetadot],[psidot]])

    #--------------------------#
    #   Angular Acceleration   #
    #--------------------------#
    
    pdot       = Gamma1*p*q - Gamma2*q*r + Gamma3*ell + Gamma4*n
    qdot       = Gamma5*p*r - Gamma4*(p*p-r*r) + (1/Jyy)*m
    rdot       = Gamma7*p*q - Gamma1*q*r + Gamma4*ell + Gamma8*n
    dxdt[9:12] = np.matrix([[pdot],[qdot],[rdot]]) 
    return dxdt

    #==================#
    #    ODE Driver    #
    #==================#

def Driver():
    global PI,rate,pub,states

    #---------------#
    # one unit step #
    #---------------#

    dy = FixedWing(states.x,states.y,states.z,states.u,states.v,states.w,PI*states.phi/180,PI*states.theta/180,PI*states.psi/180,PI*states.p/180,PI*states.q/180,PI*states.r/180)

    #------------------------#
    # propogate given the dy #
    #------------------------#

    states.x      = states.x     + dy[0,0]/rate
    states.y      = states.y     + dy[1,0]/rate
    states.z      = states.z     + dy[2,0]/rate
    states.u      = states.u     + dy[3,0]/rate
    states.v      = states.v     + dy[4,0]/rate
    states.w      = states.w     + dy[5,0]/rate
    states.phi    = 180*(states.phi*PI/180   + dy[6,0]/rate)/PI
    states.theta  = 180*(states.theta*PI/180 + dy[7,0]/rate)/PI
    states.psi    = 180*(states.psi*PI/180   + dy[8,0]/rate)/PI
    states.p      = 180*(states.p*PI/180     + dy[9,0]/rate)/PI
    states.q      = 180*(states.q*PI/180     + dy[10,0]/rate)/PI
    states.r      = 180*(states.r*PI/180     + dy[11,0]/rate)/PI

    #-----------------------------------------------------#
    # Crash if the Altitude is zeros moves out of the RISC MAAP region #
    #-----------------------------------------------------#

    if states.z < 0:
         rospy.logwarn("Aircraft has crashed. \n Push the back button to reset.")
         reset = False
         time = 0
         while not reset:
             reset = bool ( rospy.get_param("RESET") )
             if time - floor(time) == 0:
                 print int (10 - time)
             time = time+.5
             if reset:
                 global states
                 states.x      = 0.0
                 states.y      = 0.0
                 states.z      = 100.0
                 states.u      = 1.0
                 states.v      = 0.0
                 states.w      = 0.0
                 states.phi    = 0.0
                 states.theta  = 0.0
                 states.psi    = 0.0
                 states.p      = 0.0
                 states.q      = 0.0
                 states.r      = 0.0
                 rospy.loginfo("Reset")

             if time >= 10:
                 rospy.signal_shutdown(shutdownFunction)
                 break
             rospy.sleep(.5)



    #-------------------------------------------#
    # Make Header with time stamp and frame id  #
    #-------------------------------------------#

    states.name = "aircraft"
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
    rospy.init_node('fixed_wing_sim')
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
    mass          = float ( rospy.get_param("~mass","1.56") )
    Jxx           = float ( rospy.get_param("~Jxx","0.1147") )
    Jyy           = float ( rospy.get_param("~Jyy","0.0576") )
    Jzz           = float ( rospy.get_param("~Jzz","0.1712") )
    Jxz           = float ( rospy.get_param("~Jxz","0.0015") )

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub  = rospy.Subscriber('/fixed_wing_input' , Twist, GetInput)
         Driver()
         r.sleep()
    rospy.signal_shutdown(shutdownFunction)
