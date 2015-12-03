#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	January 2014
    File name: 		faster_IP_estimator.py
    Organization:	RISC Lab, Utah State University

    Notes:

    This Model does not yet include Covariance Estimate.

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
euler_max     = 0.349066 # radians
yaw_rate_max  = 0.3490659 # radians/sec
alt_rate_max  = 1000 # mm/sec
L             = 1

    #----------#
    #   Time   #
    #----------#

start_time    = 0
delay         = 0.025
h = std_msgs.msg.Header()

    #-----------#
    #   Input   #
    #-----------#

ctrl_size     = int (ceil(delay*rate))
if ctrl_size == 0:
    ctrl_size = 1
ctrl_in       = [Twist()]*ctrl_size

    #-------------------------------------------------------------#
    #    Set up Initial Conditions,Gains and Saturation values    #
    #-------------------------------------------------------------#

pen_states         = PendulumStates()
pen_states.r       = 0.0
pen_states.s       = .00001
pen_states.rdot    = 0.0
pen_states.sdot    = 0.0


cortex_states = Cortex()
states        = States()
states.x      = 0.0
states.y      = 0.0
states.z      = 1.0
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

    #----------------------------------------------------#
    #           ARDrone Inertial Coefficients            #
    #        and Attitude PID gains without hull         #
    #----------------------------------------------------#

# Grey-Box System Identification of a Quadrotor Unmanned Aerial Vehicle"
# by Qianying Li, Master's Thesis Delft University 2014
Ixx           = 0.002237568#Kg*m^2
Iyy           = 0.002985236#Kg*m^2
Izz           = 0.00480374#Kg*m^2

# Experimental System ID done by Spencer Maughan December 2014
# Details can be found in a paper currently being written
roll_kp       = 0.0782
roll_ki       = 0.0
roll_kd       = 0.0162
pitch_kp      = 0.1133
pitch_ki      = 0.0
pitch_kd      = 0.0479
yaw_kp1       = 0.0153
yaw_kp2       = 0.0027
yaw_kp3       = 0.0088
alt_kp1       = 4.0906
alt_kp2       = 0.0811
alt_kp3       = -2.2967

    #==============================#
    #    Executed Upon Shutdown    #
    #==============================#

def shutdownFunction():

    rospy.loginfo("Inverted Pendulum Estimation Shutting Down...")


    #================================================#
    #    Rotation Matrix from Body to World Frame    #
    #================================================#

def rotB2W(phi, theta, psi):
    cT = 1
    cF = 1
    cP = cos(psi)
    sT = theta
    sF = phi
    sP = sin(psi)
    ROT = np.matrix([[cT*cP, sF*sT*cP-cF*sP, cF*sT*cP+sF*sP],\
                     [cT*sP, sF*sT*sP+cF*cP, cF*sT*sP-sF*cP],\
                     [  -sT,          sF*cT,          cF*cT]])
    return ROT

    #=================#
    #    Get Input    #
    #=================#

def GetInput(I):
    global ctrl_in,ctrl_size
    C = Control()
    if len(ctrl_in) == 1:
        ctrl_in = [C]
    else:
        del ctrl_in[0]
        ctrl_in.append(C)

    #==================#
    #    Get States    #
    #==================#

def GetStates(S):
    global pen_states, states, cortex_states,PI
    # get quad states
    states = S.Obj[0]
    states.phi    = PI*(states.phi  )/180
    states.theta  = PI*(states.theta)/180
    states.psi    = PI*(states.psi  )/180
    states.p      = PI*(states.p    )/180
    states.q      = PI*(states.q    )/180
    states.r      = PI*(states.r    )/180
    pen_states.r       = S.Obj[1].x-states.x
    pen_states.s       = S.Obj[1].y-states.y
    pen_states.rdot    = S.Obj[1].u-states.u
    pen_states.sdot    = S.Obj[1].v-states.v
    cortex_states = S
    Driver(pen_states,states,cortex_states)

    #=====================#
    #    PID Controller   #
    #=====================#

def pidControl(kp,kd,ki,act,des,Ts,vel):
    err = des-act
    u = kp*err - kd*vel #+ ki*err*Ts not valid need actual Integral error term
    return u

    #========================================#
    #    3rd order Proportional Controller   #
    #========================================#

def p3Control(kp1,kp2,kp3,act,des):
    err = des-act
    u = kp1*err + kp2*err*err + kp3*err*err*err
    return u

    #============================================#
    #    Inverted Pendulum Equations of Motion   #
    #============================================#

def Inverted_Pendulum(r,s,rdot,sdot,pen_in):
    global g,L
    xddot = pen_in[0]
    yddot = pen_in[1]
    zddot = pen_in[2]
    # variables to simplify expressions
    inner_term = L*L-r*r-s*s
    if inner_term < 0:
        zeta  = 0.0000001
        zeta2 = 0.0000001
    else:
        zeta  = sqrt(inner_term)
        zeta2 = inner_term

    A = (zeta2/(zeta2 + r*r))*( -xddot + zddot*r/zeta - rdot*rdot*r/zeta2 -  sdot*sdot*r/zeta2 - rdot*rdot*r*r*r/(zeta2*zeta2) - 2*rdot*sdot*r*r*s / (zeta2*zeta2) - sdot*sdot*s*s*r/(zeta2*zeta2) + g*r/zeta)

    B = (zeta2/(zeta2 + s*s))*( -yddot + zddot*s/zeta - sdot*sdot*s/zeta2 - rdot*rdot*s/zeta2 - sdot*sdot*s*s*s/(zeta2*zeta2) - 2*sdot*rdot*s*s*r/(zeta2*zeta2) - rdot*rdot*r*r*s/(zeta2*zeta2) + g*s/zeta)

    dxdt = np.matrix([[0],[0],[0],[0]])
    dxdt[0] = rdot
    dxdt[1] = sdot
    dxdt[2] = (L*L-r*r)*(A*(L*L-s*s)-B*r*s)/((L*L-s*s)*(L*L-s*s)-r*r*s*s)
    dxdt[3] = B - dxdt[2]*r*s/(L*L-r*r)
    return dxdt

    #===================================#
    #    AR.Drone Equations of Motion   #
    #===================================#

def AR_Drone(x,y,z,u,v,w,phi,theta,psi,p,q,r,index):
    global ctrl_in,g,Ixx,Iyy,Izz,euler_max,yaw_rate_max,alt_rate_max,ctrl_size

    #------------#
    #   Inputs   #
    #------------#

    theta_c = ctrl_in[index].linear.x*euler_max # pitch
    phi_c   = ctrl_in[index].linear.y*euler_max # roll
    alt_c   = ctrl_in[index].linear.z*alt_rate_max*.001 # altitude_rate
    r_c   = ctrl_in[index].angular.z*yaw_rate_max # yaw_rate

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

    global alt_kp,alt_ki,rate
    zddot = g+p3Control(alt_kp1,alt_kp2,alt_kp3,w,alt_c)
    thrust = zddot*sqrt(1+theta*theta+phi*phi)
    acc_body_frame = np.matrix([[0.0],[0.0],[-thrust]])
    world_frame_acceleration = np.multiply(rotB2W(phi, theta, psi)*acc_body_frame, np.matrix('[1;-1;-1]')) - np.matrix([[0],[0],[g]])
    dxdt[3:6] = world_frame_acceleration

    #----------------------#
    #   Angular Velocity   #
    #----------------------#



    dxdt[6:9] = np.matrix([[p],[q],[r]])
    # These are cortex measurements and do not require rotation

    #--------------------------#
    #   Angular Acceleration   #
    #--------------------------#

    global roll_kp,roll_kd,roll_ki,pitch_kp,pitch_kd,pitch_ki,yaw_kp,yaw_ki
    tauPhi   = pidControl(roll_kp,roll_kd,roll_ki,phi,phi_c,1/rate, p)
    tauTheta = pidControl(pitch_kp,pitch_kd,pitch_ki,theta,theta_c,1/rate,q)
    tauPsi   = p3Control(yaw_kp1,yaw_kp2,yaw_kp3,r,r_c)

    dxdt[9] =  (q*r*(Iyy-Izz)/Ixx) + tauPhi/Ixx
    dxdt[10] = (p*r*(Izz-Ixx)/Iyy) + tauTheta/Iyy
    dxdt[11] = (p*q*(Ixx-Iyy)/Izz) + tauPsi/Izz
#    print dxdt[9:12]
    return dxdt

    #==================#
    #    ODE Driver    #
    #==================#

def Driver(pen_states,states,cortex_states):
    global PI,rate,pub,start_time,ctrl
    for j in range(len(ctrl_in)):
    #---------------#
    # one unit step #
    #---------------#

        dy = AR_Drone(states.x,states.y,states.z,states.u,states.v,states.w,states.phi,states.theta,states.psi,states.p,states.q,states.r,j)

    #-----------#
    # propogate #
    #-----------#

        states.x      = states.x     + dy[0,0]/rate
        states.y      = states.y     + dy[1,0]/rate
        states.z      = states.z     + dy[2,0]/rate
        states.u      = states.u     + dy[3,0]/rate
        states.v      = states.v     + dy[4,0]/rate
        states.w      = states.w     + dy[5,0]/rate
        states.phi    = (states.phi   + dy[6,0]/rate)
        states.theta  = (states.theta + dy[7,0]/rate)
        states.psi    = (states.psi   + dy[8,0]/rate)
        states.p      = (states.p     + dy[9,0]/rate)
        states.q      = (states.q     + dy[10,0]/rate)
        states.r      = (states.r     + dy[11,0]/rate)

        d_pen = Inverted_Pendulum(pen_states.r,pen_states.s,pen_states.rdot,pen_states.sdot,dy[3:6])
        pen_states.r      = pen_states.r    + d_pen[0,0]/rate
        pen_states.r      = pen_states.r    + d_pen[1,0]/rate
        pen_states.rdot   = pen_states.rdot + d_pen[2,0]/rate
        pen_states.sdot   = pen_states.sdot + d_pen[3,0]/rate

    #-------------------------------------------#
    # Make Header with time stamp and frame id  #
    #-------------------------------------------#

    states.name = "condor"
    states.visible = True
    cortex_states.Obj[1].x = pen_states.r + states.x
    cortex_states.Obj[1].y = pen_states.s + states.y
    cortex_states.Obj[1].u = pen_states.rdot + states.u
    cortex_states.Obj[1].v = pen_states.sdot + states.v
    cortex_states.Obj[0] = states

    #---------#
    # Publish #
    #---------#

    pub.publish(cortex_states)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Pen_Estimation')
    #start_time = rospy.get_time()

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#
    if time > 1/rate:
        sub       = rospy.Subscriber('/cmd_vel' , Twist, GetInput)
        start_time = rospy.get_time()

    time = rospy.get_time()-start_time
    sub_quad  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
    rospy.spin()
    rospy.signal_shutdown(shutdownFunction)
