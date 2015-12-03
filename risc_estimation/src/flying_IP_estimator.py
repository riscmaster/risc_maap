#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	January 2014
    File name: 		flying_IP_estimator.py
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
delay         = 0.005
h = std_msgs.msg.Header()

    #-----------#
    #   Input   #
    #-----------#

ctrl_size     = int (ceil(delay*rate))
if ctrl_size == 0:
    ctrl_size = 1
ctrl_in       = [Control()]*ctrl_size

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
    global ctrl_in,ctrl_size
    C = Control()
    C.theta = I.linear.x
    C.phi   = I.linear.y
    C.alt   = I.linear.z
    C.psi   = I.angular.z
    if len(ctrl_in) == 1:
        ctrl_in = [C]
    else:
        del ctrl_in[0]
        ctrl_in.append(C)

    #==================#
    #    Get States    #
    #==================#

def GetStates(S):
    global pen_states, states, cortex_states
    # get quad states
    for i in range(len(S.Obj)):
        if S.Obj[i].name == 'Condor' or S.Obj[i].name == 'Kahn':
            states = S.Obj[i]
    for i in range(len(S.Obj)):
        if S.Obj[i].name == 'Pendulum_tip':
            pen_states.r       = S.Obj[i].x-states.x
            pen_states.s       = S.Obj[i].y-states.y
            pen_states.rdot    = S.Obj[i].u-states.u
            pen_states.sdot    = S.Obj[i].v-states.v
    cortex_states = S

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
        zeta = 0.0000001
    else:
        zeta = sqrt(inner_term)
    zeta2 = L*L-r*r-s*s

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

    if len(ctrl_in) == 0:
        phi_c   = -ctrl_in.phi*euler_max # roll
        theta_c = -ctrl_in.theta*euler_max # pitch
        r_c   = -ctrl_in.psi*yaw_rate_max # yaw_rate
        alt_c   = ctrl_in.alt*alt_rate_max*.001 # altitude_rate
    else:
        phi_c   = -ctrl_in[index].phi*euler_max # roll
        theta_c = -ctrl_in[index].theta*euler_max # pitch
        r_c   = -ctrl_in[index].psi*yaw_rate_max # yaw_rate
        alt_c   = ctrl_in[index].alt*alt_rate_max*.001 # altitude_rate


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
    thrust = zddot*sqrt(1+tan(theta)*tan(theta)+tan(phi)*tan(phi))
    acc_body_frame = np.matrix([[0.0],[0.0],[-thrust]])
    world_frame_acceleration = np.multiply(rotB2W(phi, theta, psi)*acc_body_frame, np.matrix('[1;-1;-1]')) - np.matrix([[0],[0],[g]])
    dxdt[3:6] = world_frame_acceleration

    #----------------------#
    #   Angular Velocity   #
    #----------------------#

    # Gyro to Body Rotation
    G2B = np.matrix([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],\
                      [0,            cos(phi),           -sin(phi)],\
                      [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])

    angles = G2B*np.matrix([[p],[q],[r]])
    #angles = np.matrix([[phi_c],[theta_c],[r_c]])

    dxdt[6:9] = angles
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

def Driver():
    global PI,rate,pub,start_time,states,ctrl
    for j in range(len(ctrl_in)):
    #---------------#
    # one unit step #
    #---------------#

        dy = AR_Drone(states.x,states.y,states.z,states.u,states.v,states.w,PI*states.phi/180,PI*states.theta/180,PI*states.psi/180,PI*states.p/180,PI*states.q/180,PI*states.r/180,j)

    #-----------#
    # propogate #
    #-----------#

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

        d_pen = Inverted_Pendulum(pen_states.r,pen_states.s,pen_states.rdot,pen_states.sdot,dy[3:6])
        pen_states.r      = pen_states.r    + d_pen[0,0]/rate
        pen_states.s      = pen_states.s    + d_pen[1,0]/rate
        pen_states.rdot   = pen_states.rdot + d_pen[2,0]/rate
        pen_states.sdot   = pen_states.sdot + d_pen[3,0]/rate

    #-------------------------------------------#
    # Make Header with time stamp and frame id  #
    #-------------------------------------------#

    states.name = "condor"
    states.visible = True
    for m in range(len(cortex_states.Obj)):
        if cortex_states.Obj[m].name == 'Pendulum_tip':
            cortex_states.Obj[m].x = pen_states.r + states.x
            cortex_states.Obj[m].y = pen_states.s + states.y
            cortex_states.Obj[m].u = pen_states.rdot + states.u
            cortex_states.Obj[m].v = pen_states.sdot + states.v
        if cortex_states.Obj[m].name == 'Condor':
            cortex_states.Obj[m] = states

    global h
    h.seq =  h.seq + 1
    h.stamp = rospy.Time.now()
    h.frame_id = 'cortex'
    cortex_states.header = h

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
    start_time = rospy.get_time()

    #-------------------------------------#
    #    Set up Publish/Subscribe Loop    #
    #-------------------------------------#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub       = rospy.Subscriber('/cmd_vel' , Twist, GetInput)
         sub_quad  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
         Driver()
         r.sleep()
    rospy.signal_shutdown(shutdownFunction)
