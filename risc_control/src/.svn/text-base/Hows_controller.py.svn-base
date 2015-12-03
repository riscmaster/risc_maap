#!/usr/bin/env python

'''======================================================
             ICUAS 2015 HOW'S METHOD CODE
   ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import rospkg
import numpy as np
import scipy.linalg as la
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
from std_msgs.msg import Float64

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
states          = Cortex()
states.Obj      = [States()]*1
euler_max       = 0.349066 #in radians
max_yaw_rate    = 0.3490659 #in radians/sec
max_alt_rate    = 1000     # in mm/sec
rate            = 200      # Hz
time_past       = 0
I               = 0
start_time      = 0
I_Switch        = True
#kp              = 0.4
#kd              = 0.6792
#K               = np.matrix([[   kp,      0,    0,   kd*.98,    0,    0, 0],\
#                             [    0,     kp,    0,    0,   kd*1.0001,    0, 0],\
#                             [    0,      0,  -.45,   0,    0, -0.7, 0],\
#                             [    0,      0,    0,    0,    0,    0, 1]])
kp              = 0.1*2
kd              = 0.1698*2
K               = np.matrix([[   kp,      0,    0,   kd,    0,    0, 0],\
                             [    0,     kp,    0,    0,   kd,    0, 0],\
                             [    0,      0,  -.45,   0,    0, -0.9, 0],\
                             [    0,      0,    0,    0,    0,    0, 1]])
    #==================#
    #    Publishers    #
    #==================#

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 200)
pub_Range       = rospy.Publisher('/range', Float64, queue_size = 200)

    #===================================#
    #    Radians between + or - pi/2    #
    #===================================#

def pi2pi(angle):
     if abs(angle)>PI/2:
           if angle>0:
                angle = angle-PI
           else:
                angle = angle+PI
     return angle

    #========================#
    #    Get Cortex States   #
    #========================#

def GetTrajectory(W):

    traj = W
    Hows_Controller(traj,K)

def GetStates(S):

    global states
    states = S

    #========================#
    #    Hows Controller    #
    #========================#

def Hows_Controller(traj,K):
    global states,PI, euler_max, max_yaw_rate, max_alt_rate, pub_ctrl, time_past,I_Switch

    Ctrl        = Controls()
    # Initiate Control Messages
    bodies = 1
    Ctrl.Obj = [Control()]*bodies
    Ctrl.header.stamp = states.header.stamp


    g = 9.81
    m = 0.450 # ARDrone mass

    #==============================#
    #   User defined Parameters    #
    #==============================#


    kv = 0.9
    R_star = .48
    Init_Head = 2 # Initial heading of vehical,target geometry w.r.t.x in radians
    phase = PI/2 # Select phase angle of trajectory
    V_veh = 0.2
    Init_Acceleration = 0.2888

    if traj.Obj[0].name == "NGC":
       x = states.Obj[0].x
       y = states.Obj[0].y
       z = states.Obj[0].z
       xt = traj.Obj[0].x
       yt = traj.Obj[0].y
       zt = traj.Obj[0].z

       #print y
       X = np.asmatrix(np.zeros((3,1)))
       X[0] = states.Obj[0].u
       X[1] = states.Obj[0].v
       X[2] = states.Obj[0].w
       V = np.linalg.norm(X)
       #print V
       psi_v   = atan2(X[1,-1],X[0,-1])
       theta_v = atan2(X[2,-1],sqrt(X[1,-1]*X[1,-1]+X[0,-1]*X[0,-1])) #X[2,-1] + or -

       Range = sqrt((x-xt)*(x-xt)+(y-yt)*(y-yt)+(z-zt)*(z-zt))


       #if Range < R_star+.05:
          # rospy.logerr("Range too small")
       #if Range > 2*R_star:
           #rospy.logerr("Range too large")
       Rxy   = sqrt((x-xt)*(x-xt)+(y-yt)*(y-yt))
       psi_l = atan2 (yt-y, xt-x)
       theta_l = atan2 ((zt-z), Rxy)

       v=V*np.matrix([[cos(theta_v)*cos(psi_v)],[cos(theta_v)*sin(psi_v)],[sin(theta_v)]])

       r=Range*np.matrix([[cos(theta_l)*cos(psi_l)],[cos(theta_l)*sin(psi_l)],[sin(theta_l)]])
       #print Range
       EE = np.asmatrix(np.dot(r.T,v))

       eta=acos(EE/(Range*V))

       #print eta*(180/PI)


       acmd=acmd=((2*V_veh*V_veh)/R_star)*sin(eta) # ((2*V*V)/Range)*pi2pi(eta) #

       n=np.asmatrix(np.cross(v.T,r.T))
       FF = np.asmatrix(np.cross(n,v.T))
       GG = sqrt(np.asmatrix(np.dot(n,n.T)))*V
       ac=(acmd*FF)/GG

       ax=ac.T[0,-1]
       ay=ac.T[1,-1]
       az=ac.T[2,-1]

       Radius_Veh = sqrt((x*x)+(y*y))
       Radius_tgt = sqrt((xt*xt)+(yt*yt))
       ah=sqrt(ax*ax+ay*ay)
       av=az

       psidot=(ah/(V*cos(theta_v)))
       #print psidot

       thetadot=(av/V)
       #print thetadot
       vdot=-kv*(V-V_veh)





       scale = 1
       if X[0] > -0.2969 and X[1] > -0.0427 and I_Switch:##
           aax = -0.2851*scale
           aay = -0.0410*scale
       else:
           aax=(-(V_veh*sin(theta_v)*cos(psi_v)*thetadot)-(V_veh*cos(theta_v)*sin(psi_v)*psidot)+(vdot*cos(theta_v)*cos(psi_v)))
           aay=(-(V_veh*sin(theta_v)*sin(psi_v)*thetadot)+(V_veh*cos(theta_v)*cos(psi_v)*psidot)+(vdot*cos(theta_v)*sin(psi_v)))
           I_Switch = False
       aaz = ((V_veh*cos(theta_v)*thetadot)+(vdot*sin(theta_v))) +g

       #print I_Switch
       u=np.matrix([[-aax],[-aay],[aaz],[0]])
       #print u.T

       #==================================#
       #     Rotate to Vehicle 1 Frame    #
       #==================================#

       psi = states.Obj[0].psi*PI/180
       rotZ = np.matrix([[cos(-psi), -sin(-psi), 0],[sin(-psi), cos(-psi), 0],[0, 0, 1]])
       u[:-1] = rotZ*u[:-1]
       #print u


       #===================================#
       #     Normalize given the Thrust    #
       #===================================#

       T = sqrt(u[0:3].T*u[0:3])
       u[:-1] = np.divide(u[:-1],T)

       #==================#
       #   Set Controls   #
       #==================#

       # Controls for Ardrone
       # -phi = right... +phi = left
       # -theta = back... +theta = forward
       # -psi = right...  +psi = left
       ctrl        = Control()
       ctrl.name   = states.Obj[0].name
       ctrl.phi    = asin(u[1,-1])/euler_max
       ctrl.theta  = asin(u[0,-1])/euler_max
       ctrl.psi    = u[3,-1] /max_yaw_rate
       ctrl.T      = T*m
       Ctrl.Obj[0] = ctrl
       pub_ctrl.publish(Ctrl)
       pub_Range.publish(Range)



    else:
        if states.Obj[0].visible:
            X = np.asmatrix(np.zeros((7,1)))
            X[0] = traj.Obj[0].x-states.Obj[0].x
            X[1] = traj.Obj[0].y-states.Obj[0].y
            X[2] = traj.Obj[0].z-states.Obj[0].z
            X[3] = traj.Obj[0].xdot-states.Obj[0].u
            X[4] = traj.Obj[0].ydot-states.Obj[0].v
            X[5] = traj.Obj[0].zdot-states.Obj[0].w
            X[6] = pi2pi(traj.Obj[0].psi)-states.Obj[0].psi*PI/180

            #============================================#
            #     Differential Flatness Control Input    #
            #============================================#

            # LQR input
            utilde = -K*X
            # required input
            u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot],[traj.Obj[0].psiddot]])
            u = utilde-u_r+np.matrix([[0],[0],[9.81],[0]])

            #==================================#
            #     Rotate to Vehicle 1 Frame    #
            #==================================#

            psi = states.Obj[0].psi*PI/180
            rotZ = np.matrix([[cos(-psi), -sin(-psi), 0],[sin(-psi), cos(-psi), 0],[0, 0, 1]])
            u[:-1] = rotZ*u[:-1]
            #print u


            #===================================#
            #     Normalize given the Thrust    #
            #===================================#

            T = sqrt(u[0:3].T*u[0:3])
            u[:-1] = np.divide(u[:-1],T)

            #==================#
            #   Set Controls   #
            #==================#

            # Controls for Ardrone
            # -phi = right... +phi = left
            # -theta = back... +theta = forward
            # -psi = right...  +psi = left
            ctrl        = Control()
            ctrl.name   = states.Obj[0].name
            ctrl.phi    = asin(u[1,-1])/euler_max
            ctrl.theta  = asin(u[0,-1])/euler_max
            ctrl.psi    = u[3,-1] /max_yaw_rate
            ctrl.T      = T*m
            Ctrl.Obj[0] = ctrl
            pub_ctrl.publish(Ctrl)





    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Hows_Controller')

    #=======================#
    #    quad parameters    #
    #=======================#

    euler_max    = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw",".3490659")) #in radians/sec
    max_alt_rate = float(rospy.get_param("control_vz_max","1000")) #in mm/sec


    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
    blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
    rospy.spin()

