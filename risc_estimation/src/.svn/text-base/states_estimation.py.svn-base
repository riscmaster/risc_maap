#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	May 2015
    File name: 		states_estimation.py
    Organization:	RISC Lab, Utah State University
    Notes:
          This file takes an Object's marker array and 
    generates state estimates. Currently this is being
    to estimate quadrotor states as well as landmark 
    positions. These are recognized by names assigned 
    at the time of template creation using MotionAnalysis
    Cortx Software. This implies that the marker geometry
    is previously known. Therefore the mapping from 
    markers to states is entirely dependent on that 
    template. Velcities are estimated by storing position
    data for a designated period of time and using a 
    linear least squares approximation to estimate the 
    slope.
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

    #========================#
    #        Globals         #
    #========================#

rate            = 200 # Hz
pub             = rospy.Publisher('/cortex_raw',Cortex,queue_size = 1)
land            = rospy.Publisher('/states3',Landmarks,queue_size = 1)
bodies_expected = 4 # up to 4
steps           = 50 # .25 seconds at 200 Hz
v_mat           = np.asmatrix(np.zeros((bodies_expected*steps,6)))
pen_vel         = np.asmatrix(np.zeros((bodies_expected*steps,6)))

    #======================#
    #    Adjust Radians    #
    #======================#

def pi2pi(angle):
     if abs(angle)>np.pi/2:
           if angle>0:
                angle = angle-np.pi
           else:
                angle = angle+np.pi
     return angle

    #====================#
    #    Get MocapData   #
    #====================#

def GetMocapData(data):

    Estimator(data)

    #=================#
    #    Estimator    #
    #=================#

def Estimator(data):

    #============================#
    #    Get Number of Bodies    #
    #============================#

    bodies = len(data.Obj)


    bodies3 = 0 # body requiring 3 states
    bodies6 = 0 # body requiring 6 states


    #=========================================#
    #    Increment by Known Template Names    #
    #=========================================#

    for i in range(bodies):
        name = data.Obj[i].name
        if name == 'Condor' or name == 'Kahn'\
                or name == 'fluffy_II' or name == 'Raven' or \
                name == 'Pendulum_tip' or name == 'IRIS' or name == 'albatross':
            bodies6 = bodies6 +1
        if name == 'Blue_green' or name == 'Green_pink' \
                or name == 'Orange_blue' or name == 'Pink_blue':
            bodies3 = bodies3 +1

    #============================#
    #    Set up Message Types    #
    #============================#

    # 6 state objects (Position and Angle)

    state6 = Cortex()
    state6.Obj = [States()]*bodies6

    # 3 state objects (Position Only)

    state3 = Landmarks()
    state3.Obj = [Landmark()]*bodies3

    #==================================#
    #    Loop through all Templates    #
    #==================================#

    j = 0;# index for 6 state publisher
    m = 0;# index for 3 state publisher

    for i in range(bodies):

#        Set up state estimation by template name
        name = data.Obj[i].name

    #==============================#
    #    Current IRIS Templates    #
    #==============================#


        if name == 'IRIS' :
            iris = Iris(data.Obj[i],j)
            if sqrt(iris.x*iris.x+iris.y*iris.y) > 3 or iris.x != iris.x\
                    or iris.y != iris.y or iris.z != iris.z or iris.phi != iris.phi\
                    or iris.theta != iris.theta or iris.psi != iris.psi\
                    or iris.u != iris.u or iris.v != iris.v or iris.w != iris.w\
                    or iris.p != iris.p or iris.q != iris.q or iris.r != iris.r:
                iris.visible = False
            else:
                iris.visible = True

            state6.Obj[j] = iris
            j = j+1


    #==================================#
    #    Current AR.Drone Templates    #
    #==================================#


        if name == 'Condor' or name == 'Kahn' or name == 'fluffy_II' or name == 'Raven' or name == 'albatross':
            condor = Condor(data.Obj[i],j)
            if sqrt(condor.x*condor.x+condor.y*condor.y) > 3 or condor.x != condor.x\
                    or condor.y != condor.y or condor.z != condor.z or condor.phi != condor.phi\
                    or condor.theta != condor.theta or condor.psi != condor.psi\
                    or condor.u != condor.u or condor.v != condor.v or condor.w != condor.w\
                    or condor.p != condor.p or condor.q != condor.q or condor.r != condor.r:
                condor.visible = False
            else:
                condor.visible = True
            state6.Obj[j] = condor
            j = j+1

    #==========================#
    #    Landmark Templates    #
    #==========================#

        if name == 'Blue_green' or name == 'Green_pink' \
                or name == 'Orange_blue' or name == 'Pink_blue':
            lm = LM(data.Obj[i])
            state3.Obj[m] = lm
            m = m+1

    #=============================#
    #    Pendulum_Tip Template    #
    #=============================#

        if name == 'Pendulum_tip':
            pendulum = Pendulum_tip(data.Obj[i],j)
            if sqrt(pendulum.x*pendulum.x+pendulum.y*pendulum.y) > 3 or pendulum.x != pendulum.x\
                    or pendulum.y != pendulum.y or pendulum.z != pendulum.z \
                    or pendulum.u != pendulum.u or pendulum.v != pendulum.v or pendulum.w != pendulum.w:
                pendulum.visible = False
            state6.Obj[j] = pendulum
            j = j+1

    global pub,land
    state6.header = data.header
    state3.header = data.header
    #latency from camera to state generation
    #rospy.loginfo("latency = %f",data.header.stamp.to_sec()-rospy.get_time())
    pub.publish(state6)
    land.publish(state3)

    #============================#
    #    Estimation Functions    #
    #============================#

    #========================================================================#
    #    NOTE: When these functions become too numerous you should consider  #
    #    creating a library file and then just import those functions.       #
    #    This will help keep things from getting cluttered.                  #
    #========================================================================#

    #==================================================================#
    #    Function that supplies Roll,Pitch and Yaw angles given body   # 
    #    North, East and Down Vectors                                  #
    #==================================================================#

def NED2RPY(north, east, down):

    #fix y direction to correct sign
    north[1] = -north[1]
    east[1]  = -east[1]
    down[1]  = -down[1]
    #fix z direction to correct sign
    north[2] = -north[2]
    east[2]  = -east[2]
    down[2]  = -down[2]

    # north east and down must be a np.matrix form
    ix = np.matrix([[1],[0],[0]])
    iy = np.matrix([[0],[1],[0]])
    iz = np.matrix([[0],[0],[1]])
    # get yaw angle
    psi = acos(north[0]/(np.linalg.norm(north[0:2])))
    # get sign of the angle
    psi = abs(psi*north[1])/north[1]
    # Rotate into Vehicle 1 Frame
    rot_psi = np.matrix([[cos(-psi), -sin(-psi), 0],\
                     [ sin(-psi),  cos(-psi), 0],\
                     [         0,          0, 1]])
    north = rot_psi*north
    east  = rot_psi*east
    down  = rot_psi*down

    # get pitch
    theta = -abs(asin(north[2])*(north[2]))/north[2]
    # Rotate into Vehicle 2 Frame
    rot_theta = np.matrix([[ cos(-theta), 0, sin(-theta)],\
                         [           0, 1,          0],\
                         [ -sin(-theta), 0, cos(-theta)]])
    # theta is not negative because we are rotating in world frame which is not ned
    east = rot_theta*east
    north = rot_theta*north
    # get roll
    phi = abs(asin(east[2])*east[2])/east[2]
    return phi,theta,psi

    #=============================================================#
    #    Function that will produce a rotation matrix about an    # 
    #    arbitrary axis using a right-handed rotation             #
    #=============================================================#

def rotMatAnyAxis(axis,theta):
    ux = axis[0]
    uy = axis[1]
    uz = axis[2]
    ct = cos(theta)
    st = sin(theta)
    Rot = np.array([[     ct+ux*ux*(1-ct), ux*uy*(1-ct)-uz*st, ux*uz*(1-ct)+uy*st],\
                     [uy*ux*(1-ct) + uz*st,    ct+uy*uy*(1-ct), uy*uz*(1-ct)-ux*st],\
                     [  uz*ux*(1-ct)-uy*st, uz*uy*(1-ct)+ux*st,    ct+uz*uz*(1-ct)]])
    return Rot

    #===============#
    #   Landmarks   #
    #===============#

def LM(marker_array):

    #=============================#
    #   Get Markers of interest   #
    #=============================#

    # first three markers
    A = np.matrix([[marker_array.marker[0].x],\
            [marker_array.marker[0].y],[marker_array.marker[0].z]])

    B = np.matrix([[marker_array.marker[1].x],\
            [marker_array.marker[1].y],[marker_array.marker[1].z]])

    C  = np.matrix([[marker_array.marker[2].x],\
            [marker_array.marker[2].y],[marker_array.marker[2].z]])

    D  = np.matrix([[marker_array.marker[3].x],\
            [marker_array.marker[3].y],[marker_array.marker[3].z]])

    #========================#
    #   Calculate Centroid   #
    #========================#

    # take average of all markers
    center = np.divide(np.add(np.add(A,B),np.add(C,D)),4)

    #===========================#
    #   Make Landmark Message   #
    #===========================#

    s = Landmark()
    s.x = center[0]
    s.y = center[1]
    s.z = center[2]
    s.name = marker_array.name

    return s

    #==========#
    #   Iris   #
    #==========#

def Iris(marker_array,j):

    #=============================#
    #   Get Markers of interest   #
    #=============================#

    # first three markers
    A = np.matrix([[marker_array.marker[0].x],\
            [marker_array.marker[0].y],[marker_array.marker[0].z]]) # front_right

    B = np.matrix([[marker_array.marker[1].x],\
            [marker_array.marker[1].y],[marker_array.marker[1].z]]) # back_left

    C  = np.matrix([[marker_array.marker[2].x],\
            [marker_array.marker[2].y],[marker_array.marker[2].z]]) # back_right

    E  = np.matrix([[marker_array.marker[4].x],\
            [marker_array.marker[4].y],[marker_array.marker[4].z]]) # just_above Center

    #=============================================#
    #   Get North, East, and Down in Body Frame   #
    #=============================================#

    # Set up vectors
    BA = A-B
    BC = C-B

    #==========#
    #   east   #
    #==========#

    east = BC/np.linalg.norm(BC)

    #==========#
    #   down   #
    #==========#

    # take the cross product of the vectors in the northeast plane of the quad
    down = np.cross((BA/np.linalg.norm(BA)).T,east.T).T

    #===========#
    #   north   #
    #===========#

    north = np.cross(east.T,down.T).T # unit vector

    #========================#
    #   Calculate Centroid   #
    #========================#

    center = E+.05*down

    #=========================#
    #   roll, pitch and yaw   #
    #=========================#

    phi,theta,psi = NED2RPY(north, east, down)

    #=======================================#
    #   Store Pose for Velocity estimates   #
    #=======================================#

    global steps, v_mat
    # find indices of Beginning and End of submatrix
    Begin = j*steps
    End   = j*steps+steps

    # pushback values one step
    v_mat[Begin:End-1,:] = v_mat[Begin+1:End,:]
    v_mat[End-1,0] = center[0]
    v_mat[End-1,1] = center[1]
    v_mat[End-1,2] = center[2]
    v_mat[End-1,3] = phi*180/np.pi
    v_mat[End-1,4] = theta*180/np.pi
    v_mat[End-1,5] = psi*180/np.pi

    global rate
    t = np.asmatrix(np.linspace(0,.245,steps)).T
    X = np.hstack((np.asmatrix(np.ones(t.size)).T,t))
    # matrix of stored positions and angles
    A = v_mat[Begin:End,:]
    Ax     = A[0:,0]
    Ay     = A[0:,1]
    Az     = A[0:,2]
    Aphi   = A[0:,3]
    Atheta = A[0:,4]
    Apsi   = A[0:,5]

    #=========================#
    #   Estimate Velocities   #
    #=========================#

    Xinv = np.linalg.inv(X.T*X)
    vx = Xinv*X.T*Ax
    vy = Xinv*X.T*Ay
    vz = Xinv*X.T*Az
    vphi = Xinv*X.T*Aphi
    vtheta = Xinv*X.T*Atheta
    vpsi = Xinv*X.T*Apsi

    #=========================#
    #   Make States Message   #
    #=========================#

    s = States()
    s.x = center[0]
    s.y = center[1]
    s.z = center[2]
    s.phi = phi*180/np.pi
    s.theta = theta*180/np.pi
    s.psi = psi*180/np.pi
    s.u = vx[1,-1]
    s.v = vy[1,-1]
    s.w = vz[1,-1]
    s.p = vphi[1,-1]
    s.r = vpsi[1,-1]
    s.q = vtheta[1,-1]
    s.name = marker_array.name

    return s

    #============#
    #   Condor   #
    #============#

def Condor(marker_array,j):

    #=============================#
    #   Get Markers of interest   #
    #=============================#

    # first three markers
    A = np.matrix([[marker_array.marker[0].x],\
            [marker_array.marker[0].y],[marker_array.marker[0].z]]) # front_right

    B = np.matrix([[marker_array.marker[1].x],\
            [marker_array.marker[1].y],[marker_array.marker[1].z]]) # front_left

    C  = np.matrix([[marker_array.marker[2].x],\
            [marker_array.marker[2].y],[marker_array.marker[2].z]]) # back_right

    #========================#
    #   Calculate Centroid   #
    #========================#

#   refer to tutorial for a picture

    # Set up vectors
    BA = A-B
    BC = C-B
    # project BA onto BC
    proj = BA.T*BC
    # normalize
    proj = proj/(BC.T*BC)
    # project along BC to centroid
    center = B + proj[0,-1]*BC

    #================================#
    #   Calculate Angluar Position   #
    #================================#

    # axis of rotation

    axis = np.asmatrix(np.cross((A-center).T,(B-center).T).T)
    axis = axis/np.linalg.norm(axis)
    Rot45 = np.asmatrix(rotMatAnyAxis(axis,np.pi/4))

    # use this to rotate vectors to be in line with ardrone axes

    #===========#
    #   north   #
    #===========#

    # center to front right rotated 45 degrees about z axis (in xyz)
    north = Rot45*(A-center)/np.linalg.norm(A-center) # unit vector

    #==========#
    #   east   #
    #==========#

    east = Rot45*(C-center)/np.linalg.norm(C-center) # unit vector

    #==========#
    #   down   #
    #==========#

    # north cross east
    down = np.cross(north.T,east.T).T # unit vector
    if marker_array.name == 'Kahn':
        center = center + .085*down

    #=========================#
    #   roll, pitch and yaw   #
    #=========================#

    phi,theta,psi = NED2RPY(north, east, down)

    #=======================================#
    #   Store Pose for Velocity estimates   #
    #=======================================#

    global steps, v_mat
    # find indices of Beginning and End of submatrix
    Begin = j*steps
    End   = j*steps+steps

    # pushback values one step
    v_mat[Begin:End-1,:] = v_mat[Begin+1:End,:]
    v_mat[End-1,0] = center[0]
    v_mat[End-1,1] = center[1]
    v_mat[End-1,2] = center[2]
    v_mat[End-1,3] = phi*180/np.pi
    v_mat[End-1,4] = theta*180/np.pi
    v_mat[End-1,5] = psi*180/np.pi

    global rate
    t = np.asmatrix(np.linspace(0,.245,steps)).T
    X = np.hstack((np.asmatrix(np.ones(t.size)).T,t))
    # matrix of stored positions and angles
    A = v_mat[Begin:End,:]
    Ax     = A[0:,0]
    Ay     = A[0:,1]
    Az     = A[0:,2]
    Aphi   = A[0:,3]
    Atheta = A[0:,4]
    Apsi   = A[0:,5]

    #=========================#
    #   Estimate Velocities   #
    #=========================#

    Xinv = np.linalg.inv(X.T*X)
    vx = Xinv*X.T*Ax
    vy = Xinv*X.T*Ay
    vz = Xinv*X.T*Az
    vphi = Xinv*X.T*Aphi
    vtheta = Xinv*X.T*Atheta
    vpsi = Xinv*X.T*Apsi

    #=========================#
    #   Make States Message   #
    #=========================#

    s = States()
    s.x = center[0]
    s.y = center[1]
    s.z = center[2]
    s.phi = phi*180/np.pi
    s.theta = theta*180/np.pi
    s.psi = psi*180/np.pi
    s.u = vx[1,-1]
    s.v = vy[1,-1]
    s.w = vz[1,-1]
    s.p = vphi[1,-1]
    s.r = vpsi[1,-1]
    s.q = vtheta[1,-1]
    s.name = marker_array.name

    return s

    #==============#
    #   Pendulum   #
    #==============#

def Pendulum_tip(marker_array,j):

    #=============================#
    #   Get Markers of interest   #
    #=============================#

    # first three markers
    A = np.matrix([[marker_array.marker[0].x],\
            [marker_array.marker[0].y],[marker_array.marker[0].z]]) # front_right

    B = np.matrix([[marker_array.marker[1].x],\
            [marker_array.marker[1].y],[marker_array.marker[1].z]]) # front_left

    C  = np.matrix([[marker_array.marker[2].x],\
            [marker_array.marker[2].y],[marker_array.marker[2].z]]) # back_right

    #========================#
    #   Calculate Centroid   #
    #========================#

    center = np.divide(A+B+C,3)

    #====================================================#
    #   Store Pose for Velocity estimates for Pendulum   #
    #====================================================#

    global steps,v_mat
    # find indices of Beginning and End of submatrix
    Begin = j*steps
    End   = j*steps+steps

    # pushback values one step
    v_mat[Begin:End-1,:] = v_mat[Begin+1:End,:]
    v_mat[End-1,0] = center[0]
    v_mat[End-1,1] = center[1]
    v_mat[End-1,2] = center[2]

    global rate
    t = np.asmatrix(np.linspace(0,.245,steps)).T
    X = np.hstack((np.asmatrix(np.ones(t.size)).T,t))
    A = v_mat[Begin:End,:]
    # matrix of stored positions
    Ax     = A[:,0]
    Ay     = A[:,1]
    Az     = A[:,2]

    #=========================#
    #   Estimate Velocities   #
    #=========================#

    Xinv = np.linalg.inv(X.T*X)
    vx = Xinv*X.T*Ax
    vy = Xinv*X.T*Ay
    vz = Xinv*X.T*Az

    #=========================#
    #   Make States Message   #
    #=========================#

    s = States()
    s.x = center[0]
    s.y = center[1]
    s.z = center[2]
    s.u = vx[1,0]
    s.v = vy[1,0]
    s.w = vz[1,0]
    s.visible = True
    s.name = marker_array.name
    return s

    #=======================#
    #   Template Function   #
    #=======================#

def name_of_template(marker_array):

    #=============================#
    #   Get Markers of interest   #
    #=============================#

    # make vectors out of these and label them similar to cortex template
    A = np.matrix([[marker_array.marker[0].x],\
            [marker_array.marker[0].y],[marker_array.marker[0].z]]) # front_right

    B = np.matrix([[marker_array.marker[1].x],\
            [marker_array.marker[1].y],[marker_array.marker[1].z]]) # front_left

    C  = np.matrix([[marker_array.marker[2].x],\
            [marker_array.marker[2].y],[marker_array.marker[2].z]]) # back_right

    #========================#
    #   Calculate Centroid   #
    #========================#

    # Set up some vectors
    BA = A-B
    BC = C-B
    # project onto one another to get accurate centroid
    proj = BA.T*BC
    # normalize
    proj = proj/(BC.T*BC)
    # project along vector that passes through the centroid
    center = B + proj*BC

    #-------------------------------#
    #  Calculate Angluar Position   #
    #  Finding North, East and Down #
    #  Then Use Predefined Function #
    #-------------------------------#

    #===========#
    #   north   #
    #===========#

    # center to front right rotated 45 degrees about z axis (in xyz)
    north = (a-center)/np.norm(a-center) # unit vector

    #==========#
    #   east   #
    #==========#

    east = (c-center)/np.norm(c-center) # unit vector

    #==========#
    #   down   #
    #==========#

    # north cross east
    down = np.cross(north,east) # unit vector

    #=========================#
    #   roll, pitch and yaw   #
    #=========================#

    phi,theta,psi = NED2PRY(north, east, down)

    #=========================#
    #   Make States Message   #
    #=========================#

    states = States()
    states.x = center[0]
    states.y = center[1]
    states.z = center[2]
    states.phi = phi
    states.theta = theta
    states.psi = psi
    States.name = marker_array.name

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Mocap_State_Estimator')
    time_past = rospy.get_time()

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    sub  = rospy.Subscriber('/mocap_data' , Mocap_data, GetMocapData, queue_size = 1, buff_size = 2**24)
    rospy.spin()

