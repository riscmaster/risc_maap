#!/usr/bin/env python

'''======================================================
    Created by:  	D. Spencer Maughan and Ishmaal Erekson
    Last updated: 	March 2015
    File name: 		DF_Plots.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('risc_msgs')
import rospy
import numpy as np
import matplotlib as mpl
from   mpl_toolkits.mplot3d import Axes3D
import pylab as p
import matplotlib.pyplot as plt
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import * # states,controls,trajectory
from sensor_msgs.msg import Joy

    #========================#
    #        Globals         #
    #========================#

rate               = 20 # Hz
states             = Cortex()
traj               = Trajectories()
ctrl               = Controls()
start_time	   = 0
euler_max          = 45*np.pi/180
Button_pushed      = False
plot_button        = 3


    #===================================#
    #        Plotting Variables         #
    #===================================#

states_of_interest = 16
storage_mat = np.asmatrix(np.zeros((1,states_of_interest)))
index    = [0]
name     = ['Initial']


    #==================#
    #    Get States    #
    #==================#

def GetStates(I):
    global states
    states = I

    #=========================#
    #    Get Joystick Data    #
    #=========================#

def GetJoy(I):
    global Button_pushed
    Button_pushed = I.buttons[plot_button]

    #======================#
    #    Get Trajectory    #
    #======================#

def GetTrajectory(I):
    global traj
    traj = I

    #====================#
    #    Get Controls    #
    #====================#

def GetControls(I):
    global ctrl
    ctrl = I


def Plots():
    global storage_mat, index, name
    if len(index) > 2:
        for i in range(len(index)-1):
            # assign data vectors
            f = index[i+1]
            if i+2 == len(index):
                b = -1
            else:
                b = index[i+2]
            x_act     = storage_mat[f:b,0]
            y_act     = storage_mat[f:b,1]
            z_act     = storage_mat[f:b,2]
            x_des     = storage_mat[f:b,3]
            y_des     = storage_mat[f:b,4]
            z_des     = storage_mat[f:b,5]
            phi_des   = storage_mat[f:b,6]
            theta_des = storage_mat[f:b,7]
            psi_des   = storage_mat[f:b,8]
            phi_act   = storage_mat[f:b,9]
            theta_act = storage_mat[f:b,10]
            psi_act   = storage_mat[f:b,11]
            xdot_err  = storage_mat[f:b,12]
            ydot_err  = storage_mat[f:b,13]
            zdot_err  = storage_mat[f:b,14]
            t         = storage_mat[f:b,15]

            # 3d plot
            plot3d(name[i+1],x_act,y_act,z_act,x_des,y_des,z_des)
            # Roll
            plot2d(name[i+1] + ' Roll',phi_act,phi_des,t,'Time (s)','Angle (Deg)')
            # Pitch
            plot2d(name[i+1] + ' Pitch',theta_act,theta_des,t,'Time (s)','Angle (Deg)')
            # Errors
            plot3err(name[i+1] + ' Position Errors',x_des-x_act,y_des-y_act,z_des-z_act,t,'Time (s)', 'Error (m)', 'x', 'y', 'z')
            plot3err(name[i+1] + ' Velocity Errors',xdot_err,ydot_err,zdot_err,t,'Time (s)', 'Error (m/s)', 'xdot', 'ydot', 'zdot')
            plt.show(block=False)
    else:
        rospy.loginfo("insufficient data")

        #==========================#
        #    Plotting Functions    #
        #==========================#

def plot3d(Traj_name,x_act,y_act,z_act,x_des,y_des,z_des):
    x_act=list(np.array(x_act).reshape(-1))
    y_act=list(np.array(y_act).reshape(-1))
    z_act=list(np.array(z_act).reshape(-1))
    x_des=list(np.array(x_des).reshape(-1))
    y_des=list(np.array(y_des).reshape(-1))
    z_des=list(np.array(z_des).reshape(-1))

    fig = plt.figure(Traj_name)
    ax = fig.gca(projection='3d')
    ax.plot(x_act, y_act, z_act,'k-', label='Actual')
    ax.plot(x_des, y_des, z_des,'r-', label='Desired')
    ax.legend()
    ax.set_title(Traj_name + ' Trajectory', fontsize=16)
    ax.set_xlabel(r'X (m)', fontsize=14)
    ax.set_ylabel(r'Y (m)', fontsize=14)
    ax.set_zlabel(r'Z (m)', fontsize=14)
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([0, 2])

def plot3err(plot_name,err1,err2,err3,time,xaxis_label, yaxis_label,label1, label2, label3):
    Err1    = list(np.array(err1).reshape(-1))
    Err2    = list(np.array(err2).reshape(-1))
    Err3    = list(np.array(err3).reshape(-1))
    time    = list(np.array(time).reshape(-1))

    fig = plt.figure(plot_name)
    plt.plot(time, Err1,'b-', label=label1)
    plt.plot(time, Err2,'k-', label=label2)
    plt.plot(time, Err3,'r-', label=label3)
    plt.legend()
    plt.title(plot_name, fontsize=16)
    plt.xlabel(xaxis_label, fontsize=14)
    plt.ylabel(yaxis_label, fontsize=14)
    plt.xlim((time[0],time[-1]))

    y_min = min([min(Err1),min(Err2),min(Err3)])
    y_min = y_min - .2*abs(y_min)
    y_max = max([max(Err1),max(Err2),max(Err3)])
    y_max = y_max + .2*abs(y_max)
    plt.ylim((y_min,y_max))

def plot2d(plot_name,actual_data,commanded_data,time,xaxis_label, yaxis_label):
    actual_data    = list(np.array(actual_data).reshape(-1))
    commanded_data = list(np.array(commanded_data).reshape(-1))
    time           = list(np.array(time).reshape(-1))

    fig = plt.figure(plot_name)
    plt.plot(time, actual_data, 'b-', label='actual')
    plt.plot(time, commanded_data,'r:', label='Commanded')
    plt.legend()
    plt.title(plot_name, fontsize=16)
    plt.xlabel(xaxis_label, fontsize=14)
    plt.ylabel(yaxis_label, fontsize=14)
    plt.xlim((time[0],time[-1]))

    y_min = min([min(actual_data),min(commanded_data)])
    y_min = y_min - .2*abs(y_min)
    y_max = max([max(actual_data),max(commanded_data)])
    y_max = y_max + .2*abs(y_max)
    plt.ylim((y_min,y_max))

    #==================#
    #    Datalogger    #
    #==================#

def Datalogger():
    global start_time,states,ctrl, traj, euler_max

    #======================================================#
    #    If all states of interest are present log data    #
    #======================================================#


    if len(traj.Obj) > 0  and len(states.Obj) > 0 and len(ctrl.Obj) > 0:
        global storage_mat
        rospy.loginfo("logging data...")
        x_act      = states.Obj[0].x
        y_act      = states.Obj[0].y
        z_act      = states.Obj[0].z
        x_des      = traj.Obj[0].x
        y_des      = traj.Obj[0].y
        z_des      = traj.Obj[0].z
        phi_traj   = ctrl.Obj[0].phi*euler_max*180/np.pi
        theta_traj = ctrl.Obj[0].theta*euler_max*180/np.pi
        psi_traj   = traj.Obj[0].psi*180/np.pi
        phi_cort   = states.Obj[0].phi
        theta_cort = states.Obj[0].theta
        psi_cort   = states.Obj[0].psi
        u_cort_err = traj.Obj[0].xdot - states.Obj[0].u
        v_cort_err = traj.Obj[0].ydot - states.Obj[0].v
        w_cort_err = traj.Obj[0].zdot - states.Obj[0].w
        t          = float(rospy.get_time() - start_time)
        new_stack = np.asmatrix(np.array([x_act, y_act, z_act, z_des, y_des, z_des,\
                     phi_traj, theta_traj, psi_traj, phi_cort, theta_cort, psi_cort,\
                     u_cort_err, v_cort_err, w_cort_err,t]))

        storage_mat = np.append(storage_mat,new_stack,0)

    #==========================================================================#
    #     If there is a new trajectory store the index and trajectory name     #
    #==========================================================================#

    global storage_mat, states_of_interest,name,index
    if len(traj.Obj) > 0 and name[-1] != traj.Obj[0].name:
        name.append(traj.Obj[0].name)
        index.append(storage_mat.shape[0] -1)
        start_time = rospy.get_time()

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    rospy.init_node('DF_Plotter')
    start_time = rospy.get_time()
    euler_max    = float(rospy.get_param("euler_angle_max", ".78537")) #in radians
    plot_button  = int(rospy.get_param("plot_button", "3"))
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
         sub_states = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
         sub_traj   = rospy.Subscriber('/trajectory',Trajectories, GetTrajectory)
         sub_cntrl  = rospy.Subscriber('/controls' , Controls, GetControls)
         sub_joy    = rospy.Subscriber('/joy' , Joy, GetJoy)
         Datalogger()
         if Button_pushed:
             Plots()
             answer = raw_input('Erase plots and reset datalogger?')
             if answer == 'y' or answer == 'yes' or answer == 'I guess' or answer == 'sure':
                 rospy.loginfo("Resetting datalogger and erasing plots...")
                 plt.clf()
                 start_time = rospy.get_time()
                 storage_mat = np.asmatrix(np.zeros((1,states_of_interest)))
                 plt.close('all')
             else:
                 plt.clf()
                 plt.close('all')
                 rospy.signal_shutdown(0)
         r.sleep()
