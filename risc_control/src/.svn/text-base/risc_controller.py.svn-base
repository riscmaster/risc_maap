#!/usr/bin/env python
'''======================================================
    Created by:  	D. Spencer Maughan
    Last updated: 	June  2014
    File name:  	risc_controller.py
    Organization:	RISC Lab, Utah State University
 ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import message_filters
import rospy
import time
from math import *
import numpy as np

    #=======================#
    #    Messages Needed    #
    #=======================#

from sensor_msgs.msg import Joy		 # for joystick control
from risc_msgs.msg import *	         # for custom messages
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from std_msgs.msg import Bool       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

    #=======================#
    #    Some Classes       #
    #=======================#

class DroneStatus(object):
    Emergency = 0
    Inited    = 1
    Landed    = 2
    Flying    = 3
    Hovering  = 4
    Test      = 5
    TakingOff = 6
    GotoHover = 7
    Landing   = 8
    Looping   = 9

class BasicDroneController(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty,queue_size = 100)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size = 100)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty,queue_size = 100)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size = 100)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

    def ReceiveNavdata(self,navdata):
        # Although there is a lot of data in this packet, we're only interested in the state at the moment
        self.status = navdata.state

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if(self.status == DroneStatus.Landed):
            self.pubTakeoff.publish(Empty())

    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())

    def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity

    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        global Simulation
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering or Simulation:
            self.pubCommand.publish(self.command)

    #=======================#
    #    Some Classes       #
    #=======================#

from drone_video_display import DroneVideoDisplay
from PySide import QtCore, QtGui

    #==========================#
    #    Button/Axis Defaults  #
    #==========================#

data 		= Joy()
data.buttons 	= [0]*12
data.axes 	= [0.0]*8
ShutDownNode    = 0
ButtonEmergency = 1
TakeoffLand     = 2
Up              = 3
Down            = 4
JoytoWay        = 5
AxisRoll        = 6
AxisPitch       = 7
AxisYaw         = 3

    #============================#
    #    Scale Factor Defaults   #
    #============================#

ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

    #=====================#
    #    In Simulation?   #
    #=====================#

Simulation      = False

    #========================#
    #    Some More Globals   #
    #========================#

euler_max = 0.349066
delay = .3
g     = 9.81
m     = .45
zdot_act = 0
Ctrl = Controls()
Z = 0.0
# toggle flops
flop1 = True
flop2 = True
# selected quad integer
j = 0
COMMAND_PERIOD = 100 #ms

    #================#
    #    Publisher   #
    #================#

pub = rospy.Publisher('controller_status',Bool,queue_size = 1)

    #==================#
    #    Get States    #
    #==================#

def GetStates(S):
    global zdot_act
    zdot_act = S.Obj[0].w


    #============================#
    #    Get Control Commands    #
    #============================#

def GetControlCommands(cmd):
    global Ctrl
    Ctrl = cmd

    #=================================#
    #    Get Joy Buttons/Axes Data    #
    #=================================#

def ReceiveJoystickMessage(Data):
    global data
    data = Data

    #=========================#
    #   Rotation Functions    #
    #=========================#

def Rb_v2(phi):
    Rot = np.asmatrix(np.array([[ cos(phi), 0,sin(phi)],\
                                [        0, 1,       0],\
                                [-sin(phi), 0,cos(phi)]]))
    return Rot

def Rv2_v1(theta):
    Rot = np.asmatrix(np.array([[1,           0,          0],\
                                [0, cos(theta), -sin(theta)],\
                                [0, sin(theta),  cos(theta)]]))
    return Rot

    #===================================#
    #    Handle all Joy/Control Data    #
    #===================================#

def Datahandler(status):
    global data, Ctrl, flop1,flop2,j,Z,wp_north,\
            wp_south, wp_east, wp_west,WP

    #=================#
    #    Emergency    #
    #=================#

    if data.buttons[ButtonEmergency]==1:
        rospy.loginfo("Emergency Button Pressed")
        controller.SendEmergency()
        time.sleep(.5)

    #===========================#
    #    Takeoff/Land Toggle    #
    #===========================#

    if data.buttons[TakeoffLand]==1 and not Simulation:
        if not flop2:
            rospy.loginfo("Land Button Pressed")
            controller.SendLand()
        if flop2:
            rospy.loginfo("Takeoff Button Pressed")
            controller.SendTakeoff()
        flop2 = not flop2
        time.sleep(.7)
    if data.buttons[TakeoffLand] == 1 and Simulation:
        rospy.set_param("RESET",True)
        rospy.sleep(.5)
    if data.buttons[TakeoffLand] == 0 and Simulation:
        rospy.set_param("RESET",False)

        #=======================#
        #    Altitude Control   #
        #=======================#

    if data.buttons[Up]==1:
        Z =  1.0
    if data.buttons[Down]==1:
        Z = -1.0
    if data.buttons[Up]==0 and data.buttons[Down]==0:
        Z =  0.0

    #================#
    #    Shutdown    #
    #================#

    if data.buttons[ShutDownNode]==1:
        rospy.signal_shutdown("Shutting Down...  Please close drone viewing window.")
        import sys
        sys.exit(status)

        #======================================#
        #    Waypoint/Joy Controller Toggle    #
        #======================================#

    if data.buttons[JoytoWay]==1:
        flop1 = not flop1
        if flop1:
            rospy.loginfo("Manual Control")
            rospy.set_param("controller_status",False)

        elif not flop1:
            rospy.loginfo("Autonomous Control")
            rospy.set_param("controller_status",True)
        time.sleep(.4)

    pub.publish(not flop1)

    if flop1:
        # manual Command
        controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw, Z/ScaleZ)
    if not flop1:
        global euler_max, delay, zdot_act,g,m
        # Autonomous Command
        Ac = Rb_v2(Ctrl.Obj[0].phi*euler_max)*Rv2_v1(Ctrl.Obj[0].phi*euler_max)*np.matrix([[0],[0],[Ctrl.Obj[0].T]])
        zdot_c = zdot_act + delay*(Ac[2,-1]/m-g)
        controller.SetCommand(Ctrl.Obj[0].phi,Ctrl.Obj[0].theta,Ctrl.Obj[0].psi,zdot_c)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('risc_hand_controller')

    #========================================#
    #    Load Parameters from Launch File    #
    #========================================#

    JoytoWay         = int   ( rospy.get_param("~JoytoWay",JoytoWay) )
    ButtonEmergency  = int   ( rospy.get_param("~ButtonEmergency",ButtonEmergency) )
    TakeoffLand      = int   ( rospy.get_param("~TakeoffLand",TakeoffLand) )
    AxisRoll         = int   ( rospy.get_param("~AxisRoll",AxisRoll) )
    AxisPitch        = int   ( rospy.get_param("~AxisPitch",AxisPitch) )
    AxisYaw          = int   ( rospy.get_param("~AxisYaw",AxisYaw) )
    Up               = int   ( rospy.get_param("~Up",Up) )
    Down             = int   ( rospy.get_param("~Down",Down) )
    ScaleRoll        = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
    ScalePitch       = float ( rospy.get_param("~ScalePitch",ScalePitch) )
    ScaleYaw         = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
    ScaleZ           = float ( rospy.get_param("~ScaleZ",ScaleZ) )
    ShutDownNode     = int   ( rospy.get_param("~ShutDownNode",ShutDownNode) )
    Simulation       = bool  ( rospy.get_param("~Simulation", "False"))
    euler_max        = float ( rospy.get_param("~euler_angle_max", "0.349066"))
    delay            = float ( rospy.get_param("~delay", "0.3"))

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    controller = BasicDroneController()
    status = 0
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        joy = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)
        controls = rospy.Subscriber('/controls', Controls, GetControlCommands)
        sub_states = rospy.Subscriber('/cortex_raw', Cortex, GetStates)
        Datahandler(status)
        r.sleep()

    # will only progress to here once the application has been shutdown
    rospy.signal_shutdown('Shutdown')
    sys.exit(status)

