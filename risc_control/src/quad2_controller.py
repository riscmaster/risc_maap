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
import numpy

    #=======================#
    #    Messages Needed    #
    #=======================#

from sensor_msgs.msg import Joy		 # for joystick control
from risc_msgs.msg import Controls	 # for commands from Research Algorithms
from risc_msgs.msg import Waypoints  # for publishing waypoints
from risc_msgs.msg import Waypoint   # for initialization of waypoints
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
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
        self.subNavdata = rospy.Subscriber('/quad2/ardrone/navdata',Navdata,self.ReceiveNavdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/quad2/ardrone/land',Empty)
        self.pubTakeoff = rospy.Publisher('/quad2/ardrone/takeoff',Empty)
        self.pubReset   = rospy.Publisher('/quad2/ardrone/reset',Empty)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/quad2/cmd_vel',Twist)

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
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
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
Quad1           = 1
Quad2           = 7
Quad3           = 8
Quad4           = 9
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
Waynorthsouth	= 4
Wayeastwest	= 5

    #============================#
    #    Scale Factor Defaults   #
    #============================#

ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

    #======================================#
    #    Simple Waypoints for two Quads    #
    #======================================#

# North
n = Waypoint()
n.x = .935
n.y = .773
# East
e = Waypoint()
e.x = 1.0486
e.y = -.85
# South
s = Waypoint()
s.x = -.9978
s.y = -.889
# West
w = Waypoint()
w.x = -1.03
w.y =  .796
# one Quad north, One south
wp_north = Waypoints()
wp_north.Obj = [Waypoint()]*2
wp_north.Obj[0] = n
wp_north.Obj[1] = s
# one Quad east, One west
wp_east = Waypoints()
wp_east.Obj = [Waypoint()]*2
wp_east.Obj[0] = e
wp_east.Obj[1] = w
# one Quad south, One north
wp_south = Waypoints()
wp_south.Obj = [Waypoint()]*2
wp_south.Obj[0] = s
wp_south.Obj[1] = n
# one Quad west, One east
wp_west = Waypoints()
wp_west.Obj = [Waypoint()]*2
wp_west.Obj[0] = w
wp_west.Obj[1] = e
WP = Waypoints()
WP = wp_north

    #========================#
    #    Some More Globals   #
    #========================#

Ctrl = Controls()
Z = 0.0
# toggle flops
flop1 = True
flop2 = True
# selected quad integer
j = 0
COMMAND_PERIOD = 100 #ms

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

    if data.buttons[TakeoffLand]==1:
        if not flop2:
            rospy.loginfo("Land Button Pressed")
            controller.SendLand()
        if flop2:
            rospy.loginfo("Takeoff Button Pressed")
            controller.SendTakeoff()
        flop2 = not flop2
        time.sleep(.7)

        #=======================#
        #    Altitude Control   #
        #=======================#

    if data.buttons[Up]==1:
        Z =  1.0
    if data.buttons[Down]==1:
        Z = -1.0
    if data.buttons[Up]==0 and data.buttons[Down]==0:
        Z =  0.0

    #==============================================#
    #   Publish Waypoints using Joy-Axes buttons   #
    #==============================================#

    if data.axes[Waynorthsouth] > 0:
        WP = wp_north
    if data.axes[Waynorthsouth] < 0:
        WP = wp_south
    if data.axes[Wayeastwest] > 0:
        WP = wp_west
    if data.axes[Wayeastwest] < 0:
        WP = wp_east

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
            rospy.loginfo("Controller Command")
            rospy.set_param("controller_status",False)

        elif not flop1:
            rospy.loginfo("Waypoint Command")
            rospy.set_param("controller_status",True)
        time.sleep(.4)

    if flop1:
        # Joy Command
        controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw, Z/ScaleZ)
    if not flop1:
        # Waypoint Command
        controller.SetCommand(Ctrl.Obj[1].phi,Ctrl.Obj[1].theta,Ctrl.Obj[1].psi,Ctrl.Obj[1].alt)

    #=========================#
    #    Publish Waypoints    #
    #=========================#

    WP.Obj[0].z = 1
    WP.Obj[1].z = 1
#    pub.publish(WP)

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
    Quad1            = int   ( rospy.get_param("~Quad1",Quad1) )
    Quad2            = int   ( rospy.get_param("~Quad2",Quad2) )
    Quad3            = int   ( rospy.get_param("~Quad3",Quad3) )
    Quad4            = int   ( rospy.get_param("~Quad4",Quad4) )
    Waynorthsouth	 = int   ( rospy.get_param("~Waynorthsouth",Waynorthsouth) )
    Wayeastwest      = int   ( rospy.get_param("~Wayeastwest",Wayeastwest) )

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    controller = BasicDroneController()
    status = 0
    pub = rospy.Publisher('waypoints',Waypoints,queue_size = None)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        joy = rospy.Subscriber('/quad2/joy', Joy, ReceiveJoystickMessage)
        controls = rospy.Subscriber('/controls', Controls, GetControlCommands)
        Datahandler(status)
        r.sleep()

    # will only progress to here once the application has been shutdown
    rospy.signal_shutdown('Shutdown')
    sys.exit(status)

