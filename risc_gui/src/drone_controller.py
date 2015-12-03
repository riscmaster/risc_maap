#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_gui')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class DroneController(object):
    selected_quad = 0

    def __init__(self):
        # Holds the current drone status
        self.status = [-1, -1, -1, -1]

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        #self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = [rospy.Publisher('quad1/ardrone/land',Empty), rospy.Publisher('quad2/ardrone/land',Empty), rospy.Publisher('quad3/ardrone/land',Empty), rospy.Publisher('quad4/ardrone/land',Empty)]
        self.pubTakeoff = [rospy.Publisher('quad1/ardrone/takeoff',Empty), rospy.Publisher('quad2/ardrone/takeoff',Empty), rospy.Publisher('quad3/ardrone/takeoff',Empty), rospy.Publisher('quad4/ardrone/takeoff',Empty)]
        self.pubReset   = [rospy.Publisher('quad1/ardrone/reset',Empty), rospy.Publisher('quad2/ardrone/reset',Empty), rospy.Publisher('quad3/ardrone/reset',Empty), rospy.Publisher('quad4/ardrone/reset',Empty)]


        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = [rospy.Publisher('quad1/cmd_vel',Twist), rospy.Publisher('quad2/cmd_vel',Twist), rospy.Publisher('quad3/cmd_vel',Twist), rospy.Publisher('quad4/cmd_vel',Twist)]


        # Setup regular publishing of control packets
        self.command = [Twist(), Twist(), Twist(), Twist()]
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/50000.0),self.SendCommand)

    def UpdatePublishers(self):
        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher(self.selected_quad + '/ardrone/land',Empty)
        self.pubTakeoff = rospy.Publisher(self.selected_quad + '/ardrone/takeoff',Empty)
        self.pubReset   = rospy.Publisher(self.selected_quad + '/ardrone/reset',Empty)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher(self.selected_quad + '/cmd_vel',Twist)

    def SetQuad(self, quad):
        self.selected_quad = quad
        #self.UpdatePublishers()

    def SetState(self, selected_quad, state):
        self.status[selected_quad] = state

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if self.selected_quad == -1: # -1 means send command to all quadrotors
            c = 0
            for takeoff_topic in self.pubTakeoff:
                if self.status[c] == DroneStatus.Landed:
                    takeoff_topic.publish(Empty())
                c = c + 1
        else: # send command to one quadrotor
            if self.status[self.selected_quad] == DroneStatus.Landed:
                self.pubTakeoff[self.selected_quad].publish(Empty())

    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        if self.selected_quad == -1: # -1 means send command to all quadrotors
            for land_topic in self.pubLand:
                land_topic.publish(Empty())
        else: # send command to one quadrotor
            self.pubLand[self.selected_quad].publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        if self.selected_quad == -1: # -1 means send command to all quadrotors
            for reset_topic in self.pubReset:
                reset_topic.publish(Empty())
        else: # send command to one quadrotor
            self.pubReset[self.selected_quad].publish(Empty())

    def SetCommand(self, selected_quad=0,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        # Called by the main program to set the current command
        self.command[selected_quad].linear.x  = pitch
        self.command[selected_quad].linear.y  = roll
        self.command[selected_quad].linear.z  = z_velocity
        self.command[selected_quad].angular.z = yaw_velocity

    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
#        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
#            self.pubCommand.publish(self.command)
        if self.selected_quad == -1: # -1 means send command to all quadrotors
            c = 0
            for command_topic in self.pubCommand:
                if self.status[c] == DroneStatus.Flying or self.status[c] == DroneStatus.GotoHover or self.status[c] == DroneStatus.Hovering:
                    command_topic.publish(self.command[c])
                c = c + 1
        else: # send command to one quadrotor
            #print self.command[self.selected_quad]
            if self.status[self.selected_quad] == DroneStatus.Flying or self.status[self.selected_quad] == DroneStatus.GotoHover or self.status[self.selected_quad] == DroneStatus.Hovering:
                self.pubCommand[self.selected_quad].publish(self.command[self.selected_quad])

