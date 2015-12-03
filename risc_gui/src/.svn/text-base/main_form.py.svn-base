#!/usr/bin/env python
'''
        AR-Drone GUI interface
        Written by: Ashish Derhgawen
        Date: July 2014
'''
'''
    TODO:
    1. ReceivedNavdata() function only works for 2 quads right now. Needs to be fixed.
'''

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import roslib; roslib.load_manifest('ardrone_gui')
roslib.load_manifest('risc_msgs')
import rospy;

from sensor_msgs.msg import Joy # for joystick control
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from risc_msgs.msg import Controls # for commands from research algorithms
from risc_msgs.msg import Waypoints # for publishing waypoints
from risc_msgs.msg import Waypoint # for initializing of waypoint
from risc_msgs.msg import Cortex

from std_msgs.msg import String

from drone_controller import DroneController

import sys
from subprocess import Popen

import main_form_ui # GUI interface is defined in main_form_ui.py (which is created using QT 4 Designer)
import waypoint_form

from scipy.interpolate import interp1d # for mapping distances to time durations (for waypoint commands)
import math
import copy

# An enumeration of Drone statuses
from drone_status import DroneStatus

class MainDialog(QDialog, main_form_ui.Ui_mainDialog):
    # Custom signals
    batterySignal = pyqtSignal(int)
    rostopicDisplaySignal = pyqtSignal(str)
    landed = True # Flop toggle


    # Global variables
    selected_quad    = "quad1"
    quad_identifiers = {'quad1' : 0, 'quad2' : 1, 'quad3' : 2, 'quad4' : 3, 'all' : -1} # Dictionary to map quadrotor names to numeric identifiers
    waypoint_index   = 0 # index of waypoint to be published (if publishing waypoints)
    arena_x_mapping  = interp1d([-275,255],[2.146,-1.3692])
    arena_y_mapping  = interp1d([-190,170],[1.337,-1.660])


    #========================================#
    #         Joystick buttons/axes          #
    #========================================#

    joy_data     =    Joy()
    joy_data.buttons = [0]*12
    joy_data.axes   = [0.0]*8
    ShutDownNode    = 0
    ButtonEmergency = 1
    TakeoffLand     = 2
    Up              = 3
    Down            = 4
    JoytoWay        = 5
    AxisRoll        = 3
    AxisPitch       = 4
    AxisYaw         = 0

    #============================#
    #    Scale Factor Defaults   #
    #============================#

    ScaleRoll       = 1.0
    ScalePitch      = 1.0
    ScaleYaw        = 1.0
    ScaleZ          = 1.0

    def __init__(self, parent=None):
        super(MainDialog, self).__init__(parent)

        self.setupUi(self)
        self.drone_status = -1
        self.waypoints = {}
        self.waypoint_publish_rate = 1.0

        self.takeoffButton.clicked.connect(self.takeoffButton_clicked)
        self.landButton.clicked.connect(self.landButton_clicked)
        self.emergencyButton.clicked.connect(self.emergencyButton_clicked)

        self.quad1Button.clicked.connect(self.quad1Button_clicked)
        self.quad2Button.clicked.connect(self.quad2Button_clicked)
        self.quad3Button.clicked.connect(self.quad3Button_clicked)
        self.quad4Button.clicked.connect(self.quad4Button_clicked)
        self.allQuadButton.clicked.connect(self.allQuadButton_clicked)
        self.rostopicCombo.currentIndexChanged.connect(self.rostopicCombo_indexChanged)
        self.joystickRadioButton.clicked.connect(self.joystickRadioButton_toggled)
        self.controllerRadioButton.toggled.connect(self.controllerRadioButton_toggled)
        self.waypointDesignerButton.clicked.connect(self.waypointDesignerButton_clicked)
        self.publishWaypointCheckBox.clicked.connect(self.publishWaypointCheckBox_clicked)
        self.rvizButton.clicked.connect(self.rvizButton_clicked)

        self.batterySignal.connect(self.batteryBar.setValue)
        self.rostopicDisplaySignal.connect(self.rostopicDisplay.setText)

        self.subNavdata_quad1 = rospy.Subscriber('quad1/ardrone/navdata',Navdata, self.ReceiveNavdata_quad1)
        self.subNavdata_quad2 = rospy.Subscriber('quad2/ardrone/navdata',Navdata, self.ReceiveNavdata_quad2)
        self.subNavdata_quad3 = rospy.Subscriber('quad3/ardrone/navdata',Navdata, self.ReceiveNavdata_quad3)
        self.subNavdata_quad4 = rospy.Subscriber('quad4/ardrone/navdata',Navdata, self.ReceiveNavdata_quad4)

        #self.subData = rospy.Subscriber(self.selected_quad + '/ardrone/navdata', String, self.displayTopic)
        self.subData = rospy.Subscriber('/cortex', Cortex, self.displayTopic)
        self.subData = rospy.Subscriber('/cortex', Cortex, self.ReceivedCortexData)

        self.joy = rospy.Subscriber('/joy', Joy, self.ReceivedJoystickData)
        self.waypointPublisher = rospy.Publisher('waypoints',Waypoints,queue_size = None)

        self.controller_topic = None
        Ctrl = Controls()


    def rvizButton_clicked(self):
        Popen("rosrun rviz rviz -d ~/.rviz/risc.rviz", shell=True)


    def distance(self, pt1, pt2): # Calculates distance between two points
        dx = pt1[0] - pt2[0]
        dy = pt1[1] - pt2[1]

        distance = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
        return distance

    def getKey(self, item): # Returns key of list to be sorted
        return item[0]

    def waypointDesignerButton_clicked(self):
        dialog = waypoint_form.WaypointsDialog()

        self.waypoints = []

        if dialog.exec_():
            #self.waypoints = dialog.scene.items()
            #self.waypoints ={}
            for item in dialog.waypoints.items():
                # Format for waypoint items: (serial_numer, assigned_quad, (x,y), altitude)
                self.waypoints.append([item[1][0], item[1][1], [item[0].rect().x(), item[0].rect().y()],item[1][2]])

            # Sort waypoints using their serial number as the key
            self.waypoints = sorted(self.waypoints, key=self.getKey)

            print '###########'
            for item in self.waypoints:
                print item
            print '###########'

            self.waypoint_publish_rate = dialog.rateDial.value() / 10.0
        else:
            print 'cancelled'

    def update_subscriptions(self):
        #self.subNavdata.unregister()
        ##self.subData.unregister()
        #self.subNavdata = rospy.Subscriber(self.selected_quad + '/ardrone/navdata',Navdata,self.ReceiveNavdata)
        ##self.subData = rospy.Subscriber(self.selected_quad + '/ardrone/navdata', String,self.displayTopic)
        controller.SetQuad(self.quad_identifiers[self.selected_quad])

    def displayTopic(self, data):
        self.rostopicDisplaySignal.emit(str(data))

    def GetControlCommands(self, ctrl):
        if not self.controllerRadioButton.isChecked():
            return

        len_ctrl = len(ctrl.Obj)
        selected_quad = self.quad_identifiers[self.selected_quad]

        if self.selected_quad != "all":
            if len_ctrl >= selected_quad:
                controller.SetCommand(selected_quad, ctrl.Obj[selected_quad - 1].phi, ctrl.Obj[selected_quad - 1].theta, ctrl.Obj[selected_quad - 1].psi, ctrl.Obj[selected_quad - 1].alt)
            else:
                controller.SetCommand(selected_quad, ctrl.Obj[0].phi, ctrl.Obj[0].theta, ctrl.Obj[0].psi, ctrl.Obj[0].alt)

        else: # all selected
            # loop through all control objects and assign vales to appropriate quadrotors
            for i in range(len_ctrl):
                controller.SetCommand(i, ctrl.Obj[i].phi, ctrl.Obj[i].theta, ctrl.Obj[i].psi, ctrl.Obj[i].alt)

    def rostopicCombo_indexChanged(self, index):
        pass
       # selected_topic = self.selected_quad + '/ardrone/' + self.rostopicCombo.currentText()
       # print "Selected Topic: " + selected_topic
       # self.subData = rospy.Subscriber(selected_topic, String,self.displayTopic)

    def ReceivedJoystickData(self, data):
        if data.buttons[self.JoytoWay] == 1:
            if self.joystickRadioButton.isChecked():
                self.controllerRadioButton.setChecked(True)
            else:
                self.joystickRadioButton.setChecked(True)

        if not self.joystickRadioButton.isChecked():
            return

        Z = 0.0

        self.joy_data = data
        selected_quad = self.quad_identifiers[self.selected_quad]

        if data.buttons[self.ButtonEmergency] == 1:
            rospy.loginfo("Emergency button pressed")
            controller.SendEmergency()

        if data.buttons[self.TakeoffLand] == 1:
            if self.landed:
                rospy.loginfo("Takeoff button pressed")
                controller.SendTakeoff()
                self.landed = False
            else:
                rospy.loginfo("Landing")
                controller.SendLand()
                self.landed = True

        if data.buttons[self.Up] == 1:
            Z = 1.0
        elif data.buttons[self.Down] == 1:
            Z = -1.0
        else:
            Z = 0.0

        controller.SetCommand(selected_quad, data.axes[self.AxisRoll]/self.ScaleRoll,data.axes[self.AxisPitch]/self.ScalePitch,data.axes[self.AxisYaw]/self.ScaleYaw, Z/self.ScaleZ)

    def ReceivedCortexData(self, param):
        if not self.publishWaypointCheckBox.isChecked():
            return

        # Publish a waypoint

        x = self.arena_x_mapping(self.waypoints[self.waypoint_index][2][0]) # Map to arena coordinates
        y = self.arena_y_mapping(self.waypoints[self.waypoint_index][2][1]) # Map to arena coordinates

        #print x,y

        WP = Waypoints()
        WP.Obj = [Waypoints()]*1
        wp1 =  Waypoint()
        wp1.x = x
        wp1.y = y
        wp1.z = 1
        WP.Obj = [wp1]

        self.waypointPublisher.publish(WP)


    def waypoints_callback(self, param): # Publishes waypoints

        if not self.publishWaypointCheckBox.isChecked():
            return

        if self.waypoints is not None:
            wp = self.waypoints[self.waypoint_index][2]

            # If last waypoint, set next waypoint to first waypoint
            if self.waypoint_index == (len(self.waypoints) - 1):
                next_wp = self.waypoints[0][2]
                self.waypoint_index = 0
            else:
                next_wp = self.waypoints[self.waypoint_index + 1][2]
                self.waypoint_index += 1

            distance = self.distance(wp, next_wp)
            distance_time = interp1d([0,630],[0,self.waypoint_publish_rate]) # Distance to time mapping
            #print 'distance time:', distance_time(distance)
            duration = distance_time(distance)

            if duration == 0.0:
                duration = 0.5

            rospy.Timer(rospy.Duration(duration), self.waypoints_callback, True) # Reset timer to call this function. Duration is proportional to distance between current waypoint and next waypoint
            #print self.waypoint_index, wp, next_wp, distance
            #print distance


    def publishWaypointCheckBox_clicked(self):
# Subscribe to cortex here..and pass states into publish_waypoint() function
        if self.publishWaypointCheckBox.isChecked():
            rospy.Timer(rospy.Duration(1), self.waypoints_callback, True)

    def joystickRadioButton_toggled(self):
        if self.joystickRadioButton.isChecked():
            # Check if we are already subscribed to a topic. If yes, unsubsubscribe before we continue
            if self.controller_topic is not None:
                self.controller_topic.unregister()

    def controllerRadioButton_toggled(self):
        if self.controllerRadioButton.isChecked():
            self.controller_topic = rospy.Subscriber('/controls', Controls, self.GetControlCommands)

    def controllerEnabledCheckbox_clicked(self): ## OBSOLETE ##
        if self.controllerEnabledCheckBox.isChecked() == False:
            if self.controller_topic is not None:
                self.controller_topic.unregister()
            controller.SetCommand(0,0.0,0.0,0.0,0.0)
            controller.SetCommand(1,0.0,0.0,0.0,0.0)


    def takeoffButton_clicked(self):
        print "takeoff!!"
        controller.SendTakeoff()


    def landButton_clicked(self):
        print "land"
        controller.SendLand()

    def emergencyButton_clicked(self):
        print "EMERGENCY!"
        controller.SendEmergency()

    def quad1Button_clicked(self):
        self.quad2Button.setChecked(False)
        self.quad3Button.setChecked(False)
        self.quad4Button.setChecked(False)
        self.allQuadButton.setChecked(False)
        self.batteryBar.setEnabled(True)

        self.selected_quad = "quad1"

        self.update_subscriptions()

    def quad2Button_clicked(self):
        self.quad1Button.setChecked(False)
        self.quad3Button.setChecked(False)
        self.quad4Button.setChecked(False)
        self.allQuadButton.setChecked(False)
        self.batteryBar.setEnabled(True)

        self.selected_quad = "quad2"

        self.update_subscriptions()

    def quad3Button_clicked(self):
        self.quad2Button.setChecked(False)
        self.quad1Button.setChecked(False)
        self.quad4Button.setChecked(False)
        self.allQuadButton.setChecked(False)
        self.batteryBar.setEnabled(True)

        self.selected_quad = "quad3"

        self.update_subscriptions()

    def quad4Button_clicked(self):
        self.quad2Button.setChecked(False)
        self.quad3Button.setChecked(False)
        self.quad1Button.setChecked(False)
        self.allQuadButton.setChecked(False)
        self.batteryBar.setEnabled(True)

        self.selected_quad = "quad4"

        self.update_subscriptions()

    def allQuadButton_clicked(self):
        self.quad2Button.setChecked(False)
        self.quad3Button.setChecked(False)
        self.quad1Button.setChecked(False)
        self.quad4Button.setChecked(False)
        self.batteryBar.setEnabled(False)

        self.selected_quad = "all"

        self.update_subscriptions()

    def ReceiveNavdata_quad1(self, navdata):
        controller.SetState(0, navdata.state)
        if self.quad1Button.isChecked():
            self.batterySignal.emit(int(navdata.batteryPercent))
           # if self.rostopicCombo.currentText() == "navdata":
           #     self.rostopicDisplaySignal.emit(str(navdata))

    def ReceiveNavdata_quad2(self, navdata):
        controller.SetState(1, navdata.state)
        if self.quad2Button.isChecked():
            self.batterySignal.emit(int(navdata.batteryPercent))

    def ReceiveNavdata_quad3(self, navdata):
        controller.SetState(2, navdata.state)
        if self.quad3Button.isChecked():
            self.batterySignal.emit(int(navdata.batteryPercent))

    def ReceiveNavdata_quad4(self, navdata):
        controller.SetState(3, navdata.state)
        if self.quad4Button.isChecked():
            self.batterySignal.emit(int(navdata.batteryPercent))

# Setup the application
if __name__=='__main__':
    rospy.init_node('ardrone_gui')
    controller = DroneController()


    app = QApplication(sys.argv)
    form = MainDialog()
    form.show()

    #========================================#
    #    Load Parameters from Launch File    #
    #========================================#

    form.ButtonEmergency  = int   ( rospy.get_param("~ButtonEmergency",form.ButtonEmergency) )
    form.TakeoffLand      = int   ( rospy.get_param("~TakeoffLand",form.TakeoffLand) )
    form.AxisRoll         = int   ( rospy.get_param("~AxisRoll",form.AxisRoll) )
    form.AxisPitch        = int   ( rospy.get_param("~AxisPitch",form.AxisPitch) )
    form.AxisYaw          = int   ( rospy.get_param("~AxisYaw",form.AxisYaw) )
    form.Up               = int   ( rospy.get_param("~Up",form.Up) )
    form.Down             = int   ( rospy.get_param("~Down",form.Down) )
    form.ScaleRoll        = float ( rospy.get_param("~ScaleRoll",form.ScaleRoll) )
    form.ScalePitch       = float ( rospy.get_param("~ScalePitch",form.ScalePitch) )
    form.ScaleYaw         = float ( rospy.get_param("~ScaleYaw",form.ScaleYaw) )
    form.ScaleZ           = float ( rospy.get_param("~ScaleZ",form.ScaleZ) )
    form.ShutDownNode     = int   ( rospy.get_param("~ShutDownNode",form.ShutDownNode) )
    form.JoytoWay         = int   ( rospy.get_param("~JoytoWay",form.JoytoWay) )

    app.exec_()
    rospy.signal_shutdown('I am dead')
    #sys.exit(status)


