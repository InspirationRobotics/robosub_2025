"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time

from auv.utils import deviceHelper
from auv.mission import poles_mission, intersub_com_mission, poles_mission_preset, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
config = deviceHelper.variables

"""COIN TOSS"""
try:
   rc.go_to_heading(0)
   rc.activate_heading_control(True)
   rc.set_absolute_yaw(0)
   rospy.loginfo("Robot heading set to gate heading")
except Exception as e:
    rospy.logerr("ERROR SETTING ROBOT HEADING")
    rospy.logerr(e)

print("going to depth 0.6")
rc.go_to_depth(0.6)

"""WAYPOINTS"""
#bin
rc.waypointNav(x = 0, y = 7.388)
#octagon
rc.waypointNav(x = -3.596, y = 3.440)
#torpedo
rc.waypointNav(x = -7.028, y = 6.903)