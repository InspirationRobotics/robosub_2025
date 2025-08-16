import rospy
import time
import json

from auv.mission import bin_approach_mission, bin_drop_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
rc.activate_heading_control(False)
config = deviceHelper.variables
    
"""BIN DROP MISSION"""

try:
   rc.go_forward_distance(2)
   rc.go_forward_distance(-2)
   rc.go_forward_distance(2)
   rc.go_forward_distance(-2)
   rc.go_forward_distance(2)
   rc.go_forward_distance(-2)
   rc.go_forward_distance(2)
   rc.go_forward_distance(-2)
except Exception as e:
    rospy.logerr("ERROR MOVING FORWARD AND BACKWARD")
    rospy.logerr(e)
