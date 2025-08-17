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
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
config = deviceHelper.variables

print("going to depth 0.6")
rc.set_absolute_yaw(0)
rc.go_to_depth(0.4)
rc.go_to_depth(0.8)

"""WAYPOINTS"""
#bin
rc.waypointNav(x = -0.7, y = 7.2)
rc.move_servo("/auv/devices/dropper")
time.sleep(0.5)
rc.move_servo("/auv/devices/dropper")
time.sleep(0.3)
rc.move_servo("/auv/devices/dropper")

"""GATE INTERSUB MISSION"""
try:
    rospy.loginfo("Gate Intersub ")
    gateIntersub = gate_intersub_mission.GateIntersubMission(robotControl=rc)
    gateIntersub.run()
    rospy.loginfo("GATE INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

#octagon
# rc.waypointNav(x = -3.502, y = 3.502)


#torpedo
rc.waypointNav(x = -8.0, y = 10.2)
rc.go_to_depth(1.2)
rc.go_to_heading(208)
rc.set_absolute_yaw(208)
rc.go_forward_distance(0.4)
rc.move_servo("/auv/devices/torpedo")
time.sleep(0.3)
rc.move_servo("/auv/devices/torpedo")
time.sleep(0.3)
rc.move_servo("/auv/devices/torpedo")

time.sleep(1)
rc.go_forward_distance(-2)

"""MODEMS + ROLL"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)


# # return home
# rc.waypointNav(x=0,y=0)

disarm.disarm()
rc.exit()