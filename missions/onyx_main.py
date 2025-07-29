"""
Running the gate mission (dead reckoning), style, buoy, and octagon missions.
Finals run for 2024 season. 
NOTE: This code did not give the intended result because the set_heading function only works when the differences between 
actual and desired heading are outside a certain range -- if these are too close together, the loop will stall (which prevented Onyx
from reaching the Octagon mission).
"""

import rospy
import time

from auv.mission import octagon_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
config = deviceHelper.variables

rc.set_absolute_z(0.5)
time.sleep(10)
rospy.loginfo("Finish initialization")

"""GATE MISSION"""
try:
    rc.movement(forward=2)
    time.sleep(3)
    rc.movement()
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""POST-GATE STABILIZATION"""
try:
    rospy.loginfo("Stabilizing after gate traversal")
    rc.movement()  # zero all motion
    rc.activate_heading_control(activate=False)   # release yaw lock
    # rc.set_control_mode('depth_hold')
    rc.set_absolute_z(0.5)
    # rospy.sleep(2)
except Exception as e:
    rospy.logerr("ERROR DURING POST-GATE STABILIZATION")
    rospy.logerr(e)

"""OCTAGON MISSION"""
try:
    octagon = octagon_approach_mission.OctagonApproachMission(rc=rc, **config)
    octagon.run()
    octagon.cleanup()
except Exception as e:
    rospy.logerr("ERROR DOING OCTAGON MISSION")
    rospy.logerr(e)
    octagon.cleanup()

time.sleep(1.0)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()