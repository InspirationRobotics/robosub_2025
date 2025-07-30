"""
Running the gate mission (dead reckoning), style, buoy, and octagon missions.
Finals run for 2024 season. 
NOTE: This code did not give the intended result because the set_heading function only works when the differences between 
actual and desired heading are outside a certain range -- if these are too close together, the loop will stall (which prevented Onyx
from reaching the Octagon mission).
"""

import rospy
import time

from auv.mission import octagon_approach_mission, intersub_com_mission
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
    # TODO utilize heading control to move through the gate and go to the next waypoint
    rc.movement(forward=2)
    time.sleep(3)
    rc.movement()

    rospy.loginfo("GATE MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""BIN MISSION"""
try:
    pass
    rospy.loginfo("BIN MISSION FINISHED")

except Exception as e:
    rospy.logerr("ERROR DOING BIN MISSION")
    rospy.logerr(e)
    pass

"""TORPEDO MISSION"""
try:
    pass
    rospy.loginfo("TORPEDO MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING TORPEDO MISSION")
    rospy.logerr(e)
    pass


"""OCTAGON MISSION"""
try:
    octagon = octagon_approach_mission.OctagonApproachMission(target=None, rc=rc, **config)
    time.sleep(2)
    octagon.run()
    octagon.cleanup()
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING OCTAGON MISSION")
    rospy.logerr(e)
    octagon.cleanup()

time.sleep(1.0)


"""MODEMS + ROLL"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)
print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()