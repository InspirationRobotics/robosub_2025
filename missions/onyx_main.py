"""
Running the gate mission (dead reckoning), style, buoy, and octagon missions.
Finals run for 2024 season. 
NOTE: This code did not give the intended result because the set_heading function only works when the differences between 
actual and desired heading are outside a certain range -- if these are too close together, the loop will stall (which prevented Onyx
from reaching the Octagon mission).
"""

import rospy
import time

from auv.mission import poles_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
config = deviceHelper.variables

# Dive down to desire depth
rc.go_to_depth(1.0)

rospy.loginfo("Finish initialization")

"""GATE MISSION"""
try:
    # COIN FLIP
    rc.go_to_heading(0)
    rc.go_forward_distance(8.0)
    rc.go_lateral_distance(-5.0)
    rospy.loginfo("GATE MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""POLES MISSION"""
try: 
    # Run the poles mission
    rospy.loginfo("Start of poles mission...")
    poles = poles_mission.PoleSlalomMission(rc=rc,**config)
    poles.run()
    poles.cleanup()
    print("[INFO] POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)

"""BIN MISSION"""
try:
    binApproach = bin_approach_mission.BinsApproachMission(rc=rc, **config)
    binApproach.run()
    binApproach.cleanup()
    rospy.loginfo("BIN APPROACH MISSION FINISHED")
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/dropper")
#    binDrop = bins_drop_mission.BinsDropMission(rc=rc, **config)
#    binDrop.run()
#    binDrop.cleanup()
    rospy.loginfo("BIN drop MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING BIN MISSION")
    rospy.logerr(e)

"""TORPEDO MISSION"""
try:
    torpedoApproach = torpedo_approach_mission.torpedoApproachMission(rc=rc, **config)
    torpedoApproach.run()
    torpedoApproach.cleanup()
    rc.move_servo("/auv/devices/torpedo")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/torpedo")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/torpedo")
    rospy.loginfo("TORPEDO MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING TORPEDO MISSION")
    rospy.logerr(e)


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