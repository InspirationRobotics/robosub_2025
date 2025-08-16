"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

from auv.mission import poles_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

def navigate_to(name):
    Waypoint = waypoints[name]
    rc.waypointNav(Waypoint["position"][0],Waypoint["position"][1])
    rospy.loginfo(f"Reached {name} waypoint")

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
rc.activate_heading_control(False)
config = deviceHelper.variables

# Load the JSON file
with open("./missions/waypoints_delta.json", "r") as file:
    waypoints = json.load(file)

# Dive down to desire depth
rc.go_to_depth(0.8)

rospy.loginfo("Finish initialization")


"""GATE MISSION"""
try:
    navigate_to("G1")
    rospy.loginfo("GATE MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""GATE INTERSUB MISSION"""
try:
    rospy.loginfo("Gate Intersub ")
    gateIntersub = gate_intersub_mission.GateIntersubMission(robotControl=rc)
    gateIntersub.run()
    rospy.loginfo("GATE INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""SLALOM MISSION"""

try: 
    rospy.loginfo("Start of poles mission...")
    navigate_to("S1")
    navigate_to("S2")
    print("[INFO] POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)

"""MODEMS + ROLL"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)

"""BIN MISSION"""
try:
    navigate_to("B1")
    rospy.loginfo("BIN APPROACH MISSION FINISHED")
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.5) # slightly longer delay and hope for higher chance of getting into the bin
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

"""OCTAGON MISSION"""
try:
    navigate_to("O1")
    rc.go_to_heading(45)
    rc.go_to_depth(0)
    time.sleep(7)
    rc.go_to_depth(0.8)
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
   rospy.logerr("ERROR DOING OCTAGON MISSION")
   rospy.logerr(e)


"""TORPEDO MISSION"""
try:
    navigate_to("T1")
    # align with the torpedo
    rc.go_to_heading(27)
    
    # Use cv to navigate to torpedo
    torpedoApproach = torpedo_approach_mission.torpedoApproachMission(rc=rc, **config)
    torpedoApproach.run()
    torpedoApproach.cleanup()
    rc.move_servo("/auv/devices/torpedo")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/torpedo")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/torpedo")
    rospy.loginfo("TORPEDO MISSION FINISHED")
    rc.go_forward_distance(-1.5)
except Exception as e:
    rospy.logerr("ERROR DOING TORPEDO MISSION")
    rospy.logerr(e)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()