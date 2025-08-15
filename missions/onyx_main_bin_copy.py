"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

from auv.mission import poles_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

def navigate_with_heading(name):
    Waypoint = waypoints[name]
    rc.waypointNav(Waypoint["position"][0],Waypoint["position"][1])
    rc.go_to_heading(Waypoint["heading"])
    rospy.loginfo(f"Reached {name} waypoint")

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
config = deviceHelper.variables

# Load the JSON file
with open("./missions/waypoints.json", "r") as file:
    waypoints = json.load(file)

# Dive down to desire depth
rc.go_to_depth(1.0)

rospy.loginfo("Finish initialization")


# """GATE MISSION"""
# try:
#     # COIN FLIP
#     rc.go_to_heading(0)
#     rc.activate_heading_control(True)
#     rc.go_forward_distance(6)
#     rc.go_lateral_distance(-3)
#     rospy.loginfo("GATE MISSION FINISHED")
# except Exception as e:
#     rospy.logerr("ERROR DOING GATE MISSION")
#     rospy.logerr(e)

# """POLES MISSION"""
# try: 
#     # Run the poles mission
#     rospy.loginfo("Start of poles mission...")
#     poles = poles_mission.PoleSlalomMission(rc=rc,**config)
#     poles.run()
#     poles.cleanup()
#     print("[INFO] POLES MISSION COMPLETE")
# except Exception as e:
#     rospy.logerr("ERROR OCCUR IN POLES MISSION")
#     rospy.logerr(e)


"""BIN MISSION"""
try:
    rc.activate_heading_control(False)
    binApproach = bin_approach_mission.BinsApproachMission(rc=rc, **config)
    binApproach.run()
    binApproach.cleanup()
    rospy.loginfo("BIN APPROACH MISSION FINISHED")
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/dropper")
    rospy.loginfo("BIN drop MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING BIN MISSION")
    rospy.logerr(e)                              


print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()