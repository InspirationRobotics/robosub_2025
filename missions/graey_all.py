"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

print("20s before onyx initialize")
time.sleep(22)

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
with open("./missions/waypoints_thrid_chance.json", "r") as file:
    waypoints = json.load(file)

# Dive down to desire depth
rc.set_absolute_yaw(0)
rc.go_to_depth(1.2)

rospy.loginfo("Finish initialization")


"""GATE MISSION"""
try:
    navigate_to("G1")
    rospy.loginfo("GATE MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

rollMission = intersub_com_mission.intersubComMission()
rollMission.do_roll()

"""SLALOM MISSION"""

try: 
    rospy.loginfo("Start of poles mission...")
    navigate_to("S1")
    print("[INFO] POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)


"""OCTAGON MISSION"""
try:
    rc.go_to_depth(0.6)
    navigate_to("O1")
    rc.go_to_heading(-135)
    rc.set_absolute_yaw(-135)
    rc.go_to_depth(0)
    time.sleep(4)
    rc.go_to_depth(0.6)
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
   rospy.logerr("ERROR DOING OCTAGON MISSION")
   rospy.logerr(e)

"""Return home"""
rc.waypointNav(0,0)
print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()