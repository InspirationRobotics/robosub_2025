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
rc.go_to_depth(0.8)

rospy.loginfo("Finish initialization")

navigate_with_heading("Gate")
navigate_with_heading("Slalom")
navigate_with_heading("Bin")
navigate_with_heading("Torpedo")
navigate_with_heading("Octagon")

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()