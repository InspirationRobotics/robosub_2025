"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

# print("20s before onyx initialize")
# time.sleep(22)

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
rc.set_absolute_yaw(0)
rc.go_to_depth(0.4)
rc.go_to_depth(0.8)
rospy.loginfo("Finish initialization")

# move forward
rc.go_forward_distance(2)

"""GATE INTERSUB MISSION"""
try:
    start_time = time.time()
    while time.time()-start_time<15:  # 30 s timeout
        rc.send_modem(addr="010",movement="Onyx_Finished")
        msg = rc.get_latest_modem()
        if msg is not None and msg=="Graey_Start":
            rospy.loginfo("Graey started, Onyx proceeding")
            break
        time.sleep(0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

rc.go_lateral_distance(2)

"""MODEMS + ROLL"""
try:
    start_time = time.time()
    while time.time()-start_time<15:  # 15 s timeout
        rc.send_modem(addr="010",movement="Onyx_Poles_Finished")
        msg = rc.get_latest_modem()
        if msg is not None and msg=="Graey_Return_ROLL":
            rospy.loginfo("Graey return home and roll")
            break
        time.sleep(0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

rc.go_forward_distance(-2)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()