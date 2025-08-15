"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

time.sleep(30) # wait for tether disconnection


from auv.mission import poles_mission, poles_with_com_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission, gate_intersub_mission
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
with open("./missions/waypoints_b1.json", "r") as file:
    waypoints = json.load(file)

# Dive down to desire depth
rc.go_to_depth(1.2)

rospy.loginfo("Finish initialization")


"""GATE MISSION"""
try:
    # COIN FLIP
    rc.go_to_heading(0)
    rc.activate_heading_control(True)
    rc.set_absolute_yaw(0)
    rc.go_forward_distance(5.57)
    rc.go_lateral_distance(0.8)
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
# navigate_with_heading("S1")
try: 
    # Run the poles mission
    rc.activate_heading_control(False)
    rc.go_to_heading(33.69)
    rc.activate_heading_control(True)
    rc.set_absolute_yaw(33.69)
    rospy.loginfo("Start of poles mission...")
    poles = poles_with_com_mission.PoleSlalomMission(rc=rc,**config)
    poles.run()
    poles.cleanup()
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

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()