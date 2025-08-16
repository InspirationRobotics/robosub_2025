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
config = deviceHelper.variables

# Load the JSON file
with open("./missions/waypoints_delta.json", "r") as file:
    waypoints = json.load(file)

# Dive down to desire depth
rc.set_absolute_yaw(0)
rc.go_to_depth(0.5)
rc.go_to_depth(1.3)

rospy.loginfo("Finish initialization")


"""GATE MISSION"""
try:
    # COIN FLIP
    rc.go_to_heading(0)
    rc.set_absolute_yaw(0)
    
    rc.go_by_time(f=2,t=3.25*5.8)
    rc.go_by_time(l=-2,t=3.25*-1.2)
    
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
    # Run the poles mission
    rc.set_absolute_yaw(0)
    rospy.loginfo("Start of poles mission...")
    
    # first row
    rc.go_by_time(f=2,t=3.25* 6.8)
    rc.grid_lateral(-0.8)

    # second row
    rc.grid_forward(1.5)
    rc.grid_lateral(0.75)

    # thrid row
    rc.grid_forward(1.5)
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
    rc.set_absolute_yaw(0)
    rc.grid_forward(0.6)
    rc.grid_lateral(1.4)
    rospy.loginfo("BIN APPROACH MISSION FINISHED")
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.5) # slightly longer delay and hope for higher chance of getting into the bin
    rc.move_servo("/auv/devices/dropper")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/dropper")
    rospy.loginfo("BIN drop MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING BIN MISSION")
    rospy.logerr(e)

"""OCTAGON MISSION"""
try:
    # navigate to octagon
    rc.go_to_depth(0.8)
    rc.grid_lateral(-1.2)    
    rc.grid_forward(20)

    # surface and resubmerge
    rc.go_to_heading(135)
    rc.set_absolute_yaw(135)
    time.sleep(2)
    rc.go_to_depth(0)
    time.sleep(4)
    rc.go_to_depth(0.5)
    
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
   rospy.logerr("ERROR DOING OCTAGON MISSION")
   rospy.logerr(e)


"""TORPEDO MISSION"""
try:
    """
    # pure distancce
    # set heading to 0
    rc.go_to_heading(0)
    rc.set_absolute_yaw(0)

    # go to torpedo waypoint
    rc.go_forward_distance(-9)
    rc.go_forward_distance(8)
    """

    rc.waypointNav(x=3.4,y=14.7)

    # align with the torpedo
    rc.go_to_heading(30)
    rc.set_absolute_yaw(30)
    time.sleep(2)
    
    # navigate to torpedo
    rc.go_forward_distance(0.6)
    
    time.sleep(1) # wait for stabilization and drift

    rc.move_servo("/auv/devices/torpedo")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/torpedo")
    time.sleep(0.3)
    rc.move_servo("/auv/devices/torpedo")
    rospy.loginfo("TORPEDO MISSION FINISHED")
    rc.go_forward_distance(-0.8)
except Exception as e:
    rospy.logerr("ERROR DOING TORPEDO MISSION")
    rospy.logerr(e)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()