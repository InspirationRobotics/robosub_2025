"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

time.sleep(30) # wait for tether disconnection


from auv.mission import poles_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission, gate_intersub_mission
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
with open("./missions/waypoints_alpha.json", "r") as file:
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
    rc.go_forward_distance(6)
    rc.go_lateral_distance(-0.8)
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
    rc.activate_heading_control(True)
    rospy.loginfo("Start of poles mission...")
    poles = poles_mission.PoleSlalomMission(rc=rc,**config)
    poles.run()
    poles.cleanup()
    print("[INFO] POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)

"""OCTAGON MISSION"""
navigate_with_heading("O1")
try:
   rc.activate_heading_control(False)
   octagon = octagon_approach_mission.OctagonApproachMission(target=None, rc=rc, **config)
   time.sleep(2)
   octagon.run()
   octagon.cleanup()
   rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
   rospy.logerr("ERROR DOING OCTAGON MISSION")
   rospy.logerr(e)

"""BIN MISSION"""
navigate_with_heading("B1")
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
#    binDrop = bins_drop_mission.BinsDropMission(rc=rc, **config)
#    binDrop.run()
#    binDrop.cleanup()
    rospy.loginfo("BIN drop MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING BIN MISSION")
    rospy.logerr(e)


"""TORPEDO MISSION"""
navigate_with_heading("T1")
try:
    rc.activate_heading_control(False)
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


"""MODEMS + ROLL"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)

rospy.loginfo("Returning home")
rc.waypointNav(0,5)
rc.waypointNav(0,0)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()