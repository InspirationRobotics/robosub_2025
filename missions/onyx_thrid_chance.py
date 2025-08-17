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
rc.go_to_depth(0.6)
rc.go_to_depth(1.2)

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

"""SLALOM MISSION"""

try: 
    rospy.loginfo("Start of poles mission...")
    navigate_to("S1")
    navigate_to("S2")
    navigate_to("S3")
    print("[INFO] POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)

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

"""BIN MISSION"""
try:
    navigate_to("B1")
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
    rc.go_to_depth(0.6)
    navigate_to("O1")
    rc.go_to_heading(135)
    rc.set_absolute_yaw(135)
    rc.go_to_depth(0)
    time.sleep(4)
    rc.go_to_depth(0.6)
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
   rospy.logerr("ERROR DOING OCTAGON MISSION")
   rospy.logerr(e)


"""TORPEDO MISSION"""
try:
    navigate_to("T1")
    rc.go_to_depth(1.2)
    # align with the torpedo
    rc.go_to_heading(20)
    rc.set_absolute_yaw(20)
    rc.go_forward_distance(0.5)
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