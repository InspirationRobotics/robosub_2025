"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json
import threading
# print("20s before onyx initialize")
# time.sleep(22)

from auv.mission import poles_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

# def navigate_to(name):
#     Waypoint = waypoints[name]
#     rc.waypointNav(Waypoint["position"][0],Waypoint["position"][1])
#     rospy.loginfo(f"Reached {name} waypoint")

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
rc.activate_heading_control(False)
config = deviceHelper.variables

msg_to_send = None
modem_loop_flat = True
def modem_loop():
    while modem_loop_flat:
        if msg_to_send is not None:
            rc.send_modem(addr="020",movement=msg_to_send)
            rc.flash_led()
        time.sleep(0.7)
        
# # Load the JSON file
# with open("./missions/waypoints_delta.json", "r") as file:
#     waypoints = json.load(file)

# Dive down to desire depth
rc.set_absolute_yaw(0)
# rc.go_to_depth(0.4)
rc.go_to_depth(0.8)

modem_thread = threading.Thread(target=modem_loop)
modem_thread.start()
rospy.loginfo("Finish initialization")

# move forward
msg_to_send = None
rc.go_forward_distance(2)

"""GATE INTERSUB MISSION"""
try:
    start_time = time.time()
    while time.time()-start_time<15:  # 30 s timeout
        msg = rc.get_latest_modem()
        if msg is not None and msg=="Onyx_Finished":
            rospy.loginfo("Graey started, Onyx proceeding")
            msg_to_send = "Graey_Start"
            break
        time.sleep(0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)
    
msg_to_send = "Graey_Start"

rc.go_lateral_distance(2)

msg_to_send = None
"""MODEMS + ROLL"""
try:
    start_time = time.time()
    while time.time()-start_time<30:  # 15 s timeout
        msg = rc.get_latest_modem()
        if msg is not None and msg=="Onyx_Poles_Finished":
            msg_to_send = "Graey_Return_ROLL"
            rospy.loginfo("Graey return home and roll")
            break
        time.sleep(0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

msg_to_send = "Graey_Return_ROLL"
rc.go_forward_distance(-2)
msg_to_send = None

print("[INFO] Mission run terminate")
disarm.disarm()
modem_loop_flat = False
rc.exit()