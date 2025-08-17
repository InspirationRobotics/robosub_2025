"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time

print("Sleeping for 30 seconds to unplug tether")
time.sleep(30)

from auv.utils import deviceHelper
from auv.mission import poles_mission, intersub_com_mission, poles_mission_preset, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
rc.go_to_depth(1.2)
rospy.loginfo("Robot armed and set to depth 1.2 m")
gate_heading = 0 # CALIBRATE EACH TIME 
config = deviceHelper.variables
eventflags = [False,False,False,False,False]

"""GATE INTERSUB MISSION"""
try:
    start_time = time.time()
    while time.time()-start_time<30:  # 30 s timeout
        msg = rc.get_latest_modem()
        if msg is not None and msg=="Onyx_Finished":
            rospy.loginfo("Graey started, Onyx proceeding")
            msg_to_send = "Graey_Start"
            break
        time.sleep(0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""COINT TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   rc.set_absolute_yaw(gate_heading)
   rospy.loginfo("Robot heading set to gate heading")
   
   gate_forward_distance = 5.57 # m
   rospy.loginfo(f"Start moving forward {gate_forward_distance} m")
   rc.go_forward_distance(gate_forward_distance)
   rospy.loginfo(f"Moved {gate_forward_distance} m")
   
   print("[INFO] GATE MISSION COMPLETE")

except KeyboardInterrupt as e:
   rospy.logwarn("Skipping current mission")
   eventflags[0] = True
   eventflags[1] = True
except Exception as e:
   rospy.logerr("ERROR OCCUR IN GATE MISSION")
   rospy.logerr(e)
   eventflags[0] = True
   eventflags[1] = True

"""KEEP GOING FORWARD"""
rc.go_forward_distance(3)  # 3 meter away from the gate

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

"""RETURN HOME"""
rc.go_forward_distance(-5)   # GO back

"""ROLL MANEUVER"""
try:
    intersubMission.do_roll()  # <-- Execute roll here
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING ROLL MANEUVER")
    rospy.logerr(e)

"""GO BACK MORE"""
rc.go_forward_distance(-3)

rospy.loginfo("GRAEY FINISHED")
disarm.disarm()
rc.exit()