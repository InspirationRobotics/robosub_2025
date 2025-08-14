"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
from auv.mission import poles_mission, intersub_com_mission, poles_mission_preset, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.go_to_depth(1.2)
rospy.loginfo("Robot armed and set to depth 1.2 m")
gate_heading = 0 # CALIBRATE EACH TIME 
return_heading = 180
config = deviceHelper.variables
eventflags = [False,False,False,False,False]


"""GATE INTERSUB MISSION"""
try:
    gateIntersub = gate_intersub_mission.GateIntersubMission(robotControl=rc)
    gateIntersub.run()  # <-- Comms only
    rospy.loginfo("FINISHED GATE INTERSUB MISSION")
except Exception as e:
    rospy.logerr("ERROR DURING GATE INTERSUB MISSION")
    rospy.logerr(e)

"""COINT TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   rc.set_absolute_yaw(gate_heading)
   rospy.loginfo("Robot heading set to gate heading")
   
   # set event flag for coin toss mission to True
   eventflags[0] = True
   
   gate_forward_distance = 10 # m
   rospy.loginfo(f"Start moving forward {gate_forward_distance} m")
   rc.go_forward_distance(gate_forward_distance)
   rospy.loginfo(f"Moved {gate_forward_distance} m")
   
   print("[INFO] GATE MISSION COMPLETE")
   # set event flag for gate mission to True
   eventflags[1] = True
except KeyboardInterrupt as e:
   rospy.logwarn("Skipping current mission")
   eventflags[0] = True
   eventflags[1] = True
except Exception as e:
   rospy.logerr("ERROR OCCUR IN GATE MISSION")
   rospy.logerr(e)
   eventflags[0] = True
   eventflags[1] = True

# """POLES MISSION PRESET MANEUVER"""
# try: 
#    # Run the poles mission
#    rospy.loginfo("Start of poles mission...")
#    poles = poles_mission_preset.PoleSlalomMission(rc=rc,**config)
#    poles.run()
#    poles.cleanup()
#    print("[INFO] POLES MISSION COMPLETE")
#    eventflags[2] = True
# except Exception as e:
#    rospy.logerr("ERROR OCCUR IN POLES MISSION")
#    rospy.logerr(e)
#    eventflags[2] = True

# """POLES MISSION"""
# try: 
#    # Run the poles mission
#    rospy.loginfo("Start of poles mission...")
#    poles = poles_mission.PoleSlalomMission(rc=rc,**config)
#    poles.run()
#    poles.cleanup()
#    print("[INFO] POLES MISSION COMPLETE")
#    eventflags[2] = True
# except Exception as e:
#    rospy.logerr("ERROR OCCUR IN POLES MISSION")
#    rospy.logerr(e)
#    eventflags[2] = True

# """TURNING 180 DEGREES"""
# try:
#    rc.activate_heading_control(activate=False)
#    rc.go_to_heading(return_heading)
#    rospy.loginfo("Robot heading set to return heading")
# except Exception as e:
#    rospy.logerr("ERROR OCCUR IN TURNING 180 DEGREES")
#    rospy.logerr(e)
   
"""LATERAL TO THE LEFT FOR 0.75 M"""
try:
   gate_backward_distance = -0.75 # m
   rospy.loginfo(f"Start moving forward {gate_backward_distance} m")
   rc.go_forward_distance(gate_backward_distance)
   rospy.loginfo(f"Moved {gate_backward_distance} m")
except Exception as e:
    rospy.logerr("ERROR DURING LATERAL MOTION")
    rospy.logerr(e)
    
"""GOING BACK FOR 7 M"""
try:
   gate_backward_distance = -7 # m
   rospy.loginfo(f"Start moving forward {gate_backward_distance} m")
   rc.go_forward_distance(gate_backward_distance)
   rospy.loginfo(f"Moved {gate_backward_distance} m")
except Exception as e:
    rospy.logerr("ERROR DURING LATERAL MOTION")
    rospy.logerr(e)

"""MODEMS"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()  # <-- Comms only
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
    eventflags[3] = True
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)
    eventflags[3] = True  

# """FORWARD AFTER COMMUNICATION"""
# try:
#    forward_after_comms_distance = 2.7432  # 9 ft
#    rospy.loginfo(f"Start moving forward {forward_after_comms_distance} m")
#    rc.go_forward_distance(forward_after_comms_distance)
#    rospy.loginfo(f"Moved {forward_after_comms_distance} m")
# except Exception as e:
#    rospy.logerr("ERROR OCCUR IN FORWARD AFTER COMMUNICATION")
#    rospy.logerr(e)

"""ROLL MANEUVER"""
try:
    intersubMission.do_roll()  # <-- Execute roll here
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING ROLL MANEUVER")
    rospy.logerr(e)
    
"""FORWARD AFTER ROLL"""
try:
   forward_after_roll_distance = 3 # 5 ft
   rospy.loginfo(f"Start moving forward {forward_after_roll_distance} m")
   rc.go_forward_distance(-forward_after_roll_distance)
   rospy.loginfo(f"Moved {forward_after_roll_distance} m")

except Exception as e:
   rospy.logerr("ERROR OCCUR IN FORWARD AFTER ROLL")
   rospy.logerr(e)

disarm.disarm()
rc.exit()


