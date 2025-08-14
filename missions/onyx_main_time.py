"""
Perform gate, poles, bins, torpedoes, octagon, coms 
"""

import rospy
import time
import json

from auv.mission import bin_approach_mission, bin_drop_mission, octagon_approach_mission, intersub_com_mission, poles_mission_right, torpedo_approach_mission, gate_intersub_mission, poles_mission_left
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
config = deviceHelper.variables

# Dive down to desire depth
rc.go_to_depth(1.2)

rospy.loginfo("Finish initialization & depth is 1.2 m")

"""GATE MISSION"""
try:
    # COIN FLIP
    rc.go_to_heading(0)
    rc.activate_heading_control(True)
    rc.movement(forward=2)
    time.sleep(3.5*5.56)
    rc.movement(lateral=-2)
    time.sleep(3.5*0.86)
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
    rc.activate_heading_control(True)
    rospy.loginfo("Start of poles mission...")
    poles = poles_mission_left.PoleSlalomMission(rc=rc,**config)
    poles.run()
    poles.cleanup()
    print("[INFO] POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)

"""SET HEADING TO OCTAGON"""
octagon_heading = 20
rc.activate_heading_control(False)
rc.go_to_heading(octagon_heading)
rc.activate_heading_control(True)

"""MOVE TOWARD THE OCTAGON"""
rc.movement(forward=2.0)
time.sleep(19.25)

"""OCTAGON MISSION"""
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

"""SET HEADING TO BIN"""
octagon_heading = 210
rc.activate_heading_control(False)
rc.go_to_heading(octagon_heading)
rc.activate_heading_control(True)

"""MOVE TOWARD THE BIN"""
rc.movement(forward=2.0)
time.sleep(8.75)

"""BIN MISSION"""
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
    
"""SET HEADING TO TORPEDO"""
torpedo_waypoint_heading = 110
rc.activate_heading_control(False)
rc.go_to_heading(torpedo_waypoint_heading)
rc.activate_heading_control(True)

"""MOVE TOWARD THE TORPEDO"""
rc.movement(forward=2.0)
time.sleep(8.75)

"""FACE TORPEDO"""
return_home_heading = 60
rc.activate_heading_control(False)
rc.go_to_heading(return_home_heading)
rc.activate_heading_control(True)

"""TORPEDO MISSION"""
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

"""RETURN HOME"""
return_home_heading = 180
rc.activate_heading_control(False)
rc.go_to_heading(return_home_heading)
rc.activate_heading_control(True)

"""MOVE BACK TO GATE"""
rc.movement(forward=2.0)
time.sleep(35)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()