"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
from auv.mission import poles_mission, intersub_com_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_absolute_z(0.5)
rospy.loginfo("Robot armed and set to depth 0.5m")
rospy.loginfo("Waiting for 7 seconds before proceeding")
time.sleep(7)
target = "left"
gate_heading = 0 # calibrate beforehand
config = deviceHelper.variables


"""GATE MISSION"""
try:
    # Rotate towards the heading of the gate, move 2 meters forward
    current_heading = rc.orientation["yaw"]
    # TODO two ways of doing coin toss
    # 1. Using absolute heading, this require recalibration each run
    # 2. Using relative heading, this require hard code the angle
    rc.activate_heading_control(activate=True)
    rc.set_absolute_yaw(current_heading + 90) # this is hard coded angle
    rospy.sleep(8)
    rospy.loginfo("Robot heading set to gate heading")

    rospy.loginfo("Moving forward for 8 seconds")
    rc.movement(forward=2)
    time.sleep(8)
    rc.movement() # stop moving forward
except KeyboardInterrupt as e:
    rospy.logwarn("Skipping current mission")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN GATE MISSION")
    rospy.logerr(e)
    
# """WP TO POLES"""
# try:
#     # TODO consider the gate has an angle with the poles
#     rospy.loginfo("Moving forward for 5 seconds")
#     rc.movement(forward=2)
#     time.sleep(3)
#     rc.movement()
# except Exception as e:
#     rospy.logerr("ERROR OCCUR IN WP TO POLES")
#     rospy.logerr(e)

"""POLES MISSION"""
try: 
    # Run the poles mission
    rospy.loginfo("Start of poles mission...")
    poles = poles_mission.PoleSlalomMission(target=target, rc=rc,**config)
    poles.run()
    poles.cleanup()
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)
    
"""LATERAL WP"""
try:
    rospy.loginfo("Moving lateral for 8 seconds")
    rc.movement(lateral=2)
    time.sleep(8)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN LATERAL WP")
    rospy.logerr(e)
    
""" BACK TO GATE WP"""
try:
    rospy.loginfo("Moving backward for 12 seconds")
    rc.movement(forward=-2)
    time.sleep(12)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN BACK TO GATE WP")
    rospy.logerr(e)
    
"""MODEMS + ROLL"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)
"""ALIGNING WITH GATE WP"""
try:
    rospy.loginfo("Moving right for 5 seconds")
    rc.movement(lateral=2)
    time.sleep(5)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN ALIGNING WITH GATE WP")
    rospy.logerr(e)
    
"""GOING BACK THROUGH GATE"""
try:
    rospy.loginfo("Moving backward for 5 seconds")
    rc.movement(forward=-2)
    time.sleep(5)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN GOING BACK THROUGH GATE WP")
    rospy.logerr(e)

disarm.disarm()
rc.exit()


