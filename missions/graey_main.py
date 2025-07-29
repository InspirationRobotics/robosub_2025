"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
from auv.mission import poles_mission
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
target = "right"
gate_heading = 0 # calibrate beforehand
config = deviceHelper.variables


"""GATE MISSION"""
try:
    # Rotate towards the heading of the gate, move 2 meters forward
    current_heading = rc.orientation["yaw"]
    rc.activate_heading_control(activate=True)
    rc.set_absolute_yaw(current_heading)
    rospy.sleep(5)
    rospy.loginfo("Robot heading set to gate heading")

    time.sleep(3)

    # going through the Gate
    curr_time = time.time()

    rospy.loginfo("Moving forward for 6 seconds")
    rc.movement(forward=2)
    time.sleep(6)
    rc.movement() # stop moving forward
except KeyboardInterrupt as e:
    rospy.logwarn("Skipping current mission")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN GATE MISSION")
    rospy.logerr(e)
    
# """WP TO POLES"""
# try:
#     rospy.loginfo("Moving forward for 5 seconds")
#     while time.time() - curr_time < 5:
#         rc.movement(forward=2)
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
    rospy.loginfo("Moving lateral for 5 seconds")
    while time.time() - curr_time < 5:
        rc.movement(lateral=2)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN LATERAL WP")
    rospy.logerr(e)
    
""" BACK TO GATE WP"""
try:
    rospy.loginfo("Moving backward for 10 seconds")
    while time.time() - curr_time < 10:
        rc.movement(forward=-2)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN BACK TO GATE WP")
    rospy.logerr(e)
    
"""MODEMS + ROLL"""

# Placeholder for modem communication logic

"""ALIGNING WITH GATE WP"""
try:
    rospy.loginfo("Moving right for 5 seconds")
    while time.time() - curr_time < 5:
        rc.movement(lateral=2)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN ALIGNING WITH GATE WP")
    rospy.logerr(e)
    
"""GOING BACK THROUGH GATE"""
try:
    rospy.loginfo("Moving backward for 5 seconds")
    while time.time() - curr_time < 5:
        rc.movement(forward=-2)
except Exception as e:
    rospy.logerr("ERROR OCCUR IN GOING BACK THROUGH GATE WP")
    rospy.logerr(e)

disarm.disarm()
rc.exit()

# current_heading = rc.orientation["yaw"]
# return_heading = current_heading + 180
# rc.go_to_heading(return_heading)

# while time.time() - curr_time < 10:
#     rc.movement(forward=2)

# disarm.disarm()

# print("[INFO] Mission run terminate")

