"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
from auv.mission import poles_mission, intersub_com_mission, poles_mission_preset
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_absolute_z(0.5)
rospy.loginfo("Robot armed and set to depth 0.7m")
rospy.loginfo("Waiting for 7 seconds before proceeding")
time.sleep(7)
gate_heading = 0 # CALIBRATE EACH TIME 
config = deviceHelper.variables
eventflags = [False,False,False,False,False]

"""COINT TOSS + GATE MISSION"""
try:
   rc.activate_heading_control(activate=True)
   rc.set_absolute_yaw(gate_heading) # Set deire heaidng

   # wait until robot reach heaidng within 2 degrees error
   # while abs(gate_heading-rc.orientation['yaw']) > 2: # 2 degrees tolerance
   #    time.sleep(1)
   rc.go_to_heading(0)
   
   rospy.loginfo("Robot heading set to gate heading")
   eventflags[0] = True

   rospy.loginfo("Moving forward for 3 seconds")
   rc.movement(forward=2)
   time.sleep(5)
   rc.movement() # stop moving forward
   print("[INFO] GATE MISSION COMPLETE")
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
    
# """WP TO POLES"""
# try:
#     # TODO consider the gate has an angle with the poles
#     rospy.loginfo("Moving right for 6 seconds")
#     rc.movement(lateral=2)
#     time.sleep(6)
#     rc.movement()
# except Exception as e:
#     rospy.logerr("ERROR OCCUR IN WP TO POLES")
#     rospy.logerr(e)

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

"""POLES MISSION"""
try: 
   # Run the poles mission
   rospy.loginfo("Start of poles mission...")
   poles = poles_mission.PoleSlalomMission(rc=rc,**config)
   poles.run()
   poles.cleanup()
   print("[INFO] POLES MISSION COMPLETE")
   eventflags[2] = True
except Exception as e:
   rospy.logerr("ERROR OCCUR IN POLES MISSION")
   rospy.logerr(e)
   eventflags[2] = True


"""LATERAL WP"""
try:
   rospy.loginfo("Moving lateral for 2 seconds")
   rc.movement(lateral=-2)
   time.sleep(2)
   rc.movement()
except Exception as e:
   rospy.logerr("ERROR OCCUR IN LATERAL WP")
   rospy.logerr(e)
    
""" BACK TO GATE WP"""
try:
   rospy.loginfo("Moving backward for 3 seconds")
   rc.movement(forward=-2)
   time.sleep(3)
   rc.movement()
except Exception as e:
   rospy.logerr("ERROR OCCUR IN BACK TO GATE WP")
   rospy.logerr(e)
    
"""MODEMS + ROLL"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
    eventflags[3] = True
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)
    eventflags[3] = True
    
#"""ALIGNING WITH GATE WP"""
#try:
#    rospy.loginfo("Moving right for 5 seconds")
#    rc.movement(lateral=2)
#    time.sleep(5)
#except Exception as e:
#    rospy.logerr("ERROR OCCUR IN ALIGNING WITH GATE WP")
#    rospy.logerr(e)
    
"""GOING BACK THROUGH GATE"""
#try:
#    rospy.loginfo("Moving backward for 5 seconds")
#    rc.movement(forward=-2)
#    time.sleep(5)
#except Exception as e:
#    rospy.logerr("ERROR OCCUR IN GOING BACK THROUGH GATE WP")
#    rospy.logerr(e)

disarm.disarm()
rc.exit()


