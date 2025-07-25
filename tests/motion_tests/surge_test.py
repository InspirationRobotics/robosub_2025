import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm
from auv.utils import fly

rospy.init_node("Surge test for data collection", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
arm.arm()
rospy.loginfo("Diving down")
rc.go_to_depth(0.5)

fly.set_flight_mode("STABILIZE")
rospy.loginfo("Setting flight mode to STABILIZE")

rc.movement(forward=2)
rc.activate_heading_control(True) # activate heading control
time.sleep(5)
rc.movement()

rc.movement(forward=-2)
time.sleep(5)
rc.movement()

time.sleep(5)
rospy.loginfo("Reached the end")

disarm.disarm()