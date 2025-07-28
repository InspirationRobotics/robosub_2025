import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm

rospy.init_node("Full_position_test_for_data_collection", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
arm.arm()
rospy.loginfo("Diving down")
rc.set_absolute_z(0.5)
time.sleep(5)
rospy.loginfo("Reached the depth of 0.5m")

rc.movement(forward=2)
time.sleep(2)
rc.movement()
rospy.loginfo("Reached the end of the first movement")

rc.movement(forward=-2)
time.sleep(2)
rc.movement()
rospy.loginfo("Reached the end of the second movement")

rc.movement(lateral=2)
time.sleep(2)
rc.movement()
rospy.loginfo("Reached the end of the second movement")

rc.movement(lateral=-2)
time.sleep(2)
rc.movement()
rospy.loginfo("Reached the end of the second movement")

time.sleep(5)
rospy.loginfo("Reached the end")

disarm.disarm()