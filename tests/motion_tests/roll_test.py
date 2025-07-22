import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("pid")
arm.arm()
rospy.loginfo("Diving down")
rc.set_absolute_z(0.5)
time.sleep(5)

rc.set_absolute_roll(90)


rospy.loginfo("Reached the end")

disarm.disarm()
