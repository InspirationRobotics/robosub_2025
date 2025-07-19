import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
arm.arm()
rospy.loginfo("Diving down")
rc.set_absolute_z(0.5)
time.sleep(5)
rospy.loginfo("Set heading control to 0")
rc.go_to_heading(75)


rc.set_absolute_yaw(75)
rc.activate_heading_control(True) # activate heading control
rc.movement(forward=2)
time.sleep(5)
rc.movement()

rc.movement(lateral=-2)
time.sleep(2)

time.sleep(5)

rospy.loginfo("Reached the end")

disarm.disarm()
