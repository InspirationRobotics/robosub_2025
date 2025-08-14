import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()

rc.set_control_mode("depth_hold")
rc.set_flight_mode("STABILIZE")

# Diving down
rc.go_to_depth(0.2)

# Arm
arm.arm()

rc.set_absolute_yaw(rc.orientation['yaw'])
rc.activate_heading_control(True)

time.sleep(60)




rc.exit()
disarm.disarm()
