import rospy
import time

from auv.motion import robot_control
from auv.utils import arm, disarm

rospy.init_node("Depth_test", anonymous=True)
rc = robot_control.RobotControl()

arm.arm()

rc.set_depth(0.8)
print("[INFO] Setting depth to 0.4 meters")

time.sleep(20.0)

rc.set_relative_depth(-0.2)
print("[INFO] Changing depth by +0.2 meters")

time.sleep(5)
rc.set_relative_depth(0.8)
print("[INFO] Changing depth by -0.8 meters")
time.sleep(5)

disarm.disarm()
