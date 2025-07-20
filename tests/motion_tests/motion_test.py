import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode('direct')


arm.arm()
time.sleep(3.0)
print("[INFO]This is the start")

rc.movement(forward=3)
time.sleep(5)

rc.movement(forward=-3)
time.sleep(5)


print("Reached the end")

disarm.disarm()
