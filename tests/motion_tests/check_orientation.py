import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl()

arm.arm()

rospy.loginfo("This is the start")

rospy.loginfo(f"current state: {rc.orientation}")
first_time = time.time()
while time.time() - first_time < 30:
    rospy.loginfo(f"current state: {rc.orientation}")
    time.sleep(0.5)

rospy.loginfo("Reached the end")
disarm.disarm()
