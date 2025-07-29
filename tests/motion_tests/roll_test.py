import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
arm.arm()
rospy.loginfo("Diving down")
rc.set_absolute_z(0.8)
time.sleep(13)
rc.set_control_mode("direct")
rospy.loginfo("doing roll")
rc.movement(roll=-5)
time.sleep(1.5)
rc.movement()
# rc.movement(vertical=-4)
# time.sleep(1)
rc.movement()
rc.reset()
rc.set_control_mode("depth_hold")
rc.set_absolute_z(0.8)
time.sleep(13)
rc.set_control_mode("direct")
rospy.loginfo("doing roll the second time")
rc.movement(roll=-5)
time.sleep(1.5)
rc.movement()
# rc.movement(vertical=-4)
# time.sleep(1)
rc.movement()
rc.reset()
rc.set_control_mode("depth_hold")
rc.set_absolute_z(0.8)
time.sleep(13)



rospy.loginfo("Reached the end")

disarm.disarm()
