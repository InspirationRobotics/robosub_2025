import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()

rc.set_control_mode("direct")
rc.set_flight_mode("MANUAL")



rc.movement(yaw=0.4)
time.sleep(45)




rc.exit()
disarm.disarm()
