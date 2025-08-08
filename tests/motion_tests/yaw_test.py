import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
rc.set_flight_mode("STABILIZE")
arm.arm()
# Diving down
rc.set_absolute_z(0.5)
while abs(rc.position['z'] - 0.5)>0.1:
    time.sleep(1)

rospy.loginfo("Reached depth")

rc.movement(yaw=0.6)
try:
    for i in range(60*5):
        time.sleep(1)
except KeyboardInterrupt:
    disarm.disarm()
    rc.exit()

# Exit
rospy.loginfo("Reached the end")
disarm.disarm()
rc.exit()
