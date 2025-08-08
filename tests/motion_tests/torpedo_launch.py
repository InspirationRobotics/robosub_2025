import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
rc.set_flight_mode("STABILIZE")
current_heading = rc.orientation['yaw']
rc.set_absolute_yaw(current_heading)
rc.activate_heading_control(True)

arm.arm()

# Diving down
rc.set_absolute_z(0.8)
while abs(rc.position['z'] - 0.8)>0.1:
    time.sleep(1)

rospy.loginfo("Reached depth")

time.sleep(2)

rc.move_servo("/auv/devices/torpedo")
time.sleep(2)
rc.move_servo("/auv/devices/torpedo")
time.sleep(2)
rc.move_servo("/auv/devices/torpedo")

# Exit
rospy.loginfo("Reached the end")
disarm.disarm()
rc.exit()
