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

def go_to_and_hold(degrees):
    rospy.loginfo(f"Going to {degrees}")
    rc.go_to_heading(degrees)
    rc.set_absolute_yaw(degrees)
    rc.activate_heading_control(True)
    time.sleep(5)
    rc.activate_heading_control(False)
    rospy.loginfo(f"Actual heading: {rc.get_heading()} | desired: {degrees}")



go_to_and_hold(-30)
go_to_and_hold(30)
go_to_and_hold(0)
go_to_and_hold(90)
go_to_and_hold(180)
go_to_and_hold(270)
go_to_and_hold(0)

time.sleep(5)




rc.exit()
disarm.disarm()
