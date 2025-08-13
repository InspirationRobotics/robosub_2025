import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()

rc.set_control_mode("depth_hold")
rc.set_flight_mode("STABILIZE")

# Diving down
rc.go_to_depth(0.8)

# Arm
arm.arm()

heading = rc.get_heading()
print(f"[DEBUG]: Heading is {heading}")

rc.go_to_heading(heading + 90)
print(f"[DEBUG]: Heading is {heading}")

rc.go_to_heading(heading + 90)
print(f"[DEBUG]: Heading is {heading}")

rc.go_to_heading(heading + 180)
print(f"[DEBUG]: Heading is {heading}")

rc.go_to_heading(heading - 90)
print(f"[DEBUG]: Heading is {heading}")

rc.go_to_heading(heading + 90)
print(f"[DEBUG]: Heading is {heading}")



time.sleep(2.0)

disarm.disarm()
