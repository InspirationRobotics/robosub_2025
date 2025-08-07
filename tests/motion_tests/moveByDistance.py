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

# move forward by distance
Fdistance = 1.0
rospy.loginfo(f"Start moving forward {Fdistance} m")
rc.go_forward_distance(Fdistance)
rospy.loginfo(f"Moved {Fdistance} m")

# move lateral by distance
Ldistance = 1.0
rospy.loginfo(f"Start moving lateral {Ldistance} m")
rc.go_lateral_distance(Ldistance)
rospy.loginfo(f"Moved {Ldistance} m")

# Exit
rospy.loginfo("Reached the end")
disarm.disarm()
rc.exit()
