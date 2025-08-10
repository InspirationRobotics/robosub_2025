import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
rc.set_flight_mode("STABILIZE")

# Diving down
rc.go_to_depth(0.5)

# set to 0
rc.go_to_heading(0)

# maintain current heading
current_heading = rc.orientation['yaw']
rc.set_absolute_yaw(current_heading)
rc.activate_heading_control(True)
arm.arm()

rospy.loginfo("Reached depth")

# move forward by distance
Fdistance = 3.9624 #1.9812 
rospy.loginfo(f"Start moving forward {Fdistance} m")
rc.go_forward_distance(Fdistance)
rospy.loginfo(f"Moved {Fdistance} m")

time.sleep(15)

# # move lateral by distance
# Ldistance = 1.0
# rospy.loginfo(f"Start moving lateral {Ldistance} m")
# rc.go_lateral_distance(Ldistance)
# rospy.loginfo(f"Moved {Ldistance} m")

# Exit
rospy.loginfo("Reached the end")
disarm.disarm()
rc.exit()
