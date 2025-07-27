"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time

from auv.mission import poles_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Graey", anonymous = True)

rc = robot_control.RobotControl()

target = "right"

gate_heading = 0 # calibrate beforehand

# time.sleep(60)

rc.set_control_mode('depth_hold')
arm.arm()
rc.set_absolute_z(0.5)
print("[INFO] Robot armed and set to depth 0.5m")

time.sleep(5)
print("[INFO] Waiting for 5 seconds before proceeding")

# Rotate towards the heading of the gate, move 2 meters forward
rc.go_to_heading(gate_heading)
rc.set_absolute_yaw(gate_heading)
rc.activate_heading_control(activate=True)
print("[INFO] Robot heading set to gate heading")

curr_time = time.time()

while time.time() - curr_time < 10:
    rc.movement(forward=2)
    print("[INFO] Moving forward for 10 seconds")
    
# Run the poles mission
poles = poles_mission.PoleSlalomMission(target=target, rc=rc)
poles.run()
poles.cleanup()

# Returning back through the gate

while time.time() - curr_time < 10:
    rc.movement(lateral=2)

current_heading = rc.orientation["yaw"]
return_heading = current_heading + 180
rc.go_to_heading(return_heading)

while time.time() - curr_time < 10:
    rc.movement(forward=2)

disarm.disarm()

print("[INFO] Mission run terminate")
