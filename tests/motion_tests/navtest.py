import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)
print("[INFO] This is the start")
rc.waypointNav(x=0,y=5) 
time.sleep(5.0)

#1)Roll motion with depth hold test:
first_time = time.time()
while time.time() - first_time < 6:
    rc.movement(roll=5)

print("Reached the end")

rc.set_depth(0.0)
disarm.disarm()
