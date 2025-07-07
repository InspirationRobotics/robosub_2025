import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)
print("[INFO}This is the start")
rc.set_depth(0.9) 
# rc.set_mode("MANUAL")
#first_time = time.time()
time.sleep(5.0)

#1)Roll motion with depth hold test:
first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(lateral=5)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(lateral=-5)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(forward=3)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(forward=-3)

first_time = time.time()
while time.time() - first_time < 6:
    rc.movement(roll=5)


#2)Yaw control test:
## Wait until vectornav yaw is available
#timeout = time.time() + 10  # Wait up to 10 second
#rc.set_heading(270, "vectornav_imu")

#print("[INFO]rc.setHeading function executed")



#3)Prequal tests:
#first_time = time.time()
#while time.time() - first_time < 26:
 #  rc.movement(forward = 2)

#first_time = time.time()
#while time.time() - first_time < 4.5:
 #   rc.movement(lateral = -2)

#first_time = time.time()
#while time.time() - first_time < 6:
   # rc.movement(forward =-2)

#first_time = time.time()
#while time.time() - first_time < 5:
 #   rc.movement(lateral = 2)

#first_time = time.time()
#while time.time() - first_time < 16:
 #  rc.movement(forward = -2)




#first_time = time.time()
#while time.time() - first_time < 3:
    #rc.movement(forward = 2)

#first_time = time.time()
#while time.time() - first_time < 3:
    #rc.movement(forward = -2)

#rc.set_relative_depth(0.1)

#time.sleep(5)

#rc.set_relative_depth(-0.1)

#time.sleep(5)

# rc.button_press(256)


print("Reached the end")

rc.set_depth(0.0)
disarm.disarm()
