import time
import rospy
from auv.motion.robot_control import RobotControl

rospy.loginfo("Starting modem test script...")
rospy.init_node("modem_test", anonymous=True)

rc = RobotControl()
time.sleep(2)
rospy.loginfo("Initialized robot control")

# Send message
for i in range(60):
    rc.send_modem(addr="010",movement="Graey_Start")
    rospy.loginfo(f"Sent message ")
    time.sleep(1)

#time.sleep(2)

rospy.loginfo("Test ended")

