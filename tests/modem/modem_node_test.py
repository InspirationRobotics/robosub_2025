import time
import rospy
print("import rc")
from auv.motion.robot_control import RobotControl

rospy.loginfo("Starting modem test script...")
rospy.init_node("modem_test", anonymous=True)

rc = RobotControl()
time.sleep(2)
rospy.loginfo("Initialized robot control")

# Send message
for i in range(5):
    rc.send_modem(addr="020",movement="ROLL",ack=0,priority=1)
    rospy.loginfo(f"Sent message {i+1}")
    time.sleep(1)

time.sleep(2)

rospy.loginfo("Test ended")


