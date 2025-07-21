# IMPORT modem class form modem_api.py
from auv.device.modems.modems_api import Modem
import time
import rospy
from standard_msgs.msg import String
# ROS Publisher to modem_send 

# Publish a string using the publish method

# Initiliaze a modem class
modem = Modem()

rospy.init_node("modem_sender", anonymous=True)
pub = rospy.Publisher("/auv/devices/modem_send", String, queue_size=10)

rate = rospy.Rate(1)  # 1 Hz

if not rospy.is_shutdown():
    # Format a message: "DEST-MSG-ACK-PRIORITY"
    msg_str = "020-ROLL-1-0"
    rospy.loginfo(f"Publishing: {msg_str}")
    pub.publish(String(data=msg_str))
    rate.sleep()





# ROSPY Init Node
