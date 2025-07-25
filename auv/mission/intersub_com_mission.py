import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper

rospy.loginfo("Starting modem test script...")
rospy.init_node("intersub_coms_mission", anonymous=True)

" Set this to true if you are the sending sub/party"
""""""""""""""""""""""""""""""""
sending_sub = deviceHelper.variables.get("sub")  # This will be either graey or onyx
""""""""""""""""""""""""""""""""
rc = RobotControl()
rc.set_control_mode("depth_hold")
rc.go_to_depth(0.5)
print(f"Reach depth")
rc.activate_heading_control(False)
# On Receiving a message 
def rec_callback(msg):
    rospy.loginfo(f"Received message: {msg.data}")
    if msg.data == "YAW":
        rospy.loginfo("Attempting to YAW")
        rc.go_to_heading(90)
        rc.go_to_heading(180)
        rc.go_to_heading(270)
        rc.go_to_heading(360)


# Send a message to test modem functionality
def send_modem_message():
    try: 
        rc.send_modem(addr="020", movement="YAW")
        rospy.loginfo("Sent message to address 020 with movement YAW")
    except Exception as e:
        rospy.logerr(f"Failed to send modem message: {e}")

sub = rospy.Subscriber("/auv/devices/modem/received", String, rec_callback )

#if sending_sub:
    #rospy.loginfo("This sub is configured to send messages.")
    #destination = "010"  # Example destination address
    #send_modem_message()

rospy.loginfo("Spining")
rospy.spin()
#try:
 #   rc.send_modem(addr="020", movement="YAW")
 #   rospy.loginfo(f"Sent message to address 020 with movement YAW")
#except Exception as e:
  #  rospy.logerr(f"Failed to initialize modem: {e}")
  #  modem = None

