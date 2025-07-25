import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper

rospy.loginfo("Starting modem test script...")
rospy.init_node("intersub_coms_mission", anonymous=True)
rc = RobotControl()


" Set this to true if you are the sending sub/party"
""""""""""""""""""""""""""""""""
#sending_sub = deviceHelper.variables.get("sub")  # This will be either graey or onyx
sending_sub = True  # Set to True for sending sub, False for receiving sub
""""""""""""""""""""""""""""""""
" Will change the above logic to reduce human error in case both are set to true or false"

# Send a message to test modem functionality
def send_modem_message():
    try: 
        rc.send_modem(addr=destination_addr, movement="YAW")
        rospy.loginfo("Sent message to address 020 with movement YAW")
        time.sleep(1)
    except Exception as e:
        rospy.logerr(f"Failed to send modem message: {e}")

# On Receiving a message 
def rec_callback(msg):
    rospy.loginfo(f"Received message: {msg.data}")
    if msg.data == "YAW":
        rospy.loginfo("Attempting to YAW")
        rc.go_to_heading(90)
        rc.go_to_heading(180)
        rc.go_to_heading(270)
        rc.go_to_heading(360)




current_sub = deviceHelper.variables.get("sub")  # This will be either graey or onyx
if current_sub == "Graey":
    destination_addr = "020"
elif current_sub == "Onyx":
    destination_addr = "010"
else:
    rospy.logerr("Unknown sub type, cannot determine destination address.")
    destination_addr = None

rc.set_control_mode(f"{current_sub} depth_hold")
rc.go_to_depth(0.5)
print(f"{current_sub} Reach depth")
rc.activate_heading_control(False)

try: 
    if sending_sub:
        rospy.loginfo("This is the sending sub, preparing to send messages.")
        # Send a message to test modem functionality
        send_modem_message()

except Exception as e:
    rospy.logerr(f"Error in sending message: {e}")


sub = rospy.Subscriber("/auv/devices/modem/received", String, rec_callback )

rospy.loginfo("Spinning")
rospy.spin()

