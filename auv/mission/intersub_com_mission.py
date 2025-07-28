import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper

class intersubComMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.sub = deviceHelper.variables.get('sub')
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)

    # Send a message to test modem functionality
    def send_modem_message(self, dest_addr, move):
        try: 
            self.rc.send_modem(addr=dest_addr, movement=move)
            rospy.loginfo("Sent message to address 020 with movement YAW")
            time.sleep(1)
        except Exception as e:
            rospy.logerr(f"Failed to send modem message: {e}")

    # On Receiving a message 
    def rec_callback(self,msg):
        rospy.loginfo(f"Received message: {msg.data}")
        if msg.data == "YAW":
            rospy.loginfo("Attempting to YAW")
            self.rc.go_to_heading(90)
            self.rc.go_to_heading(180)
            self.rc.go_to_heading(270)
            self.rc.go_to_heading(360)
        if msg.data == "ROLL":
            self.rc.go_to_depth(1.3)
            self.rc.set_control_mode = "MANUAL"
            self.rc.movement(roll=5)
            time.sleep(3)  # fine tune this value for Graey
            self.rc.movement()

    def start(self):
        # go to desire position
        self.rc.set_control_mode("depth_hold")
        self.rc.go_to_depth(0.5)
        rospy.loginfo(f"{current_sub} Reach depth")
        self.rc.activate_heading_control(False)

        current_sub = self.sub  # This will be either graey or onyx
        if current_sub == "graey":
            destination_addr = "020"
            for i in range(90):
                # if graey recieves the roll messages, accuate it yaw
                pass
        elif current_sub == "onyx":
            destination_addr = "010"
            rospy.loginfo("Sending message to Graey")
            self.send_modem_message(dest_addr=destination_addr, move="ROLL")
            
            # one onyx has sent the message 5 times, it can stop sending











start_time = time.time()
rospy.loginfo("waiting")
for i in range(180):
    rospy.loginfo(f"{i} second has passed")
    time.sleep(1)

if __name__=="__main__":
        
    rospy.loginfo("Starting modem test script...")
    rospy.init_node("intersub_coms_mission", anonymous=True)
    rc = RobotControl()

    mission = intersubComMission(robotControl=rc)

