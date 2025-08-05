import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper

class intersubComMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)
        self.sub = deviceHelper.variables.get('sub')

        # flag for ending
        self.end = False

    # Send a message to the other sub through modem
    def send_modem_message(self, dest_addr, move):
        """
        Send message to the other sub
        Args:
            - dest_addr (String) : destination address of the other sub ex. '010'
            - move (String) : movement you want to perfor ex. ROLL, YAW
        """
        try: 
            self.rc.send_modem(addr=dest_addr, movement=move)
            rospy.loginfo("Sent message to address 020 with movement YAW")
            time.sleep(1)
        except Exception as e:
            rospy.logerr(f"Failed to send modem message: {e}")

    # On Receiving a message 
    def rec_callback(self,msg):
        if not self.end and self.sub=="graey":  # callback function when receiving a message from onyx
            rospy.loginfo(f"Received message: {msg.data}")
            if msg.data == "YAW":
                rospy.loginfo("Attempting to YAW")
                self.rc.go_to_heading(90)
                self.rc.go_to_heading(180)
                self.rc.go_to_heading(270)
                self.rc.go_to_heading(360)
            if msg.data == "ROLL":
                rospy.loginfo("Doing roll maneuver")
                self.rc.set_absolute_z(0.7)
                time.sleep(10)
                self.rc.set_flight_mode("ACRO")
                self.rc.set_control_mode("direct")
                self.rc.movement(roll=5)
                time.sleep(4)  # fine tune this value for Graey
                self.rc.movement()
                self.rc.set_control_mode("depth_hold")
            self.end = True

    def run(self):
        current_sub = self.sub  # This will be either graey or onyx
        # go to desire position
        self.rc.set_control_mode("depth_hold")
        self.rc.set_absolute_z(0.5)
        self.rc.activate_heading_control(False)
        time.sleep(3)
        if current_sub == "graey":

            destination_addr = "020"
            time_counter = 0
            while not self.end:
                if time_counter>= 5: # set time out to 5s for testing purpose
                    rospy.loginfo("Time out, mission terminated, roll without onyx")
                    fake_msg = String()
                    fake_msg.data = "ROLL"
                    rospy.loginfo("Sending fake roll msg to callback func")
                    self.rec_callback(fake_msg)
                    self.end=True
                time.sleep(1)
                time_counter += 1
                rospy.loginfo(f"Waited for {time_counter} seconds")

            # stabilizing the sub after rolling / mission terminated
            rospy.loginfo("Stabilizing the sub...")
            self.rc.set_absolute_z(0.5)
            time.sleep(10)
            self.rc.set_flight_mode("STABILIZE")
            rospy.loginfo("Back to Stabilize mode...")


        elif current_sub == "onyx":
            destination_addr = "010"
            rospy.loginfo("Sending message to Graey")
            # once onyx has sent the message 5 times, it can stop sending
            for i in range(5):
                self.send_modem_message(dest_addr=destination_addr, move="ROLL")
                time.sleep(3)

        
            




if __name__=="__main__":
    """
    rostopic pub -1 /chatter std_msgs/String "data: 'hello world'"
    Please use the above example to simulate a msg received when testing
    """
    rospy.loginfo("Starting modem test script...")
    rospy.init_node("intersub_coms_mission", anonymous=True)
    rc = RobotControl()
    rc.set_control_mode("depth_hold")
    mission = intersubComMission(robotControl=rc)
    mission.run()
    rc.exit()
