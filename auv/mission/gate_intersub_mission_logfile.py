import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper

class GateIntersubMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)
        self.sub = deviceHelper.variables.get('sub')
        self.end = False
        self.roll_requested = False  # <-- New flag

    def send_modem_message(self, dest_addr, msg):
        try: 
            self.rc.send_modem(addr=dest_addr, movement=msg)
            with open("coms.log", 'a') as file:
                file.write(f"[{time.time()}] Sent message to address {dest_addr} with message {msg}\n")
            time.sleep(1)
        except Exception as e:
            rospy.logerr(f"Failed to send modem message: {e}")

    def rec_callback(self, msg):
        if not self.end and self.sub == "graey":
            with open("coms.log", 'a') as file:
                file.write(f"[{time.time()}] Received message: {msg.data}\n")
            if msg.data == "Onyx_Gate_Finished":
                rospy.loginfo("Graey start going through the gate")
                for i in range(3):
                    self.rc.send_modem(addr="020", movement="Graey_Start")
                    time.sleep(1)
                rospy.loginfo("Finishing Gate Intersub mission")
            self.end = True
        if not self.end and self.sub == "onyx":
            with open("coms.log", 'a') as file:
                file.write(f"[{time.time()}] Received message: {msg.data}\n")
            if msg.data == "Graey_Start":
                rospy.loginfo("ACK Received, Onyx proceeding!")
                # NOTE We can add more stuff here if we want after receiving the ACK
                rospy.loginfo("Finishing Gate Intersub mission")
            self.end = True

    def run(self):
        current_sub = self.sub
        self.rc.set_control_mode("depth_hold")
        self.rc.activate_heading_control(True)
        time.sleep(3)

        if current_sub == "graey":
            time_counter = 0
            while not self.end:
                if time_counter >= 60: # 30 second timeout
                    rospy.loginfo("Time out, no message received, Graey going through the gate")
                    fake_msg = String()
                    fake_msg.data = "Onyx_Gate_Finished"
                    self.rec_callback(fake_msg)
                time.sleep(1)
                time_counter += 1

        elif current_sub == "onyx":
            destination_addr = "010"
            rospy.loginfo("Sending message to Graey")
            while not self.end:
                # Continously send message if not received ACK
                self.rc.send_modem(addr=destination_addr, movement="Onyx_Gate_Finished")
                time.sleep(1) 
            


if __name__=="__main__":
    """
    rostopic pub -1 /chatter std_msgs/String "data: 'hello world'"
    Please use the above example to simulate a msg received when testing
    """
    rospy.loginfo("Starting modem test script...")
    rospy.init_node("intersub_coms_mission", anonymous=True)
    rc = RobotControl()
    rc.set_control_mode("depth_hold")
    rc.set_absolute_z(0.5)
    time.sleep(10)
    mission = GateIntersubMission(robotControl=rc)
    mission.run()
    rc.exit()
