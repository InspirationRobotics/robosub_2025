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
        self.end = False
        self.roll_requested = False  # <-- New flag

    def send_modem_message(self, dest_addr, move):
        try: 
            self.rc.send_modem(addr=dest_addr, movement=move)
            rospy.loginfo(f"Sent message to address {dest_addr} with movement {move}")
            time.sleep(1)
        except Exception as e:
            rospy.logerr(f"Failed to send modem message: {e}")

    def rec_callback(self, msg):
        if not self.end and self.sub == "graey":
            rospy.loginfo(f"Received message: {msg.data}")
            if msg.data == "YAW":
                rospy.loginfo("Attempting to YAW")
                self.rc.go_to_heading(90)
                self.rc.go_to_heading(180)
                self.rc.go_to_heading(270)
                self.rc.go_to_heading(360)
            if msg.data == "ROLL":
                rospy.loginfo("Roll maneuver requested")
                self.roll_requested = True  # <-- Store request, but don't execute yet
            self.end = True

    def do_roll(self):
        """Executes the roll maneuver if requested"""
        if self.roll_requested:
            rospy.loginfo("Doing roll maneuver")
            self.rc.set_absolute_z(0.7)
            time.sleep(10)
            self.rc.set_flight_mode("ACRO")
            self.rc.set_control_mode("direct")
            self.rc.movement(roll=5)
            time.sleep(4)
            self.rc.movement()
            self.rc.set_control_mode("depth_hold")
            rospy.loginfo("Roll maneuver complete")
        else:
            rospy.logwarn("Roll was not requested, skipping roll maneuver")

    def run(self):
        """Only handles communication, not the roll maneuver"""
        current_sub = self.sub
        self.rc.set_control_mode("depth_hold")
        self.rc.activate_heading_control(True)
        time.sleep(1)

        if current_sub == "graey":
            time_counter = 0
            while not self.end:
                if time_counter >= 60:
                    rospy.loginfo("Time out, no message received, faking ROLL request")
                    fake_msg = String()
                    fake_msg.data = "ROLL"
                    self.rec_callback(fake_msg)
                time.sleep(1)
                time_counter += 1

        elif current_sub == "onyx":
            destination_addr = "010"
            rospy.loginfo("Sending message to Graey")
            for i in range(60):
                self.send_modem_message(dest_addr=destination_addr, move="ROLL")
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
    time.sleep(10)
    mission = intersubComMission(robotControl=rc)
    mission.run()
    rc.exit()
