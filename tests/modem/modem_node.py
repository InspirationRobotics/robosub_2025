import serial
import time
import threading
from auv.utils import deviceHelper
from auv.motion.robot_control import RobotControl
import rospy
from std_msgs.msg import String

class modem:
    def __init__(self):
        """
        - Talks to the modem over serial.
        - Publishes every raw incoming line on /auv/devices/modem/receive
        - Subscribes to /auv/devices/modem/send to send messages to the modem.
        - Parse any unicast data packets (lines starting with #U) and turns 
        payloads like "ROLL" into direct calls to RobotControl.roll().
        """
        # 1)Configure the serial port and baudrate
        self.port   = deviceHelper.dataFromConfig("modem") # DONE use deviceHelper to get port
        self.sub_id = 111 if deviceHelper.onyx else 222  # Use deviceHelper to get what sub is this
        self.ser    = serial.Serial(port=self.port,
                           baudrate=9600,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS)
        
        # tell modem "I am node xxx"
        addr_cmd = f"$A{self.sub_id:03d}"
        rospy.loginfo(f"[MODEM] Setting node address → {addr_cmd}")
        self.ser.write(addr_cmd.encode())                                                   # convert to bytes before sending
               
        # 2)Ros Setup
        rospy.init_node('modem_Node', anonymous=True)                                       # register modem_node w/ ROS & if there is another node with the same name, it will be renamed
        self.pub_rx = rospy.Publisher("/auv/devices/modem/receive", String, queue_size=10)  # publish a object to ROS / queue_size is the max number of messages to buffer before dropping old ones
        self.sub_tx = rospy.Subscriber("/auv/devices/modem/send", String, self.on_send)     # subscribe to a topic, when some node publishes to this topic, it will call the on_send function with the message                              

        #Link to RobotControl
        self.rc = RobotControl()

        # Start background I/O thread
        t = threading.Thread(target=self.io_loop, daemon=True)                              # daemon=True means it will exit when the main thread exits
        t.start()
        rospy.spin()                                                                        # keep the node running until it is shut down


    def on_send(self, msg: String):
        """ Called when some node publishes 'DEST|PAYLOAD' ex "111|ROLL" to /auv/devices/modem/send """
        try:
            dest_str, payload = msg.data.split("|",1)   # split the message into destination and payload        
            dest = int(dest_str)                            # convert destination to integer  
        except ValueError:
            rospy.logwarn("Bad format on /modem/send. Expected 'DEST|MSG'")
            return
        # 3) Send the payload to the modem
        nn  = f"{len(payload):02d}"         # number of characters in the payload, zero-padded to 2 digits
        cmd = f"$M{dest:03d}{nn}{payload}"  # command to send to the modem "$M<DEST><LEN><DATA>"
        rospy.loginfo(f"[MODEM →] {cmd}")
        self.ser.write(cmd.encode())

    def io_loop(self):
        """Continuously send queued messages and read incoming lines."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                rate.sleep()
                continue

            rospy.loginfo(f"[MODEM ←] {line}")
            # 1) Publish raw modem line
            self.pub_rx.publish(line)

            # 2) If it’s a data packet carrying a command, react:
            #    e.g. "#Uxxx04ROLL"  → payload="ROLL"
            if line.startswith("#U"):
                nn = int(line[2:4])
                data = line[4:4+nn]
                if data == "ROLL":
                    rospy.loginfo("ModemNode: triggering roll()")
                    self.rc.roll(power=3, set_time=5)
                # …handle more commands…

            rate.sleep()

if __name__ == "__main__":
    ModemNode()

            

