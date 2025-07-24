"""
Simplified modem for intersub communication without acknowledgment handling.
"""

import time
import RPi.GPIO as GPIO
import serial
import threading
import rospy
from auv.utils import deviceHelper
from std_msgs.msg import String


class Modem:
    """Simplified modem communication handler without acknowledgments"""
    def __init__(self, auto_start=True):
        self.__port = deviceHelper.dataFromConfig("modem")
        self.ser = serial.Serial(
            port=self.__port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        # Message parsing handlers
        self.parse_msg = {
            "#B": self.parse_broadcast,
            "#U": self.parse_unicast,
        }
        
        # Message queue
        self.in_transit = []  # [(message, destination)]
        self.queue_lock = threading.Lock()

        # ROS interface
        rospy.init_node('modem_node')
        self.pub = rospy.Publisher('/auv/devices/modem/received', String, queue_size=10)
        self.sub = rospy.Subscriber("/auv/devices/modem/send", String, self.send_callback)

        # Thread management
        self.receive_active = True
        self.sending_active = True
        self.recv_thread = threading.Thread(target=self.receive_loop)
        self.send_thread = threading.Thread(target=self.send_loop)

        if auto_start:
            self.start()

    # ROS Interface ############################################################
    def publish_to_ros(self, msg):
        """Publish message to ROS topic"""
        ros_msg = String(data=msg)
        self.pub.publish(ros_msg)
        rospy.loginfo(f"Published to ROS: {msg}")

    def send_callback(self, msg):
        """
        Handle messages from ROS send topic
        Format: "DEST_ADDR-MESSAGE"
        Example: "020-ROLL"
        """
        try:
            parts = msg.data.split('-', 1)  # Split into destination and message
            if len(parts) < 2:
                raise ValueError("Invalid message format")
                
            dest_addr = int(parts[0])
            message = parts[1]
            
            with self.queue_lock:
                self.in_transit.append((message, dest_addr))
                
        except Exception as e:
            rospy.logerr(f"Message processing failed: {str(e)}")

    # Core Modem Operations ####################################################
    def transmit_packet(self, data, dest_addr=None):
        """Transmit raw data packet through modem"""
        prefix = "U" if dest_addr is not None else "B"
        dest_addr = str(dest_addr).zfill(3) if dest_addr else ""
        
        # Format packet
        encoded = data.encode("utf-8")
        length = str(len(encoded)).zfill(2)
        packet = f"{prefix}{dest_addr}{length}{data}"
        
        self.ser.write(f"${packet}".encode())
        time.sleep(0.1 + (len(packet) * 0.0125))  # Transmission delay
        rospy.loginfo(f"Sent packet: ${packet}")

    # Message Processing ######################################################
    def handle_received_message(self, message):
        """
        Central handler for received messages:
        1. Flash receive LED
        2. Publish to ROS
        3. Log received message
        """
        self.publish_to_ros(message)
        
        # Log received message
        log_entry = f"[{time.time()}][RECV] {message}"
        with open("underwater_coms_recv.log", "a+") as f:
            f.write(log_entry + "\n")

    # Parsing Methods #########################################################
    def parse_broadcast(self, packet):
        """Parse broadcast message: #B<SRC><LEN><DATA>"""
        length = int(packet[2:4])
        message = packet[4:4+length]
        return message

    def parse_unicast(self, packet):
        """Parse unicast message: #U<LEN><DATA>"""
        length = int(packet[2:4])
        message = packet[4:4+length]
        return message

    # Thread Loops ############################################################
    def receive_loop(self):
        """Continuous receive message processing loop"""
        while self.receive_active and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    rospy.loginfo("Receiving data from modem...")
                    raw_data = self.ser.readline().decode("utf-8").strip()
                    print(f"Raw Data: {raw_data}")
                    if raw_data:
                        self.process_raw_packet(raw_data)
            except Exception as e:
                rospy.logerr(f"Receive error: {str(e)}")
            time.sleep(0.01)

    def process_raw_packet(self, packet):
        """Process raw modem packet through parsing pipeline"""
        prefix = packet[:2]
        if prefix in self.parse_msg:
            try:
                message = self.parse_msg[prefix](packet)
                self.handle_received_message(message)
            except Exception as e:
                rospy.logerr(f"Packet processing failed: {str(e)}")

    def send_loop(self):
        """Message transmission management loop - simplified"""
        while self.sending_active and not rospy.is_shutdown():
            # Get messages to send
            to_send = []
            with self.queue_lock:
                to_send = self.in_transit[:]
                self.in_transit = []
            
            # Send all messages
            for message, dest in to_send:
                try:
                    self.transmit_packet(message, dest)
                    self.log_sent_message(dest, message)
                except Exception as e:
                    rospy.logerr(f"Transmission failed: {str(e)}")
            
            time.sleep(0.1)

    # Logging #################################################################
    def log_sent_message(self, dest_addr, message):
        """Log sent messages to file"""
        log_entry = f"[{time.time()}][SEND][dst:{dest_addr}] {message}"
        with open("underwater_coms_send.log", "a+") as f:
            f.write(log_entry + "\n")

    # Lifecycle Management ####################################################
    def start(self):
        """Start modem threads"""
        self.recv_thread.start()
        self.send_thread.start()
        rospy.loginfo("Modem started")
        rospy.spin()

    def stop(self):
        """Graceful shutdown"""
        self.receive_active = False
        self.sending_active = False
        self.recv_thread.join()
        self.send_thread.join()
        self.ser.close()
        rospy.loginfo("Modem stopped")

# Main Execution ##############################################################
if __name__ == "__main__":
    modem = Modem()
    try:
        modem.start()
    except KeyboardInterrupt:
        modem.stop()
