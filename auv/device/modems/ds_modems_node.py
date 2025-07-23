"""
Creates modem (intersub communication) functionality with enhanced acknowledgment handling.
"""

import time
import RPi.GPIO as GPIO
import serial
import threading
import rospy
from auv.utils import deviceHelper
from auv.utils.deviceHelper import dataFromConfig, variables
from std_msgs.msg import String

class LED:
    """LED status indicator for message transmission/reception"""
    def __init__(self):
        try:
            import RPi.GPIO as GPIO
            self.enabled = True
        except ImportError:
            print("RPi.GPIO not found, disabling LED")
            self.enabled = False

        self.t_pin = 31  # Transmit pin
        self.r_pin = 32  # Receive pin

    def on_send_msg(self):
        """Visual indicator for message transmission"""
        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

    def on_recv_msg(self):
        """Visual indicator for message reception"""
        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.r_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

    def clean(self):
        """Clean up GPIO pins"""
        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.LOW)
        GPIO.output(self.r_pin, GPIO.LOW)
        GPIO.cleanup()

class Modem:
    """Modem communication handler with acknowledgment management"""
    ACK_TIMEOUT = 30.0  # Seconds before message times out
    ACK_RETRY_INTERVAL = 1.0  # Seconds between retries
    
    def __init__(self, auto_start=True):
        self.led = LED()
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
            "#R": self.parse_range,
            "#T": self.parse_timeout,
        }
        
        # Message handlers
        self.recv_callbacks = [self.handle_received_message]
        self.send_callbacks = [self.log_sent_message]
        
        self.data_buffer = ""
        self.next_ack = 1
        self.in_transit = []  # [msg, send_time, last_retry, ack, dest, priority]
        self.received_acks = set()
        
        # Modem parameters
        self.modemAddr = deviceHelper.variables.get("modem_address")
        self.voltage = None

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
        Format: "DEST_ADDR-MESSAGE-ACK_FLAG-PRIORITY"
        Example: "020-ROLL-1-0"
        """
        try:
            parts = msg.data.split('-')
            if len(parts) != 4:
                raise ValueError("Invalid message format")
                
            dest_addr = int(parts[0])
            message = parts[1]
            ack_flag = int(parts[2])
            priority = int(parts[3])
            
            self.send_message(
                message=message,
                dest_addr=dest_addr,
                require_ack=bool(ack_flag),
                priority=priority
            )
        except Exception as e:
            rospy.logerr(f"Message processing failed: {str(e)}")

    # Core Modem Operations ####################################################
    def send_message(self, message, dest_addr=None, require_ack=True, priority=1):
        """
        Queue message for transmission with ACK handling
        Priority 0: Timeout after ACK_TIMEOUT
        Priority 1: Persistent until delivered
        """
        ack_num = self.next_ack if require_ack else None
        if require_ack:
            self.next_ack += 1
            
        self.in_transit.append([
            message,
            time.time(),        # Initial send time
            0,                  # Last retry time
            ack_num,            # ACK number
            dest_addr,          # Destination address
            priority            # Message priority
        ])
        
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

    def send_acknowledgment(self, ack_num, dest_addr=None):
        """Send standalone ACK packet"""
        self.transmit_packet(f"@{ack_num}", dest_addr)

    # Message Processing ######################################################
    def handle_received_message(self, src_addr, message, received_ack, distance):
        """
        Central handler for received messages:
        1. Flash receive LED
        2. Process embedded ACKs
        3. Send acknowledgments if required
        4. Publish to ROS
        """
        self.led.on_recv_msg()
        
        # Process embedded ACK
        if received_ack is not None:
            self.received_acks.add(received_ack)
            
        # Send ACK if required
        if message is not None:
            self.publish_to_ros(message)
            
            # Log received message
            log_entry = f"[{time.time()}][RECV][src:{src_addr}]"
            if received_ack is not None:
                log_entry += f"[ack:{received_ack}]"
            if distance is not None:
                log_entry += f"[dist:{distance}]"
            log_entry += f" {message}" if message else ""
            
            with open("underwater_coms_recv.log", "a+") as f:
                f.write(log_entry + "\n")

    # Parsing Methods #########################################################
    def parse_broadcast(self, packet):
        """Parse broadcast message: #B<SRC><LEN><DATA>"""
        src_addr = packet[2:5]
        length = int(packet[5:7])
        data = packet[7:7+length]
        
        # Extract message and ACK
        parts = data.split('@', 1)
        message = parts[0]
        received_ack = int(parts[1]) if len(parts) > 1 else None
        
        return src_addr, message, received_ack, None

    def parse_unicast(self, packet):
        """Parse unicast message: #U<DEST><LEN><DATA>"""
        dest_addr = packet[2:5]
        length = int(packet[5:7])
        data = packet[7:7+length]
        
        # Extract message and ACK
        parts = data.split('@', 1)
        message = parts[0]
        received_ack = int(parts[1]) if len(parts) > 1 else None
        
        return dest_addr, message, received_ack, None

    def parse_range(self, packet):
        """Parse range info: #R<SRC>R<DIST>"""
        src_addr = packet[2:5]
        distance = int(packet[7:12]) * 1500 * 3.125e-5
        return src_addr, None, None, distance

    def parse_timeout(self, packet):
        """Parse timeout notification: #TO"""
        return None, None, None, None

    # Thread Loops ############################################################
    def receive_loop(self):
        """Continuous receive message processing loop"""
        while self.receive_active and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    raw_data = self.ser.readline().decode("utf-8").strip()
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
                result = self.parse_msg[prefix](packet)
                for callback in self.recv_callbacks:
                    callback(*result)
            except Exception as e:
                rospy.logerr(f"Packet processing failed: {str(e)}")

    def send_loop(self):
        """Message transmission management loop"""
        while self.sending_active and not rospy.is_shutdown():
            now = time.time()
            expired_messages = []
            
            for idx, packet in enumerate(self.in_transit):
                msg, send_time, last_retry, ack_num, dest, priority = packet
                
                # Handle expired messages
                if priority == 0 and (now - send_time) > self.ACK_TIMEOUT:
                    rospy.logwarn(f"Message expired: {msg}")
                    expired_messages.append(idx)
                    continue
                    
                # Handle retransmission
                needs_retry = (
                    ack_num is not None and 
                    ack_num not in self.received_acks and
                    (now - last_retry) > self.ACK_RETRY_INTERVAL
                )
                
                if needs_retry:
                    try:
                        # Format message with ACK
                        formatted = f"*{msg}*@{ack_num}" if ack_num else msg
                        self.transmit_packet(formatted, dest)
                        self.in_transit[idx][2] = now  # Update last retry time
                        self.log_sent_message(dest, msg, ack_num)
                    except Exception as e:
                        rospy.logerr(f"Transmission failed: {str(e)}")
            
            # Cleanup processed messages
            self.in_transit = [
                p for i, p in enumerate(self.in_transit)
                if i not in expired_messages and 
                (p[3] is None or p[3] not in self.received_acks)
            ]
            time.sleep(0.1)

    # Logging #################################################################
    def log_sent_message(self, dest_addr, message, ack_num):
        """Log sent messages to file"""
        log_entry = f"[{time.time()}][SEND][dst:{dest_addr}]"
        if ack_num is not None:
            log_entry += f"[ack:{ack_num}]"
        log_entry += f" {message}" if message else ""
        
        with open("underwater_coms_send.log", "a+") as f:
            f.write(log_entry + "\n")
        
        self.led.on_send_msg()

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










