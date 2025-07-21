"""
Creates modem (intersub communication) functionality. 
"""

import time
import RPi.GPIO as GPIO
import serial
import threading
import rospy
from auv.utils import deviceHelper
from auv.utils.deviceHelper import dataFromConfig, variables # Configuration of the various devices attached to a sub (either Graey or Onyx)
from auv.mission.style_mission import StyleMission
from std_msgs.msg import String


class LED:
    """
    Turns LEDs on and off based on whether messages have been sent and/or received
    """
    def __init__(self):
        """
        Initialize the LED class; establishes the Jetson pin to light up when receiving (32) and sending (31) messages.
        """
        try:
            import RPi.GPIO as GPIO
            self.enabled = True
        except ImportError:
            print("RPi.GPIO not found, disabling LED")
            self.enabled = False

        self.t_pin = 31
        self.r_pin = 32

    def on_send_msg(self):
        """
        Light up Jetson pin 31 when sending messages
        """
        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

    def on_recv_msg(self):
        """
        Light up Jetson pin 32 when receiving messages
        """
        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.r_pin, GPIO.HIGH)
        # Do sync yaw
        yaw_cmd = StyleMission()
        yaw_cmd.run()
        yaw_cmd.cleanup()
        time.sleep(0.1)
        self.clean()

    def clean(self):
        """
        Clean up the LEDs -- turn all of the pins "off"
        """
        if not self.enabled:
            return
        
        # set to low to turn off
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.LOW)
        GPIO.output(self.r_pin, GPIO.LOW)
        GPIO.cleanup()


led = LED()


class Modem:
    """
    Establishes modem functionality.
    """
    def __init__(self, auto_start=True):
        """
        Initialize the Modem class. 

        Args:
            auto_start (bool): Flag indicating whether to automatically start the modem or not.
        """
        self.led = LED()
        self.__port = deviceHelper.variables.get("modem_port")  # Get the modem port ID from the JSON config file of the sub
        # Initialize the serial communication
        self.ser = serial.Serial(
            port=self.__port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        # Callback functions to parse each message type (broadcast, unicast, acknowledgment, timeout)
        self.parse_msg = {
            "#B": self.parse_B,
            "#U": self.parse_U,
            "#R": self.parse_R,
            "#T": self.parse_TO,
        }
        # Callback functions for receiving messages
        self.recv_callbacks = [
            self.on_receive_flash_led,
            self.on_receive_msg_logging,
            self.on_receive_ack,
        ]
        # Callback functions for sending messages
        self.send_callbacks = [
            self.on_send_msg_logging,
        ]
        self.data_buffer = ""  # This stores previously received data to handle large messages

        # A simple way to ensure message is received through acknowledgment messages
        self.ACK = 0
        self.blocked = False
        self.in_transit = []  # [message, time_sent, time_last_sent, ack-expected, modemaddr, priority]
        self.ack_received = [] # To hold the received acknowledgment numbers 

        # Variables specific to this particular type of modem
        self.modemAddr = deviceHelper.variables.get("modem_address")
        self.voltage = None

        self.receive_active = True
        self.sending_active = True
        self.received_handshake = False

        # The receive and sending threads
        self.thread_recv = threading.Thread(target=self._receive_loop)
        self.thread_send = threading.Thread(target=self._send_loop)

        # Initalize ros subscribers and publishers
        rospy.init_node('maestroServer')
        self.pub = rospy.Publisher('/auv/devices/modem_received', String, queue_size=10)
        self.sub = rospy.Subscriber("/auv/devices/modem_send", String, self.send_callback)
        if auto_start:
            self.start()
    def publish_to_ros(self, msg):
        tosend = String()
        tosend.data = msg
        self.pub.publish(tosend)

    def send_callback(self, msg):
        """
        Callback function when received a data from /auv/devices/modem_send
        This function store a message to self.in_transit
        """
        toSend = msg.data

        # TODO parse the message and format it to """[msg, time.time(), 0, ack, dest_addr, priority]""" so we can store it at self.in_transit
        # Get a list ready to be added to self.in_transit


        # Example of expected message: destination Address-Movement-Acknowledgement-Priority. "020-ROLL-1-0"
        parts = toSend.strip().split("-")
        if len(parts) != 4:
            print(f"[Modem] Malformed message: '{toSend}' (expected 4 parts)")
            return

        MSG = parts[1]

        try:
            DEST_ADDR = int(parts[0])
            ACK = int(parts[2])
            PRIORITY = int(parts[3])
        except ValueError as e:
            print(f"[Modem] Failed to parse integer fields: {e}")
            return

        self.send_msg(msg= MSG, ack= ACK, dest_addr= DEST_ADDR, priority= PRIORITY) # need to fill these in by parsing the incoming string form the topic

    def _send_to_modem(self, data):
        """
        Send data to the modem for intersub communication (send the data to the other sub), and wait for a response.

        Args:
            data: Data to be encoded and sent to the modem

        Returns:
            str: Decoded response message from the other sub
        """
        buffer = f"${data}"
        self.ser.write(buffer.encode()) # Convert the string of data into bytes
        out = b""
        time.sleep(0.1)
        while self.ser.in_waiting > 0:
            out += self.ser.read(1)
        # If there is an actual response message from the other sub, then decode it(change from bytes back to strings).
        if out != b"": 
            output = out.decode()
            return output

    def query_status(self):
        """
        Queries the status of the modem (voltage and modem address)
        """
        QUERY_COMMAND = "?"
        data = self._send_to_modem(QUERY_COMMAND)
        if data is None:
            return
        self.modemAddr = data[2:5] # Get the address of the modem
        self.voltage = round(float(int(data[6:11]) * 15 / 65536), 2) # Extract and manipulate the voltage data
        # Print the voltage and modem address
        print(f"Modem Addr: {self.modemAddr}")
        print(f"Voltage: {str(self.voltage)}")

    def _transmit_packet(self, msg, addr=None):
        """
        Transmit a packet of data to the modem (send the packet to the other sub)

        Args:
            msg: Message to send
            addr: Where to send the message to, defaults to None
        """
        # Select the transmission node
        prefix = "U"
        if addr is None:
            prefix = "B"
            addr = ""

        # Encode based on the length of the message
        length = len(msg.encode("utf-8"))
        if length < 10:
            length = f"0{str(length)}"

        # Send the message
        data = f"{prefix}{addr}{length}{msg}"
        self._send_to_modem(data)

    def _transmit(self, msg, ack=None, dest_addr=None):
        """
        Transmit the messsage to the modem. This is a blocking function until the message is sent.

        Args:
            msg: Message payload to send
            ack: Acknowledgment message to send, defaults to None
            dest_addr: Destination address, where to send the message to, defaults to None

        Returns:
            The time when the packet was transmitted
        """
        while self.blocked:
            time.sleep(0.01)
        self.blocked = True

        # If there is no message payload, send an acknowledgment message
        if msg is None:
            # ack message
            msg = f"@{ack}"
        else:
            # Create a normal message with and acknowledgment message
            # If there is no acknowledgment message, generate a new one
            if ack is None:
                self.ACK += 1
                ack = self.ACK
            msg = f"*{msg}*@{ack}"

        # Max length of a message is 64 bytes
        # Minimum length of a message is 3 bytes
        length = len(msg.encode("utf-8"))
        if length > 64:
            if length % 64 < 3:
                msg = f"  {msg}"
                length += 2

        # Split the message into multiple packets if the length of the message is too large
        dataToSend = []
        while length > 64:
            dataToSend.insert(0, msg[-64:])
            length -= 64
            msg = msg[:length]

        # Insert the message into the dataToSend list
        if length > 0:
            dataToSend.insert(0, msg)

        # Send the dataToSend list, which contains the message to send, to the right address. Breaks the message down into individual
        # characters and sends each one as a byte.
        for data in dataToSend:
            length = len(data.encode("utf-8"))

            self.led.on_send_msg()
            self._transmit_packet(data, dest_addr)

            sleepAmt = round(0.105 + (length + 16) * 0.0125, 3) # Calculate the amount of time needed for each piece of information to travel
            time.sleep(sleepAmt)

        self.blocked = False
        t = time.time()

        # Call the _on_send_msg_logging function with the correct arguments, to log the sent message
        for callback in self.send_callbacks:
            callback(dest_addr, msg, ack)

        return t

    def send_msg(self, msg, ack=None, dest_addr=None, priority=1):
        """
        Append a msg to the sending list. This is a non-blocking function.
        The message will be processed after all the previous messages are sent out.

        Messages with priority 0 will be timed out after 30 seconds.
        Messages with priority 1 won't be timed out.

        Args:
            msg: Message to send
            ack: Acknowledgment message, defaults to None
            dest_addr: Address to send the message to, defaults to None
            priority (int): Flag indicating priority, 0 or 1
        """
        # Generate a new, unique acknowledgment message if there is currently none
        if ack is None:
            self.ACK += 1
            ack = self.ACK

        # Append the message to the sending list
        self.in_transit.append([msg, time.time(), 0, ack, dest_addr, priority])

    def send_ack(self, ack, dest_addr=None):
        """
        Send an acknowledgment message to the modem. 
        This is a non-blocking function. 
        This will ONLY send an ACK, not a message.
        
        Args:
            ack: Acknowledgment message to send
            dest_addr: Destination to send the message to, defaults to None.
        """
        packet = [None, time.time(), 0, ack, dest_addr, 0] # Set the format of the acknowledgment message
        for it, p in enumerate(self.in_transit):
            # If there's a message that's not an acknowledgment message (p[0] != None), or the acknowledgment message is greater than the passed-in acknowledgment
            # value, meaning that the passed-in value is obsolete, replace the current message in the sending list with the new message
            if p[0] is not None or p[3] > ack:
                self.in_transit[it] = packet
                return
        self.in_transit.append(packet) # Append the new message to the sending list

    def _send_loop(self):
        """
        Manages the modem transmission loop
        """
        while self.sending_active:
            to_remove = []
            self.ack_received = []

            for it, packet in enumerate(self.in_transit):
                msg, time_sent, time_last_sent, ack, dest_addr, priority = packet

                # Stop sending messages
                if self.sending_active is False:
                    return

                # If there is no actual message payload, send an acknowledgment message
                if msg is None:
                    self._transmit(msg, ack=ack, dest_addr=dest_addr)
                    to_remove.append(it)
                    continue

                # Timeout if no acknowledgment message has been received after 30 seconds
                if time.time() - time_sent > 30 and priority == 0:
                    print(f'[WARNING] Message "{msg}" timed out')
                    to_remove.append(it)
                    continue

                # Retry if no acknowledgment message is received after 1 second (better be safe than sorry)
                if time.time() - time_last_sent > 1.0 and ack not in self.ack_received:
                    ret = self._transmit(msg, ack=ack, dest_addr=dest_addr)
                    packet[2] = ret

                time.sleep(0.5)
            time.sleep(0.5)

            # Remove timed-out (to_remove) messages and received acknowledgmen[packet for it, packet in enumerate(self.in_transit) if it not in to_remt messages (self.ack_received)
            self.in_transit = [packet for (it, packet) in enumerate(self.in_transit) if it not in to_remove and packet[3] not in self.ack_received]

    def _dispatch(self, packet):
        """
        Dispatches each message by its message type to its corresponding callback 

        Args:
            packet: The received message to be dispatched in order to be parsed by the respective callback
        """
        msg_type = packet[:2] # Get the type of the message

        if msg_type not in self.parse_msg.keys():
            return

        # Parse the message by message type
        src_addr, msg, ack, distance = self.parse_msg[msg_type](packet)

        # Determine if the message is complete or not (if the message ends without an asterisk, that means it is incomplete)
        # NOTE: there will be an issue if we receive different messages from different modems at the same time
        if msg is not None and msg[-1] != "*":
            self.data_buffer += msg # Store the first part of the message
            return

        # If we are at this position in the loop, that means the message was incomplete; get the end of the message by concatenation and wipe
        # self.data_buffer so it can be used for the next incomplete message
        if msg is not None:
            msg = self.data_buffer + msg
            self.data_buffer = ""

        # Dispatch message to the received messsage callbacks
        for callback in self.recv_callbacks:
            callback(src_addr, msg, ack, distance)

    def _receive_loop(self):
        """
        Loop for receiving messages
        """
        while self.receive_active:
            # Read a line of data from the serial connection
            raw_packet = self.ser.readline()

            # If the byte was empty, continue
            if raw_packet == b"":
                pass

            try:
                # TODO: Handle large messages
                # Decode the byte into a UTF-8 encoded string
                packet = raw_packet.decode("utf-8")
                # Dispatch the packet to be handled by callback functions
                self._dispatch(packet)

            except KeyboardInterrupt:
                break

            except Exception as e:
                print(e)
                pass

    def parse_B(self, packet: str):
        """
        Parses through broadcast messages (messages that are sent to "everybody")

        Args:
            packet (str): The message received. Format will be "#Bxxxnnddd".

        Returns:
            tuple: source address, message payload, acknowledgement (if present), distance between recipient and sender (None in this case)
        """

        msg_type = packet[:2]

        # Verify that the message is a broadcast message
        if msg_type != "#B":
            print("Received a non broadcast message in broadcast callback")
            return

        src_addr = packet[2:5] # Extract the source address
        length = int(packet[5:7]) # Extract the length of the message

        data = packet[7 : 7 + length] # Extract the message payload
        msg = data.split("@")[0] 
        ack = None if "@" not in data else data.split("@")[1] # Find whether there was an acknowledgment of reception in the message
        ack = int(ack)
        distance = None

        return src_addr, msg, ack, distance

    def parse_U(self, packet: str):
        """
        Parses unicast messages (messages that are sent directly to another device)

        Args:
            packet (str): The message received. Will be in format "#Unnddd".

        Returns:
            tuple: the source address (None in this case), message payload, acknowledgement (if present), and distance between sender and recipient (None in this case)
        """

        msg_type = packet[:2]

        # Verify that the message is a unicast message
        if msg_type != "#U":
            print("Received a broadcast message in unicast callback")
            return

        src_addr = None
        length = int(packet[5:7]) # Extract the length of the message

        data = packet[7 : 7 + length] # Extract the message payload
        msg = data.split("@")[0]
        ack = None if "@" not in data else data.split("@")[1] # Find whether there was an acknowledgment of receiving the message in the message
        ack = int(ack)
        distance = None

        return src_addr, msg, ack, distance

    def parse_R(self, packet: str):
        """
        Parses acknowledgment messages (messages that are sent confirming the reception and successful processing of the message)

        Args:
            packet (str): The message received. Will be in format "#RxxxTyyyyy".

        Returns:
            tuple: the source address, message payload (None in this case), acknowledgement (None in this case), and distance between sender and recipient
        """

        msg_type = packet[:2]

        # Verify that the message is an acknowledgment message
        if msg_type != "#R":
            print("Received a broadcast message in unicast callback")
            return

        src_addr = packet[2:5] # Get the source address
        msg = None
        ack = None
        distance = packet[7:12] * 1500 * 3.125e-5 # Calculate the distance by extracting the number defined by positions 7-12 and multiplying by constants

        return src_addr, msg, ack, distance

    def parse_TO(self, packet: str):
        """
        Parses timeout messages

        Args:
            packet (str): The message received. Will be in format "#TO".

        Returns:
            tuple: the source address (None in this case), message payload (None in this case), acknowledgement (None in this case), and distance between sender and recipient (None in this case)
        """
        return None, None, None, None

    def on_receive_msg_logging(self, src_addr: str, msg: str, ack: int, distance: int):
        """
        Log the received message into a file
        
        Args:
            src_addr (str): Source address
            msg (str): Message payload
            ack (int): Acknowledgment message
            distance (int): Distance between sender and receiver
        """

        # Open the log file and write the message into the file

        with open("underwater_coms_recv.log", "a+") as f:
            if distance is not None and msg is not None:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}][dist:{distance}] {msg}\n")
            elif distance is not None:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}][dist:{distance}]\n")
            elif msg is not None:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}] {msg}\n")
            else:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}]\n")
        
        self.publish_to_ros(msg)

    def on_receive_msg(self, src_addr: str, msg: str, ack: int, distance: int):
        """
        Callback function for the received message -- prints out the message in the same format as it would be written in the log file

        Args:
            src_addr (str): Source address
            msg (str): Message payload
            ack (int): Acknowledgment message
            distance (int): Distance between sender and receiver
        """
        if msg is None:
            return
        
        self.publish_to_ros(msg)
        print(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}][dist:{distance}] {msg}")

    def on_receive_flash_led(self, src_addr: str, msg: str, ack: int, distance: int):
        """
        Flashes the designated "received" LED when the message is received

        Args (unused):
            src_addr (str): Source address
            msg (str): Message payload
            ack (int): Acknowledgment message
            distance (int): Distance between sender and receiver
        """
        self.publish_to_ros(msg)
        led.on_recv_msg()

    def on_receive_ack(self, src_addr: str, msg: str, ack: int, distance: int):
        """
        Callback for received acknowledgment message

        Args (unused):
            src_addr (str): Source address
            msg (str): Message payload
            ack (int): Acknowledgment message
            distance (int): Distance between sender and receiver
        """
        if ack is None:
            return

        # Send an acknowledgment message for the reception of the message
        if msg is None:
            self.send_ack(ack)
            return

        # Add the acknowlegment message to the list of received acknowledgment numbers, as long as there is a corresponding message found that goes along with
        # the corresponding acknowledgment number (checks whether there is a message that had the received acknowledgment number)
        for packet in self.in_transit:
            if packet[3] == ack:
                self.ack_received.append(ack)
                return
        self.publish_to_ros(msg)
        print(f'[WARNING] Received ack: "{ack}" but no corresponding message found, maybe timed out?')

    def on_send_msg_logging(self, dst_addr: str, msg: str, ack: int):
        """
        Log the sent message in the log file

        Args:
            dst_addr (str): Address of where the message is being sent
            msg (str): Message payload
            ack (int): Acknowledgment number
        """
        with open("underwater_coms_send.log", "a+") as f:
            # Include the message if there is an actual message
            if msg is None:
                f.write(f"[{time.time()}][SEND][dst:{dst_addr}][ACK:{ack}]\n")
            else:
                f.write(f"[{time.time()}][SEND][dst:{dst_addr}][ack:{ack}] {msg}\n")

        self.publish_to_ros(msg)

    def on_receive_handshake(self, msg: str):
        """
        Callback for receiving the handshake

        Args:
            msg (str): Message received
        """
        if msg == "handshake":
            print("Handshake received")
            self.received_handshake = True

    def handshake_start(self):
        """
        Initiates a handshake by sending a handshake and waiting for an acknowledgment

        Returns:
            bool: Whether there was a handshake or not
        """
        self.received_handshake = False
        # Append the on_receive_handshake to the received message callback function list
        if self.on_receive_handshake not in self.recv_callbacks:
            self.recv_callbacks.append(self.on_receive_handshake)

        # Sending handshake (30s timeout)
        msg = "handshake"
        self.send_msg(msg, ack=42, priority=0) # Priority 0 means the message will timeout in 30 seconds
        print("Waiting for handshake...")

        # Loop continues until 30 seconds have elapsed or all other messages in the queue have been sent out
        while not self.received_handshake:
            if len(self.in_transit) == 0:
                break
            time.sleep(0.1)

        if self.received_handshake:
            print("Handshake successful")
        else:
            print("Handshake failed")

        return self.received_handshake

    def dummy_callback(self, src_addr: str, msg: str, ack: int, distance: int):
        pass

    def start(self):
        """Start the receiving and sending threads"""
        self.receive_active = True
        self.sending_active = True
        self.thread_recv.start()
        self.thread_send.start()
        rospy.spin()  # keep the script running in termianl

    def stop(self):
        """Stop the receiving and sending threads"""
        self.receive_active = False
        self.sending_active = False
        self.thread_recv.join()
        self.thread_send.join()


def manual_coms():
    """
    Creates a modem object to enter messages manually typed in by the user
    """
    modem = Modem()
    while True:
        msg = input("Enter message: ")
        modem.send_msg(msg, priority=1)

# For testing purposes
if __name__ == "__main__":
    modem = Modem()
    modem.send_msg("this is a test")
    modem.send_msg("blabla")
    modem.send_msg("omg this is insane !!!")

    # led.on_send_msg()
    # led.on_recv_msg()
