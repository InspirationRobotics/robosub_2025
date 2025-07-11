import serial
import time
import threading

from auv.utils import deviceHelper

# TODO integrate with ros
import rospy
# TODO find msg type what we want to use for ros


class modem:
    def __init__(self):
        self.port   = deviceHelper.dataFromConfig("modem") # DONE use deviceHelper to get port
        self.sub    = deviceHelper.variables.get("sub") # TODO use deviceHelper to get what sub is this
        self.ser    = serial.Serial(port=self.port,
                           baudrate=9600,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS)

        # TODO consider using stacks or queue to store information
        self.msgs_send = []   # to store msgs want to send
        self.msgs_received = [] # store msg received

        # Initialize ros things
        rospy.init_node('modemNode', anonymous=True)
        self.rate = rospy.Rate(20) # TODO find a good frequency for the background thread

        # TODO replace message type None with the correct ros topic message
        self.sub_send = rospy.Subscriber('/auv/devices/modem/send', None, self.topic_callback)   # subscribe to a ros topic to send data when necessary
        self.pub_receive = rospy.Publisher('/auv/devices/modem/receive', None, queue_size=10) 


    def topic_callback(self, msg):
        # TODO this callback function defined what to do after receiving a message that the current sub want to send to a different sub
        
        # unpack rostopic msg

        # store msg to self.msgs_send -> again, this represent the messages we want to send to the other sub

        # we don't send the msg here, we all our sending and receiving in the background thread
        pass
    
    def receive_callback(self, msg):
        """
        The callback function when receive a message from the other sub
        Args:
            msg (String): TODO what message we are we expecting to receive and what to do with it
        """
        pass

    def background_thread(self):
        """
        This background thread will continously running when the node is up
        """
        while not rospy.is_shutdown():
            if len(self.msgs_send)>0: # if we have message to send
                # TODO put your logic for sending message
                pass

            myMessage = self.ser.readline()  # read a line from serial port to see if there's any message received without waiting for it

            if myMessage:  # if somehting is received
                # TODO put your logic for receiver and store the message received at self.msgs_received
                pass
                self.pub_receive()  # execute the call back with the message
            
    def start(self):
        bgThread = threading.Thread(target=self.background_thread)
        bgThread.start()
        

            

