import time
import RPi.GPIO as GPIO
import serial
import threading
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class LED:
    """
    Turns LEDs on and off based on whether messages have been sent and/or received
    How to use it:  rosservice call /auv/devices/LED/received "{}"
    """
    def __init__(self):
        """
        Initialize the LED class; establishes the Jetson pin to light up when receiving (32) and sending (31) messages.
        """
        try:
            import RPi.GPIO as GPIO
            self.enabled = True
            rospy.init_node("LED_node", anonymous=True)  

            self.service = rospy.Service("/auv/devices/LED/received", Trigger, self.on_recv_msg)
            self.service = rospy.Service("/auv/devices/LED/send", Trigger, self.on_send_msg)
        except ImportError:
            print("RPi.GPIO not found, disabling LED")
            self.enabled = False

        self.t_pin = 31
        self.r_pin = 32

        rospy.loginfo(f"Enabled: {self.enabled}")
        rospy.loginfo("Ready to take LED requests.")
        rospy.spin()


    def on_send_msg(self,request):
        """
        Light up Jetson pin 31 when sending messages
        """
        rospy.loginfo("Received")
        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

        return TriggerResponse(
            success=True,
            message="Modem message sent"
        )

    def on_recv_msg(self,request):
        """
        Light up Jetson pin 32 when receiving messages
        """
        rospy.loginfo("Received")

        if not self.enabled:
            return
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.r_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

        return TriggerResponse(
            success=True,
            message="Modem message received"
        )

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

if __name__=="__main__":
    node = LED()