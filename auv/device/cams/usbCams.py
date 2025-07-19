"""
Method for handling camera streams from USB cameras
"""

import os
import platform
import signal
import sys
import threading
import time

import cv2
import numpy as np
from auv.utils.img_bridge import CvBridge
from sensor_msgs.msg import Image

from auv.device.cams import pyfakewebcam # For simulating a webcam by writing frames to a V4L2 virtual video device


class USBCamera:
    """
    Creates a camera stream from the USB cameras
    """

    def __init__(self, rospy, id, ogDevice, newDevice):
        """
        Initializes the class

        Args:
            rospy: ROS context (provided outside the class)
            id: ID of the device (configuration)
            ogDevice: the real USB camera to be used
            newDevice: the fake USB camera to be used (meaning the simulated webcam using V4L2 to be used)
        """
        # Image width, height
        self.IMG_W = 640
        self.IMG_H = 480
        
        self.rospy = rospy
        self.id = id
        self.frame = None

        # For converting between ROS type "Images" to CV type "mat"
        self.br = CvBridge()

        # Node cycle rate (in Hz)
        self.loop_rate = rospy.Rate(30)

        self.isKilled = True
        self.ogDevice = ogDevice
        self.newDevice = newDevice

        # Create a fake webcam 
        self.fake = pyfakewebcam.FakeWebcam(newDevice, self.IMG_W, self.IMG_H)
        
        # Publishers and subscribers to be used 
        self.pub = self.rospy.Publisher(f"/auv/camera/videoUSBRaw{str(id)}", Image, queue_size=10)
        self.rospy.Subscriber(f"/auv/camera/videoUSBOutput{str(id)}", Image, self.callbackMain)

        self.time = time.time()

        # To communicate the corresponding paths for the same device (fake vs real)
        print(f"Camera ID {str(id)}: {ogDevice} is available at {newDevice}")

    def callbackMain(self, msg):
        """
        Received camera output and sends frame to be written to the virtual video device (fake)

        Args:
            msg: frame to be written
        """

        if self.isKilled:
            return
        self.time = time.time()
        # Convert the image from a ROS Image to an OpenCV image, and write it to the V4L2 video device
        self.sendFakeFrame(self.br.imgmsg_to_cv2(msg))

    def sendFakeFrame(self, msg):
        """
        Write a frame the the V4L2 virtual video device

        Args:
            msg (numpy.ndarray): the frame to be written
        """
        try:
            # Convert from BGR format to RGB format
            self.frame = cv2.cvtColor(msg, cv2.COLOR_BGR2RGB)
            # Write the frame to the V4L2 video device
            self.fake.schedule_frame(self.frame)
        except Exception as e:
            print(f"Camera {str(self.id)} Output Error, make sure running in correct python")
            print(e)

    def runner(self):
        """
        Continously reads frames from the camera, processes them, and publishes them to the correct ROS topic
        """
        while not self.rospy.is_shutdown() and not self.isKilled:
            try:
                # Read the frame
                ret, frame1 = self.cam.read()
                if ret:
                    # If the frame was read successfully, convert from a CV image (numpy.ndarray) to a ROS Image
                    msg = self.br.cv2_to_imgmsg(frame1)
                    # Publish the image to the correct publisher
                    self.pub.publish(msg)
                    if time.time() - self.time > 3:  
                        # If it has been more than 3 seconds since there was a new frame received, then default to the camera view (via V4L2 virtual video device)
                        self.sendFakeFrame(frame1)
                pass
            except Exception as e:
                print(f"Camera {str(self.id)} Input Error")
                print(e)
        self.loop_rate.sleep()

    def kill(self):
        """
        Kill all camera streams by joining threads and releasing resources
        """
        if self.isKilled:
            return
        self.rospy.loginfo(f"Killing Camera {str(self.id)} Stream...")
        self.isKilled = True
        self.usbThread.join()
        self.cam.release()
        self.rospy.loginfo(f"Killed Camera {str(self.id)} Stream...")

    def start(self):
        """
        Start the camera streams by starting new threads
        """
        self.cam = cv2.VideoCapture(self.ogDevice)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.IMG_W)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.IMG_H)
        self.isKilled = False
        self.rospy.loginfo(f"Starting Camera {str(self.id)} Stream...")
        self.usbThread = threading.Timer(0, self.runner)
        self.usbThread.daemon = True
        self.usbThread.start()