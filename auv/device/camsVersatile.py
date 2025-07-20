"""
Handles camera streams
"""

import lsb_release

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")
import os
import platform
import signal
import sys
import time
import rospy
import json
import re
import depthai as dai
from std_msgs.msg import String
from auv.device.cams import pyfakewebcam, usbCams, oakCams
from auv.utils import deviceHelper


# Order of cameras is [forward-facing, bottom-facing]
usbIDS  = [deviceHelper.dataFromConfig("forwardUSB"), deviceHelper.dataFromConfig("bottomUSB")]
ogDev   = []
oaks    = []
newDev  = []


def list_devices():
    """
    Get the number of avaliable DepthAI devices (OAK-D cams) and put them in a list

    Returns:
        list: List of avaliable DepthAI devices (OAK-D cams)
    """
    # We want to return Forward Oak then Bottom Oak
    forward_Oak_MXID = deviceHelper.dataFromConfig("forwardOak")
    bottom_Oak_MXID = deviceHelper.dataFromConfig("bottomOak")

    print(forward_Oak_MXID)
    print(bottom_Oak_MXID)

    available_devices = [device.getMxId() for device in dai.Device.getAllAvailableDevices()]
    print(available_devices)

    if len(available_devices) < 2:
        pass
    elif len(available_devices) == 2:
        ordered_devices = [None, None]
        for mxid in available_devices:
            print(mxid)
            if mxid == forward_Oak_MXID:
                ordered_devices[0] = mxid
            elif mxid == bottom_Oak_MXID:
                ordered_devices[1] = mxid

        available_devices = ordered_devices

    return available_devices

    # for device in dai.Device.getAllAvailableDevices():
    #     available_devices.append(device.getMxId())

    # print(available_devices)
    # if len(available_devices) < 2:
    #     pass
    # elif len(available_devices) == 2:
    #     for mxid in available_devices:
    #         print(mxid)
    #         if mxid == forward_Oak_MXID:
    #             available_devices[0] = mxid
    #         elif mxid == bottom_Oak_MXID:
    #             available_devices[1] = mxid

    # return available_devices


def difference(string1, string2):
    """
    Find the symmetric difference between two strings (treats the string as a set of words, and finds the number of words that are 
    in one list but not the other).

    Args:
        string1 (str): 1st string
        string2 (str): 2nd string

    Returns:
        list: The list of words that are in one list but not the other
    """
    string1 = string1.split()
    string2 = string2.split()
    A = set(string1)
    B = set(string2)
    str_diff = A.symmetric_difference(B)
    return list(str_diff)


def num_sort(test_string):
    """
    Return the first numeric value in a string

    Args:
        test_string: String to find the find the first numeric value (positionally) of
    Returns:
        list: First numeric value found in the input string
    """
    return list(map(int, re.findall(r"\d+", test_string)))[0]



class cameraStreams:
    """
    Class to start and control the camera streams
    """
    def __init__(self, debug=False):
        """
        Initialize the cameraStreams object

        Args:
            debug (bool): Flag indicating whether debugging mode is on or not, defaults to False
        """
        rospy.loginfo("Initializing Camera Streams...")
        self.cams = [] # Camera objects
        self.camID = [] # Camera IDs
        self.activeCams = [] # Active camera IDs
        self.debug = debug
        self.limit = 2  # Maximum number of cameras that can be used at a time

        # Instantiate (create an instance of) each USB and OAK-D camera, append each camera object to the camera object list, and append the ID to the 
        # camera ID list.
        for i in range(camAmt):
            self.cams.append(usbCams.USBCamera(rospy, i, ogDev[i], newDev[i]))
            self.camID.append(i)
        for i in range(oakAmt):
            self.cams.append(oakCams.oakCamera(rospy, i + camAmt, oaks[i], newDev[i + camAmt]))
            self.camID.append(i + camAmt)
        rospy.Subscriber("/auv/camsVersatile/cameraSelect", String, self.callbackCamSelect) # Subscribe to the camera selection topic
        if debug:
            print("*****************\ndebug mode is ON\n*****************")

    def start(self):
        """
        Start the camera stream; if debug mode is enabled, this method enters a user control loop to control the camera.
        """
        while self.debug and not rospy.is_shutdown():
            # Select camera ID -- this will create a camera stream from that camera
            stream = input("Select Camera ID to control\n")
            print(
                "Camera "
                + stream
                + " is selected. Enter 'start' or 'stop' or 'model' to pick a model to run or 'back' to pick different camera \n"
            )
            # Control the camera with "start", "stop", "back", or "model"
            cmd = input()
            if cmd == "start":
                self.camCtrl(stream, True)
            elif cmd == "stop":
                self.camCtrl(stream, False)
            elif cmd == "back":
                continue
            elif cmd == "model":
                model = input("Name of model to run: (or 'back' to return to camera selection)\n")
                if cmd == "back":
                    continue
                else:
                    self.camCtrl(stream, True, model)
        rospy.spin()

    def camCtrl(self, id, state, model=None):
        """
        Controls the state of a camera stream

        Args:
            id (str): Camera ID
            state (bool): The state of the camera stream -- if True, start the stream, if false, stop the stream
            model: ML model to run on the stream (defaults to None)

        Returns:
            bool: Indicating success of activating a camera stream or failure
        """
        # If the ID format is not a numerical string
        if not id.isnumeric():
            print(f"Invalid ID {id}, please pick from {str(self.camID)}")
            return
        
        id = int(id)

        # If the ID is not one of the recognized current camera IDs
        if id not in self.camID:
            print(f"Invalid ID of {id}, please pick from {str(self.camID)}")
            return False
        
        # If the camera is not currently active, and there are already 2 streams running
        if state and len(self.activeCams) == 2 and id not in self.activeCams:
            print(f"2 camera streams are already active with ids: {str(self.activeCams)}")
            print(f"Killing camera {str(self.activeCams[0])} to start camera {str(id)}")
            self.camCtrl(str(self.activeCams[0]), False)
            self.camCtrl(str(id), True, model)
            #print("Please stop one of these streams first\n")
            return False
        
        # If state == True, and the camera is not currently active
        if state and id not in self.activeCams:
            # If there is no model, then immediately start
            if model == None:
                self.cams[id].start()
            else:
                # If we are trying to run a ML model on a USB camera
                if id - camAmt < 0:
                    print("Cannot load a model on a nonOak camera")
                    return False
                self.cams[id].start(model)
            self.activeCams.append(id)
            return True
        
        # If the camera is active and state == False, then kill the stream
        if not state and id in self.activeCams:
            self.cams[id].kill()
            self.activeCams.remove(id)
            return True
        
        # If there is a model to run, and the camera is already active
        if state and id in self.activeCams and model != None:
            # Make sure the camera is an OAK-D, not a USB
            if id - camAmt < 0:
                print("Cannot load a model on a nonOak camera")
                return False
            self.cams[id].callbackModel(model, True)
            return True

    """
    The camera selection message will be formatted as a JSON. 

    The JSON will include the following information:
        - Camera ID (an ID for each camera):
            - If the camera is a USB low-light: forward-facing = 0 ; bottom-facing = 1
            - If the camera is an OAK-D: forward-facing = 10 ; bottom-facing = 20 ; POE (Powered Over Ethernet) = 30

            ** See cv_handler.py, in _ScriptHandler.initCameraStreams() for the logic behind this

        - Mode:
            - If the mode is "start", the state will be True
            - If the mode is "stop", the state will be False

        - Model (will only be parsed if the mode == "start"):
            - Name of ML model to run by mission (i.e. "gate", "buoy", etc.)

        - Kill (ONLY call to kill all camera streams):
            - If Kill is called, it will disregard all other parameters above

    Sample Messages:

    Start forward-facing OAK-D camera stream while running the "gate" model: 
    {[“camera_ID”]: 10, [“mode”]: True, [“Model”]: “gate”}

    Stop the Forward OAK-D: 
    {[“camera_ID”]: 10, [“mode”]: False}

    Start USB low-light forward-facing camera stream: 
    {[“Camera_ID”]: 0, [“mode”]: True}

    Start POE OAK-D camera stream without a model:
    {[“camera_ID”]: 30, [“mode”]: True, [“Model”]: “raw”} or
    {[“camera_ID”]: 30, [“mode”]: True}

    Kill all active cameras streams: 
    {[“Kill”]: True}
    """

    def callbackCamSelect(self, msg):
        """
        Callback function for handling received camera selection messages on the camera selection ROS topic.

        Args:
            msg: Message received on the ROS topic
        """
        data = json.loads(msg.data) # Convert to JSON
        print("Received data:", data)
        kill = data.get("kill")
        if kill != None:
            for i in self.activeCams:
                self.camCtrl(i, False) # Kill the camera stream
        camID = data.get("camera_ID") # Get the camera ID
        mode = data.get("mode") # Get the mode of the camera (either "start" or "stop")
        model = data.get("model") # Get the model to run

        # See above for the logic behind this -- these are for the OAK-D cameras
        if camID >= 10:
            camID = (camAmt - 1) + camID / 10

        # Start or stop the camera stream based on the mode
        if mode == "start":
            self.camCtrl(str(int(camID)), True, model)
        if mode == "stop":
            self.camCtrl(str(int(camID)), False)

    def stop(self):
        """
        Stop the camera stream, delete the camera object.
        """
        for i in self.cams:
            i.kill()
            del i


def onExit(signum, frame):
    """
    For exiting the camera streams gracefully

    Args:
        signum: Signal number
        frame: Current stack frame -- frame that the camera stream was currently displaying when shut down
    """
    try:
        print("\Closing Cameras and exiting...")
        my_node.stop()
        time.sleep(3)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        print("Please run: \nsudo modprobe -r v4l2loopback\n")
        exit(1)
    except:
        pass


preDevices = os.popen("ls /dev/video*").read()  # Find the video devices present (USB cameras)
ogDev = deviceHelper.findCam(usbIDS)            # Find the USB cameras avaliable
oaks = list_devices()                           # Find the OAK-D cameras avaliable
oakAmt = len(oaks)                              # Get the number of OAK-D cameras avaliable
print(oaks)
camAmt = len(ogDev)                             # Find the number of USB cameras avaliable
print(ogDev)
os.system(f"sudo modprobe v4l2loopback devices={str(camAmt + oakAmt)}")     # Create V4L2 (Video 4 Linux Version 2) virtual devices for each camera (OAK-D and USB)
postDevices = os.popen("ls /dev/video*").read()                             # Find the new total video devices present (this should be OAK-Ds + USBs + V4L2s)
diff = difference(preDevices, postDevices)                                  # Find the new video devices present (these are the V4L2 devices)
if len(diff) == 0:
    print("Failed to detect if any new v4l2loopback devices were made")
    exit(1)

# Process the V4L2 devices in numerical reverse order
for i in range(camAmt + oakAmt - 1, -1, -1):
    newDev.append(diff[i])

newDev.sort(key=num_sort)

signal.signal(signal.SIGINT, onExit)    # If Ctrl + C is pressed, exit the camera streams

if __name__ == "__main__":
    rospy.init_node("CameraStream", anonymous=True)
    my_node = cameraStreams(True)
    my_node.start()

# https://youtu.be/2l913YwWYe4
# https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

# At the top of pyfakewebcam.py and __init__.py add "from __future__ import absolute_import"
# path: /home/inspiration/.local/lib/python2.7/site-packages/pyfakewebcam