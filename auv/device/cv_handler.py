"""
For handling the running of individual CV scripts.
"""
import lsb_release

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")

import importlib
import json
import os
import sys
import threading
import time
import traceback

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CVHandler:
    """
    Class for running individual CV scripts
    """
    def __init__(self, **config):
        """
        Initialize the CV handler

        Args:
            config: Configuration of the sub to be used as settings to run a particular mission (not really used here)
        """
        self.config = config

        # Stores the running CV scripts, with the dictionary's keys for each script being the script's file name
        self.active_cv_scripts = {}
        self.subs = {}

    def start_cv(self, file_name, callback, dummy_camera=None):
        """
        Start a CV script

        Args:
            file_name (str): the name of the file that contains the script
            callback: function that gets the JSON data from the CV handler
            dummy_camera (optional): the video file that simulates a camera feed
        """
        if file_name in self.active_cv_scripts:
            print("[ERROR] [cv_handler] Cannot start a script that is already running")
            return

        try:
            # Generic module file path: auv.cv.file_name
            module = importlib.import_module(f"auv.cv.{file_name}")
        except Exception as e:
            print("[ERROR] [cv_handler] Error while importing CV module from file name")
            print(f"[ERROR] {e}")
            return

        # Import the CV class defined in the particular file
        cv_class = getattr(module, "CV", None)
        if cv_class is None:
            print("[ERROR] [cv_handler] No CV class found in file, check the file name and file content")
            return

        # Initialize simulated CV script handler or a real CV script handler
        if dummy_camera:  
            self.active_cv_scripts[file_name] = _DummyScriptHandler(file_name, cv_class(**self.config), dummy_camera)
        else:  
            self.active_cv_scripts[file_name] = _ScriptHandler(file_name, cv_class(**self.config))
        self.subs[file_name] = rospy.Subscriber(f"auv/cv_handler/{file_name}", String, callback)

    def stop_cv(self, file_name):
        """
        Stop a CV script

        Args:
            file_name (str): File that contains the script that should be stopped
        """
        if file_name not in self.active_cv_scripts:
            print("[ERROR] [cv_handler] Cannot stop a script that is not running")
            return

        self.active_cv_scripts[file_name].stop()
        del self.active_cv_scripts[file_name]

        self.subs[file_name].unregister()
        del self.subs[file_name]

    def switch_oakd_model(self, file_name, model_name):
        """
        Changes the ML model of a running CV script. This method makes sure that the CV script is running, and specifically on an OAK-D camera.

        Args:
            file_name: the file name that contains the relevant CV script
            model_name: the name of the new model to be run
        """
        if file_name not in self.active_cv_scripts:
            print("[ERROR] [cv_handler] Cannot change model of a script that is not running")
            return

        if not self.active_cv_scripts[file_name].is_oakd:
            print("[ERROR] [cv_handler] Cannot change model of a script that is not running on an OAK-D camera")
            return

        if not isinstance(model_name, str):
            print("[ERROR] [cv_handler] Model name must be a string")
            return

        self.active_cv_scripts[file_name].pub_oakd_model.publish(model_name)
        print(f"[INFO] model published {model_name}")

    def set_target(self, file_name, target):
        """
        Changes the target of a CV script (this is important in something like the gate mission), where there is a particular side/target
        to aim at. As an example, in respect to the gate mission, the target side to enter the gate would be set to "Abydos".

        Args:
            file_name (str): the file name that contains the relevant CV script (the script that we want to set the target of)
            target: the target
        """
        if file_name not in self.active_cv_scripts:
            print("[ERROR] [cv_handler] Cannot change target of a script that is not running")
            return

        self.active_cv_scripts[file_name].target = target


class _ScriptHandler:
    """
    Handle running the CV scripts on a real camera feed (live).
    """
    def __init__(self, file_name, cv_object):
        """
        Initialize the ScriptHandler class object.

        Args:
            file_name: The file that contains the script
            cv_object: The class of the particular CV mission
        """
        self.cv_object = cv_object
        self.file_name = file_name

        # Get the camera topic, if not specified, use the default front camera.
        self.camera_topic = getattr(self.cv_object, "camera", None)
        if self.camera_topic is None:
            print("[WARN] No camera topic specified, using default front camera")
            self.camera_topic = "/auv/camera/videoUSBRaw0"

        # Create a CV bridge; enables transfer of data between ROS Images and OpenCV-type images.
        self.br = CvBridge()

        # Create the ROS node, the subscribers and the publishers
        self.sub_cv = rospy.Subscriber(self.camera_topic, Image, self.callback_cam)
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10)
        self.pub_out = rospy.Publisher(f"auv/cv_handler/{file_name}", String, queue_size=10)
        self.pub_cam_select = rospy.Publisher("/auv/camsVersatile/cameraSelect", String, queue_size=10)

        # Initialize the OAK-D camera topics and other stuff
        if "OAKd" in self.camera_topic:
            self.is_oakd = True
            pub_oakd_model_topic = self.camera_topic.replace("Raw", "Model")
            pub_oakd_data_topic = self.camera_topic.replace("Raw", "Data")
            self.pub_oakd_model = rospy.Publisher(pub_oakd_model_topic, String, queue_size=10)
            self.sub_oakd_data = rospy.Subscriber(pub_oakd_data_topic, String, self.callback_oakd_data) # When messages are published to the Oak-D data topic, callback_oakd_data() will be invoked
        else:
            self.is_oakd = False
            self.pub_oakd_model = None
            self.sub_oakd_data = None
        time.sleep(1.5)  # Wait for the ROS publisher / subscriber to be ready.

        self.initCameraStream()  # Sends JSON data to camsVersatile.py to determine which camera to start and with which model (assuming there will be a model).

        self.target = "main"
        self.oakd_data = None
        self.next_frame = None
        self.running = False
        self.closed = False
        self.last_received = time.time()
        self.last_processed = self.last_received

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def callback_cam(self, msg):
        """
        Method to callback the camera subscriber. Converts ROS Images into OpenCV images for processing.

        Args:
            msg: ROS Image to be processed into cv2 type images
        """
        try:
            self.next_frame = self.br.imgmsg_to_cv2(msg)
            self.last_received = time.time()
        except Exception as e:
            print("[ERROR] [cv_handler] Error while converting image to cv2")
            print(f"[ERROR] {e}")

    def callback_oakd_data(self, msg):
        """
        Method to callback the Oak-D data subscriber. The data from the Oak-D data topic contains the results of the model being run on the camera, i.e.
        the list of detections passed back by the ML model after processing each frame from the Oak-D camera. This method loads that detections data into a 
        JSON format and stores each detection and its relevant data.
        
        Args:
            msg: Data from the Oak-D data ROS topic, which contains the detections passed back by the ML model in question.
        """
        try:
            dataList = []
            data = json.loads(msg.data)
            for detections in data.values():
                dataList.append(Detection(detections))
            self.oakd_data = dataList
        except Exception as e:
            print("[ERROR] [cv_handler] Error while converting oakd data to json")
            print(f"[ERROR] {e}")
            return

    def initCameraStream(self):
        """
        Initialize the camera stream
        """
        """
        Theory: When the ID is greater than or equal to 10 for the OAK-D, this helps camsVersatile.py determine that the camera is an OAK-D. OAK-D cameras are identified after lowlight cameras,
        for example [lowlight, lowlight, OAK-D]. This then becomes a simple operation of ID/10 + number of lowlight cameras - 1 (subtract by 1 because the counting starts at 0).
        """
        topic = self.camera_topic # Which camera to use
        toSend = {}
        if not self.is_oakd:
            self.camID = int(topic[-1])  # By default the lowlight cameras are identified as 0 for forward and 1 for bottom so this matches up instantly.
        else:
            if (
                "Forward" in topic
            ):  
                self.camID = 10  # For Onyx (with one lowlight camera) this would be an ID of 1 10/10 + 1 - 1 = 1. For Graey with two lowlights, 10/10 + 2 - 1 = ID of 2.
            elif "Bottom" in topic:
                self.camID = 20  # The multiples of 10 allow the same concept with any multiple of 10 and still get the matching ID. 
            elif "Poe" in topic:
                self.camID = 30
            model = getattr(self.cv_object, "model", None) # Find the ML model set to be run for the particular mission, return None if there is no set model.
            if model is None:
                print("[INFO] No oakD model specified, defaulting to raw")
                model = "raw"
            toSend["model"] = model
        toSend["camera_ID"] = self.camID
        toSend["mode"] = "start"
        data = json.dumps(toSend) # Convert to JSON
        self.pub_cam_select.publish(data) # Publish the camera to activate

    def killCameraStream(self, killAll=False):
        """
        Kill the camera stream.

        Args:
            killAll (bool): Whether to command to kill all of the cameras streams or not. Default is set to False.
        """
        if killAll:
            data = json.dumps({["kill"]: True})
        else:
            toSend = {["camera_id"]: self.camID, ["mode"]: "stop"}
            data = json.dumps(toSend)
        self.pub_cam_select.publish(data)

    def run(self):
        """
        Run the CV script on a real-time camera stream
        """
        self.running = True

        while self.running:
            # Check if the a new frame is received
            if self.next_frame is None:
                continue
            if self.last_received == self.last_processed:
                continue
            self.last_processed = self.last_received

            # Run the CV script
            frame = self.next_frame
            try:
                ret = self.cv_object.run(frame, self.target, self.oakd_data)
            except Exception as e:
                traceback.print_exception(type(e), e, e.__traceback__)
                continue
            
            # If ret is a tuple of length 2, this that the result contains both the result, which is a dictionary of motion commands, and the visualization.
            if isinstance(ret, tuple) and len(ret) == 2:
                result, viz_img = ret
            
            # If ret is a dictionary, this means that only the result is returned. The result is a dictionary of motion commands.
            elif isinstance(ret, dict):
                result = ret
                viz_img = None
            else:
                print("[ERROR] [cv_handler] CV returned invalid type")
                continue

            # Publish the result
            self.pub_out.publish(json.dumps(result))

            if viz_img is not None:
                self.pub_viz.publish(self.br.cv2_to_imgmsg(viz_img)) # Publish the visualization of the frame as a ROS Image

    def stop(self):
        """
        Stop the CV script
        """
        self.running = False
        self.thread.join()
        # self.killCameraStream() # Commented out temporarily so we can continue watching stream post mission
        self.sub_cv.unregister()
        self.pub_viz.unregister()
        self.pub_out.unregister()
        self.pub_cam_select.unregister()
        self.closed = True


class _DummyScriptHandler:
    """
    Runs CV scripts on a video file rather than an actual camera feed
    """
    def __init__(self, file_name, cv, dummy):
        """
        Initialize the _DummyScriptHandler class

        Args:
            file_name: The file of the script to run
            cv: The CV class for the particular mission
            dummy: video file for simulating the cameara
        """
        self.file_name = file_name
        self.cv = cv

        # Get the camera topic, if not specified, use the default front camera.
        self.camera_topic = getattr(self.cv_object, "camera", None)
        if self.camera_topic is None:
            print("[WARN] No camera topic specified, using default front camera")
            self.camera_topic = "/auv/camera/videoUSBRaw0"

        self.is_oakd = False
        self.target = "main"
        self.oakd_data = None
        self.running = False
        self.closed = False

        # Capture the video frames from the video file; this is our "dummy" camera.
        self.dummy_video = dummy
        self.cap = cv2.VideoCapture(self.dummy_video)
        if not self.cap.isOpened():
            print("[ERROR] [cv_handler] Error while opening dummy video")
            return

        # Run the video in a thread, loop when it ends.
        self.thread = threading.Thread(target=self.run)

        # QUESTION: Should we post the visualization to the visualization topic?
        # Replace all instances where the string has "Raw" with "Output". For example, if the camera's name is "/auv/camera/videoUSBRaw0", 
        # it will change the name to "/auv/camera/videoUSBOutput0"
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10) 
        self.pub_out = rospy.Publisher(f"auv/cv_handler/{file_name}", String, queue_size=10)

    def run(self):
        """
        Run the CV script on the video file that simulates the camera stream.
        """
        self.running = True

        while self.running and not rospy.is_shutdown():
            ret, frame = self.cap.read() # Read each frame

            # Loop if video ends
            if not ret:
                self.cap = cv2.VideoCapture(self.dummy_video)
                continue

            # Run the CV script
            try:
                ret = self.cv_object.run(frame, self.target, self.oakd_data)
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise KeyboardInterrupt
            except Exception as e:
                traceback.print_exception()
                print(f"[ERROR] [cv_handler] Error while running CV {self.file_name}")
                continue

            # If ret is a tuple of length 2, this that the result contains both the result, which is a dictionary of motion commands, and the visualization.
            if isinstance(ret, tuple) and len(ret) == 2:
                result, viz_img = ret

            # If ret is a dictionary, this means that only the result is returned. The result is a dictionary of motion commands.
            elif isinstance(ret, dict):
                result = ret
                viz_img = None
            else:
                print("[ERROR] [cv_handler] CV returned invalid type")
                continue

            # Publish the dictionary of motion commands as JSON data.
            self.pub_out.publish(json.dumps(result))

            # Publish the visualized frame as a ROS image, if the visualized frame is avaliable.
            if viz_img is not None:
                self.pub_viz.publish(self.br.cv2_to_imgmsg(viz_img))

    def stop(self):
        """
        Stop the simulated camera feed
        """
        self.running = False
        self.thread.join()

        self.pub_viz.unregister()
        self.pub_out.unregister()
        self.closed = True


class Detection:
    """
    Takes data from the detections topic, and for each detection, which is a list, it gives names describing what each position means:

    label = 1st position (0), confidence = 2nd position (1), xmin = 3rd, xmax = 4th, ymin = 5th, ymax = 6th
    """
    def __init__(self, data):
        self.label = data[0]
        self.confidence = data[1]
        self.xmin = data[2]
        self.xmax = data[3]
        self.ymin = data[4]
        self.ymax = data[5]


if __name__ == "__main__":
    # For testing purposes. 
    # NOTE: Requires camsVersatile.py to run beforehand.

    def dummy_callback(msg):
        print(f"[INFO] received: {msg.data}")

    file_name = "buoy_cv"

    cv = CVHandler()
    rospy.init_node("CV_Handler_TEST", anonymous = True)
    try:
        cv.start_cv(file_name, dummy_callback)
    #try:
         # Generic module file path: auv.cv.file_name
         #module = importlib.import_module(f"auv.cv.{file_name}")
    #except Exception as e:
        # print("[ERROR] [cv_handler] Error while importing CV module from file name")
        # print(f"[ERROR] {e}")

    #cv_class = getattr(module, "CV", None)
    #if cv_class is None:
        # print("[ERROR] [cv_handler] No CV class found in file, check the file name and file content")

    except rospy.KeyboardInterrupt:
        cv.stop_cv(file_name)
        
        
