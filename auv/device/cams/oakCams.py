"""
Handles camera streams for the OAK-D cameras -- this file also runs models on the raw data from these camera streams
"""

import os
import platform
import signal
import sys
import threading
import time

import cv2
import json
import glob
import numpy as np
import depthai as dai
from auv.utils.img_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from auv.device.cams import pyfakewebcam # For simulating a virtual camera feed by writing individual frames to a V4L2 video device
from auv.utils import deviceHelper # For obtaining configuration (path) of each device (camera)


class oakCamera:
    """
    Class for handling the camera streams for the OAK-Ds
    """
    def __init__(self, rospy, id, mxid, newDevice):
        """
        Initializes the oakCamera class

        Args:
            rospy: ROS context (provided outside of class)
            id: the path of the camera (configuration)
            mxid: the unique id of the OAK-D camera (this is used for identifying the DepthAI device and using its associated resources)
            newDevice: the path to the V4L2 virtual video device 
        """
        # Image width and height
        self.IMG_W = 640
        self.IMG_H = 480
        
        self.rospy = rospy
        self.id = id
        self.mxid = mxid

        # Change the name of the camera to represent the way it faces (forward, bottom). "PoE" is also a valid name, and represents "Power Over Ethernet"
        self.name = self.mxidToName(mxid)
        self.frame = None

        # To allow change in message types from ROS Image types to CV type "mat" images
        self.br = CvBridge()

        # Node cycle rate (Hz)
        self.loop_rate = self.rospy.Rate(30)

        self.isKilled = True
        self.modelPath = -1

        # Establish a fake web camera feed
        self.fake = pyfakewebcam.FakeWebcam(newDevice, self.IMG_W, self.IMG_H)

        # Relavent subscribers and publishers
        self.pubFrame = self.rospy.Publisher(f"/auv/camera/videoOAKdRaw{self.name}", Image, queue_size=10)
        self.pubData = self.rospy.Publisher(f"/auv/camera/videoOAKdData{self.name}", String, queue_size=10)
        self.rospy.Subscriber(f"/auv/camera/videoOAKdModel{self.name}", String, self.callbackModel)
        self.rospy.Subscriber(f"/auv/camera/videoOAKdOutput{self.name}", Image, self.callbackMain)

        self.time = time.time()

        # Denote the path where the Oak-D camera stream is avaliable
        print(f"Camera ID {str(id)}: Oak-D {self.name} is available at {newDevice}")

    def createPipeline(self, modelPath="raw", confidence=0.5):
        """
        Creates a camera pipeline (path) getting the data from the OAK-Ds to the model path and confidence threshold (confidence for each detection). "Raw" represents 
        a frame that is directly from the camera, i.e. no data manipulation has been performed on the frame. 

        Args:
            modelPath: path to the model directory or "raw" for raw camera feed (there is a model for each mission; see auv/device/cams/models)
            confidence: the confidence rating of any detections
        """
        """
        DepthAI pipelines allow models to run on data directly from the camera feed. There are "nodes" which define each step of the process. These nodes are configured individually.

        The camera node (camRgb) is the node that captures color frames from the OAK-D. In this case, the camera node feeds its data directly to the model node.

        The input node (this is a XLinkIn node, in the code it is called controlIn), allows for changes of the settings of the camera.

        The model node (assuming the there is a model), is the model that runs on the data. It outputs its results.

        The output node is what takes the results of the model and transmits it back to the host (user). These are XLinkOut nodes, and in this case, we have output nodes,
        one for just the raw camera data, and one for the results of the model.

        These nodes are "linked" to one another. This means that the output of the camera node can be linked to the output node's input, or to the model's input, as an example. These linkages
        are what connect the pipeline to make it continous.

        In summary, the pipeline goes something like: input node (controlIn) -> camera node (camRGB) -> model node (detectionNetwork) -> output node (nnOut)
                                                                                                    -> output node (xoutRGB)
        """
        self.modelPath = modelPath

        # Create a simple pipeline to create a camera feed for the raw data
        if self.modelPath == "raw":
            pipeline = dai.Pipeline() # Create a pipeline
            xoutRgb = pipeline.createXLinkOut() # Create an output node
            xoutRgb.setStreamName("rgb") # Set the stream name so that the output stream is an RGB
            controlIn = pipeline.create(dai.node.XLinkIn) # Create an input node
            controlIn.setStreamName("control") # Set the stream name to control so that the input stream is a camera control input
            camRgb = pipeline.createColorCamera() # Create a ColorCamera node to represent the camera
            # Note that these "nodes" are DepthAI nodes, which are different processing units/data flows relavent to DepthAI, not ROS nodes
            
            # Set the resolution of the Oak-D camera based on the direction it is facing
            if self.name == "Forward":
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
            else:
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            # Setting the camera's ISP (Image Signal Processor -- this processes raw image data and converts it into a usable format) scale 
            camRgb.setIspScale(2, 3) # Note that 480 (the width of the frame) is 2/3 of 640 (the height of the frame)
            
            # ***Previewing refers to viewing the real-time continous camera feed
            camRgb.setPreviewSize(self.IMG_W, self.IMG_H) # Setting the size of the frame
            camRgb.setInterleaved(False) # False means that there will be three color channels instead of one which would blend the three (R, G, B) channels together
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB) # Sets the color order (arrangement of color channels in each pixel of the image) to RGB
            camRgb.preview.link(xoutRgb.input) # Link the camera preview to the RGB output node
            controlIn.out.link(camRgb.inputControl) # Link the control input node to the camera control input
            
            # Create a device using the MXID device information and pipeline
            device_info = dai.DeviceInfo(self.mxid)
            self.device = dai.Device(pipeline, device_info)

            # Create an input queue for camera control
            controlQueue = self.device.getInputQueue("control")

            # Set up the auto-focus trigger and disable continuous auto-focus
            ctrl = dai.CameraControl()
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
            ctrl.setAutoFocusTrigger()
            controlQueue.send(ctrl)
        
        # If the model path is not "raw", a model is being run on the data (hence using a model-based pipeline)
        else:
            # Find the JSON (config for the model) and blob (model containing the neural network) file associated with the model path
            jsonFile = glob.glob(f"{self.modelPath}*.json")[0]
            blobFile = glob.glob(f"{self.modelPath}*.blob")[0]
            print("Found model; creating pipeline")

            # Open the JSON file and load the data
            jsonFile = open(jsonFile)
            data = json.load(jsonFile)
            NN_params = data["nn_config"]["NN_specific_metadata"]
            nnBlobPath = blobFile
            classAmt = NN_params["classes"]
            anchors = NN_params["anchors"]
            anchorMasks = NN_params["anchor_masks"]
            self.labelMap = data["mappings"]["labels"]

            # Create the model-based pipeline
            pipeline = dai.Pipeline()

            # Define the sources and outputs
            camRgb = pipeline.create(dai.node.ColorCamera)
            detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
            xoutRgb = pipeline.create(dai.node.XLinkOut)
            nnOut = pipeline.create(dai.node.XLinkOut)

            # Setting the correct stream names
            xoutRgb.setStreamName("rgb")
            nnOut.setStreamName("nn")

            # Set the properties of the camera feed (size, resolution, # of channels -- [Interleaved == True is one channel, Interleaved == False is three channels] , pixel color order, FPS)
            camRgb.setPreviewSize(self.IMG_W, self.IMG_H)
            if self.name == "Forward":
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
            else:
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            # camRgb.setIspScale(2, 3)
            # camRgb.setPreviewSize(640, 480)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            camRgb.setFps(30)

            # Neural network-specific settings
            detectionNetwork.setConfidenceThreshold(confidence) # Detections having a confidence below this confidence value will be ignored
            detectionNetwork.setNumClasses(classAmt) # Specify the number of classes the detection network should recognize
            detectionNetwork.setCoordinateSize(4) # Define the size of the coordinates to represent each detection (there are 4: xmin, ymin, xmax, ymax)
            detectionNetwork.setAnchors(anchors) # Set the anchors for the network. Anchors are used in determining the default sizes and shapes of bounding boxes
            detectionNetwork.setAnchorMasks(anchorMasks) # Set the anchor masks for the network. Anchor masks help focus on different aspects of an image for different anchors. 
            detectionNetwork.setIouThreshold(0.5) # Set the Intersection over Union (IoU) threshold for filtering overlapping detections
            detectionNetwork.setBlobPath(nnBlobPath) # Set the path for the blob file that contains the Neural Network to be used
            # detectionNetwork.setNumInferenceThreads(2) needed?
            detectionNetwork.input.setBlocking(False) # Make the input queue non-blocking so the input stream does not wait for a full queue to continue running the stream

            # Linking the different components of the pipeline
            camRgb.preview.link(detectionNetwork.input) # Link the color camera (RGB) to the input of the detection network
            
            
            # This effectively allows for two feeds, one for performing manipulations through the NN that the a feed will show,
            # while another feed will show just the RGB feed of the raw OAK-D camera stream 
            detectionNetwork.passthrough.link(xoutRgb.input) # Link a passthrough (allows for the data to pass through the NN (neural network) link unaltered) to the color camera
            detectionNetwork.out.link(nnOut.input) # Link the NN results to the NN output (shows the NN labeling results)

            # Initializing Oak-D with pipeline
            device_info = dai.DeviceInfo(self.mxid)
            self.device = dai.Device(pipeline, device_info)

        self.isKilled = False

    def mxidToName(self, mxid):
        """
        Converts the name of the OAK-D device from its MXID to a convenient name, based on its location on the sub; if the sub's forward 
        facing OAK-D has an MXID of "1ADFD103903" (this is NOT the real MXID), and that matches the MXID of the camera given in the argument, then
        that camera will be given a name of "forward".
        
        The three possible conversion names: 
            "fowardOak" -> "Forward"
            "bottomOak" -> "Bottom"
            "poeOak" -> "Poe" (stands for Powered over Ethernet)

        Args:
            mxid (str): ID of the OAK-D device in question (this is not "forwardOak", but a long string of numbers and letters that denotes the actual DepthAI ID)
        """
        if mxid == deviceHelper.dataFromConfig("forwardOak"):
            return "Forward"
        elif mxid == deviceHelper.dataFromConfig("bottomOak"):
            return "Bottom"
        elif mxid == deviceHelper.dataFromConfig("poeOak"):
            return "Poe"
        else:
            return "Unknown"

    def callbackMain(self, msg):
        """
        Callback function for writing a frame to the V4L2 virtual video device (creating a camera feed)

        Args:
            msg: ROS Image to be written to the V4L2
        """
        # If the camera feed has been killed
        if self.isKilled:
            return
        self.time = time.time()
        # Write the frame to the V4L2 by converting the ROS Image type into a CV2 ("mat") image type
        self.sendFakeFrame(self.br.imgmsg_to_cv2(msg))

    def sendFakeFrame(self, msg):
        """
        Method for writing a frame to the V4L2

        Args:
            msg (numpy.ndarray): Frame to be written to the V4L2 (this will be in BGR)
        """
        try:
            self.frame = cv2.cvtColor(msg, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB color format
            self.fake.schedule_frame(self.frame) # Write the frame to the V4L2 video device
        except Exception as e:
            print(f"Camera {str(self.id)} Output Error, make sure running in correct python")
            print(e)

    def modelSelect(self, modelName: str):
        """
        Navigates to the file that contains the model to run

        Args:
            modelName (str): the name of the model (this can either be the path or just the model name)
        
        Returns:
            modelPath (str): the path to the model
        """
        root = os.path.normpath("/home/inspiration/auv/auv/device/cams/models/")

        if modelName == "raw":
            modelPath = "raw"
        
        # If there is a line breaker "/" in the model name, use the model name as the direct path
        elif os.path.sep in modelName:
            # If the model name does not end with a "/" (path separator)
            if not modelName.endswith(os.path.sep):
                modelName += os.path.sep
            modelPath = modelName # Set the model path to the model name
        # Otherwise, put the model name at the end of the root path, and make that the model path
        else:
            modelPath = f"{root}/{modelName}Model/"
        
        # Handle if the model path is invalid
        if not modelPath == "raw" and not os.path.isdir(modelPath):
            print(f"Invalid model name, {modelPath}")
            print(f"Available models: {glob.glob(os.path.join(root, '*'))}")
            return

        if self.modelPath == modelPath:
            print("Model already running...")
            return

        print(f"Switching {self.name} oakD to {modelName} model")
        return modelPath

    def callbackModel(self, msg, debug=False):
        """
        Function that is called to run the particular NN model

        Args:
            msg: ROS string that names the model to run
            debug: flag that, when True, stores the entire ROS string message for debugging purposes
        """
        if self.isKilled:
            return
        if debug:
            modelName = msg
        else:
            modelName = msg.data # Get only the "data" part of the message -- this is the actual payload, and in this case is the name of the model (or path of the model)
        if self.modelSelect(modelName) == None:
            return
        self.kill() # Kill all other models/streams than may be running
        self.start(modelName) # Start the model

    def runner(self):
        """
        Deals with running a ML (neural network) model on the OAK-D data. This is what actually retrieves the results from the data processing by the model.
        """
        cam = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False) # Get the RGB camera output (maxSize == 4 represents the largest queue possible)
        
        # If there is an actual model to be run, then get the device's neural network output queue
        if self.modelPath != "raw":
            qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        while not self.rospy.is_shutdown() and not self.isKilled:
            try:
                # Get an RGB frame from the camera queue
                frame1 = cam.get().getCvFrame()
                if self.modelPath != "raw":
                    # Retrieve a detection packet from the detection queue
                    inDet = qDet.get()
                    # Get the detections from the detection packet
                    detections = inDet.detections
                    dataToSend = {} # Dictionary for holding the detections data

                    for i, detection in enumerate(detections):
                        # Denormalize bounding boxes around the detections
                        x1 = int(detection.xmin * self.IMG_W)
                        x2 = int(detection.xmax * self.IMG_W)
                        y1 = int(detection.ymin * self.IMG_H)
                        y2 = int(detection.ymax * self.IMG_H)

                        # Try to get the label of the detection from the label map, otherwise just use the detection label
                        try:
                            label = self.labelMap[detection.label]
                        except:
                            label = detection.label
                        
                        # Put the detection label and the confidence on the screen next to the detection
                        cv2.putText(frame1, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(
                            frame1,
                            f"{detection.confidence * 100:.2f}",
                            (x1 + 10, y1 + 35),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )
                        # Draw the bounding box that surrounds the detection
                        cv2.rectangle(frame1, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)  # change for multiple colors?

                        # Send the detection data to the dictionary
                        dataToSend[str(i)] = [label, round(detection.confidence * 100, 3), x1, x2, y1, y2]

                    # Publish the detections data to the right ROS topic by converting the dataToSend dictionary into a JSON string format
                    self.pubData.publish(str(json.dumps(dataToSend)))
                
                # Publish the frame that was used as a ROS Image in the correct topic
                msg = self.br.cv2_to_imgmsg(frame1)
                self.pubFrame.publish(msg)

                # If three seconds have elapsed since a new CV output frame was received, then move to camera view (simulated webcam)
                if time.time() - self.time > 3:  
                    self.sendFakeFrame(frame1)
                pass
            except Exception as e:
                print(f"Camera {str(self.id)} Input Error")
                print(e)
            self.loop_rate.sleep()

    def kill(self):
        """
        To kill camera streams by joining threads and deleting the devices (the virtual camera stream with the pipeline)
        """
        if self.isKilled:
            return
        self.isKilled = True
        self.rospy.loginfo(f"Killing Camera {str(self.id)} Stream...")
        self.oakThread.join()
        del self.device
        self.modelPath = -1
        self.rospy.loginfo(f"Killed Camera {str(self.id)} Stream...")
        pass  # todo

    def start(self, modelPath="raw"):
        """
        For starting a model to run on the OAK-D data. This function creates the Depth AI pipeline based on the model path, 
        and begins a thread to run the model.

        Args:
            modelPath (str): the model name (either just the name like "gate3" or the actual path)
        """
        if not self.isKilled:
            return
        self.createPipeline(self.modelSelect(modelPath)) # Create the pipeline based on the model path
        self.isKilled = False
        self.rospy.loginfo(f"Starting Camera {str(self.id)} Stream...")
        self.oakThread = threading.Timer(0, self.runner)
        self.oakThread.daemon = True
        self.oakThread.start()