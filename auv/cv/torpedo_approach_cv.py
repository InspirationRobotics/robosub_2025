"""
Torpedo Approach CV. Finds the Torpedo, and approaches the Torpedo until it can no longer see it.
"""

import cv2
import numpy as np
import time
from typing import Tuple
import queue

class CV:
    """
    Torpedo Approach CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    def __init__(self, **config: dict): # ALWAYS add type hints to parameters and return types. It makes the code much easier to read for someone who's never seen it before.
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config (dict): Dictionary that contains the configuration of the devices on the sub.
        """
        # Camera to get the camera stream from.
        self.camera = "/auv/camera/videoOAKdRawForward"
        self.focal_length = 286.2  # focal length of OAK-D WIDE on Onyx | Unit: pixel | https://en.wikipedia.org/wiki/Pinhole_camera_model | we resize the frame to 640*480, focal length in pixel will change
        self.model = "everything" # Change later once data is collected for the platform

        self.config = config
        self.shape = (640, 480) # maybe self.frame or self.cam_frame would be a better var name
        self.framecenter_x = self.shape[0]/2
        self.framecenter_y = self.shape[1]/2

        self.tolerance = 120 # Pixels

        self.state = "search"
        self.end = False
        self.estimated_distance = None # Estimated distance away from torpedo poster
        self.prev_detected = False
        self.prev_offset = None
        self.prev_time = time.time()
        self.start_time = self.prev_time
        print("[INFO] Torpedo Approach CV Initialization")
    
    def smart_approach(self, offset: int) -> Tuple[float, float]:
        """Function to properly yaw and move forward
        
        Args:
            offset (int): Difference (in pixels) between frame x-midpoint and bounding box x-midpoint
        
        Returns:
            forward (float): Forward PWM (between -5 and 5)
            yaw (float): Yaw PWM (between -5 and 5)"""
        forward = 0
        yaw = 0
        
        # If the detection is centered or there is none, center it
        if offset is None or abs(offset) < self.tolerance:
            yaw = 0
            forward = 2.0
        
        # Yaw right if detection is too far right
        elif offset > 0:
            yaw = 0.8
        
        # Yaw left if detection is too far left
        elif offset < 0:
            yaw = -0.8
        
        return forward, yaw

    def process_detection(self, detection):
        """Find offset, estimated distance when there is a detection"""
        # Calculate offset
        self.target_x = (detection.xmin + detection.xmax) / 2
        self.target_y = (detection.ymin + detection.ymax) / 2
        self.curr_offset = self.target_x - self.framecenter_x # These var names could use some work. Both self.target_x and self.framecenter_x are technically midpoints - the former of the detection bounding box, the latter of the frame
        self.prev_detected = True
        self.prev_offset = self.curr_offset

        # Estimate distance away form torpdeo poster
        height = abs(detection.ymin - detection.ymax)
        target_height = 0.6096 # unit: met
        self.estimated_distance = (target_height * self.focal_length) / height

        self.state = "approach"
        print(f"[DEBUG] self.target_x is {self.target_x}, self.target_y is {self.target_y}") # Why are we including the self.target_y here but not if there's one detection?

    
    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame(cvFrame): The frame from the camera stream
            target: This can be any type of information, for example, the object to look for
            detections: This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {motion commands/flags for servos and other indication flags}, visualized frame
        """
        print("Reached run function")
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        self.target_x = None
        self.target_y = None

        # Configure search state if there aren't detections
        if detections is None:
            detections = []
        
        # Utilize Torpedo detections with at least 55% confidence
        detected_list = []
        detection_confidence = 0.55
        for det in detections:
            if "torpedo" in det.label:
                print(f"[DEBUG] Detected {det.label} with confidence {det.confidence}")
                if det.confidence > detection_confidence:
                    detected_list.append(det)

        self.curr_offset = None
        if len(detected_list)==0:
            self.curr_offset = None
            print(f"[INFO] No detection!")
        elif len(detected_list)==1:
            self.prev_time = time.time()
            detection = detected_list[0]
            self.process_detection(detection=detection)
        else: 
            # select the highest confidence Torpedo deteciton if multiple
            self.prev_time = time.time()
            detection = max(detected_list, key=lambda det: det.confidence)
            detection_confidence = detection.confidence
            print(f"[DEBUG] Multiple Torpedos detected. Using highest confidence detection: {detection_confidence}")
            self.process_detection(detection=detection)

        if self.state == "approach":
            print("[DEBUG] Approaching now!")
            print(f"[INFO] offset is {self.curr_offset}")
            forward, yaw = self.smart_approach(self.curr_offset)
            
        # Check Endings

        # Ending 1: Total Misison Time Out
        if time.time() - self.start_time > 120: # Arbitary set to 120 s for now, need to see what's the estimated mission time including searching
            self.end = True
        
        # Ending 2: Approach the torpedo until a certain distance, stop at that distance and launch torpedo
        if self.state=="approach":
            if self.estimated_distance < 1.5:  # put 1.5 m distance for now, TODO check distance estimation accuracy and find the desire distance away
                self.end = True
        
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"state": self.state, "prev_offset": self.prev_offset,"lateral": lateral, "forward": forward, "yaw": yaw, "vertical" : vertical, "end": self.end}, frame
