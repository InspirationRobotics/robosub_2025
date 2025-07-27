"""
Octagon Approach CV. Finds the octagon, and approaches the octagon until it can no longer see it.
"""

import time

import cv2
import numpy as np
import time
import queue

class CV:
    """
    Octagon Approach CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawForward"
    model = "everything" # Change later once data is collected for the platform

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """
        # Camera to get the camera stream from.
        self.camera = "/auv/camera/videoOAKdRawForward"
        self.model = "everything" # Change later once data is collected for the platform

        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0]/2
        self.y_midpoint = self.shape[1]/2

        self.tolerance = 120 # Pixels

        self.prev_detected = False
        self.state = "search"

        self.last_yaw = 0
        self.yaw_time_search = 10
        self.search_counter = 1
        self.search_stage_one = None # store time
        self.search_stage_two = None # store time
        self.end = False
        self.prev_time = time.time()
        
        print("[INFO] Octagon Approach CV Initialization")
    
    def smart_approach(self, offset):
        """Function to properly yaw and move forward"""
        forward = 0
        yaw = 0
        if offset is None or abs(offset) < self.tolerance:
            yaw = 0
            forward = 2.0
        elif offset > 0:
            yaw = 0.75
        elif offset < 0:
            yaw = -0.75
        
        return forward, yaw

    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target: This can be any type of information, for example, the object to look for
            detections: This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {motion commands/flags for servos and other indication flags}, visualized frame
        """

        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        target_x = None
        target_y = None

        # Extract octagon detection
        if detections is None:
            detections = []
        if len(detections) == 0 and self.prev_detected == False:
            self.state = "search"
        

        detected_list = []
        detection_confidence = 0.65
        for det in detections:
            if det.label == "octagon":
                if det.confidence > detection_confidence:
                    detected_list.append(det)

        # select the highest confidence octagon deteciton if multiple
        if len(detected_list)==0:
            offset = None
        elif len(detected_list)==1:
            detection = detected_list[0]
            target_x = (detection.xmin + detection.xmax) / 2
            target_y = (detection.ymin + detection.ymax) / 2
            offset = target_x - self.x_midpoint
            detection_confidence = detection.confidence
            self.prev_detected = True
            self.state = "approach"
            print(f"[DEBUG] target_x is {target_x}")
            print(f"[DEBUG] Detected octagon with confidence {detection.confidence}")
        else:  # when there are more than one octagon detection
            # Select the detection with the highest confidence
            detection = max(detected_list, key=lambda det: det.confidence)
            target_x = (detection.xmin + detection.xmax) / 2
            target_y = (detection.ymin + detection.ymax) / 2
            offset = target_x - self.x_midpoint
            detection_confidence = detection.confidence
            self.prev_detected = True
            self.state = "approach"
            print(f"[DEBUG] Multiple octagons detected. Using highest confidence detection: {detection_confidence}")
            print(f"[DEBUG] target_x is {target_x}, target_y is {target_y}")


        if self.state == "search":
            print("[DEBUG] Searching")
            if self.search_counter<2:
                if self.search_stage_one is None:
                    self.search_stage_one = time.time()
                if time.time()-self.search_stage_one > 3:
                    self.search_counter += 1
                    self.search_stage_one = time.time()
                if self.search_counter%2==1:
                    yaw = 1
                else:
                    yaw = -1
            else:
                if self.search_stage_two is None:
                    self.search_stage_two = time.time()
                yaw = 1

        if self.state == "approach":
            print("[DEBUG] Approaching now!")
            # if we had detection but lost it
            if self.prev_detected and target_x is None:
                target_x = self.x_midpoint
            
            print(target_x)
            forward, yaw = self.smart_approach(target_x)
            self.prev_time = time.time()
            
        # Check Ending 
        if self.state=="search" and self.search_stage_two is not None and time.time()-self.search_stage_two > 30:
           # when we went through stage one and time out for 30 seconds
           print(f"[DEBUG] time out in searching")
           self.end = True

        if self.state=="appraoch" and (offset is None) and self.prev_detected == True:
            if time.time() - self.prev_time > 3:
                print(f"[DEBUG] Ending with prev detected: {self.prev_detected}, detection list: {self.detection_list}")
                self.end = True
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical" : vertical, "end": self.end}, frame
