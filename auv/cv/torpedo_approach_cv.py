"""
Torpedo Approach CV. Finds the Torpedo, and approaches and circumnavigates to face perpendicular to the Torpedo.
"""

import time

import cv2
import numpy as np
import time
import queue

class CV:
    """
    Torpedo Approach CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

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
        "TODO Confirm the tolerance for the Torpedo poster "
        self.tolerance = 120 # Pixels

        self.prev_detected = False
        self.state = None

        self.start_time = time.time()
        self.last_yaw = 0
        self.yaw_time_search = 2
        self.end = False
        self.reached = False
        self.prev_detect_timestamp = None
        self.prev_time = time.time()
        self.detection_list = [] # Queue to store the detections(bool) from the ML model
        print("[INFO] Torpedo Approach CV Initialization")

    
    def update_list(self, value):
        """
        Update the detection list with the new value.

        Args:
            value: The value to update the detection list with.
        """
        if len(self.detection_list) >= 10:
            self.detection_list.pop(0)
        self.detection_list.append(value)
    
    def smart_approach(self, offset: int) -> Tuple[float, float]:
        """Function to properly yaw and move forward"""
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

        # Find the torpedo if no detection is found
        # Align with the torpedo and move forward (through strafe and yaw should be fine)
        # If we have lost sight of the bin, then end

        # So we do not get a NoneType error
        if detections is None:
            detections = []
        if len(detections) == 0 and self.prev_detected == False:
            self.state = "search"
        
        detected_list = []
        detection_confidence = 0.65
        for detection in detections:
            if detection.label == "torpedo_target":
                if detection.confidence > detection_confidence:
                    self.update_list(1)
                    print(f"[DEBUG] Detected torpedo with confidence {detection.confidence}")
                    target_x = (detection.xmin + detection.xmax) / 2
                    target_y = (detection.ymin + detection.ymax) / 2
                    detection_confidence = detection.confidence
        else:
            self.update_list(0)
            target_x = None
            target_y = None

        if target_x is None:
            self.state = "search"
        elif target_x is not None and target_y is not None:
            self.prev_detected = True
            self.state = "approach"

        if self.state == "search":
            # Scrap search grid in favor of circular search
            yaw = 1

        if self.state == "approach":
            print("[DEBUG] Approaching now!")
            print(target_x)
            forward, yaw = self.smart_approach(target_x)
            self.prev_time = time.time()
            
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical" : vertical, "end": self.end}, frame
