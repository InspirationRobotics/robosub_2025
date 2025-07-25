"""
Octagon Approach CV. Finds the octagon, and approaches the octagon until it can no longer see it.
"""

import time

import cv2
import numpy as np
import time

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

        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0]/2
        self.y_midpoint = self.shape[1]/2

        self.tolerance = 120 # Pixels

        self.prev_detected = False
        self.state = None

        self.start_time = None
        self.last_yaw = 0
        self.yaw_time_search = 2
        self.end = False
        self.prev_time = time.time()
        
        print("[INFO] Octagon Approach CV Initialization")

    def smart_approach(self, detection_x):
        """Function to properly yaw and move forward"""
        forward = 0
        # Yaw cannot go below 0.5
        if detection_x < self.x_midpoint - self.tolerance:
            yaw = -0.75
        elif detection_x > self.x_midpoint + self.tolerance:
            yaw = 0.75
        else:
            yaw = 0
            forward = 2.0

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

        # Find the bin if no detection is found
        # Align with the bin and move forward (through strafe should be fine)
        # If we have lost sight of the bin, then end

        # So we do not get a NoneType error
        if detections is None:
            detections = []
        if len(detections) == 0 and self.prev_detected == False:
            self.state = "search"
        
        if len(detections) == 0 and self.prev_detected == True:
            if time.time() - self.prev_time < 2:
                self.state = None
                forward = 0
            else:
                self.end = True

        if len(detections) >= 1:
            if len(detections) == 1:
                for detection in detections:
                    print(f"[DEBUG] Detection confidence: {detection.confidence}")
                    if detection.confidence > 0.65:
                        target_x = (detection.xmin + detection.xmax) / 2
                        target_y = (detection.ymin + detection.ymax) / 2
                    # Might need to do something in an elif or else here
                    else:
                        target_x = None
                
            elif len(detections) > 1:
                # Target the detection with the highest confidence. The detection targeted
                # doesn't matter since this is a localization script, not a mission script

                # Increased required confidence to 0.65 to account for multiple false positives without
                # true positive
                detection_confidence = 0.65
                for detection in detections:
                    if detection.confidence > detection_confidence and detection.confidence > 0.65:
                        target_x = (detection.xmin + detection.xmax) / 2
                        target_y = (detection.ymin + detection.ymax) / 2
                        detection_confidence = detection.confidence

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
