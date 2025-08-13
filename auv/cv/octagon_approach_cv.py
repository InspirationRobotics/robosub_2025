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
    model = "poles" # Change later once data is collected for the platform

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
        self.adjust_search_time = None
        self.search_counter = 0
        self.search_stage_one = None # store time
        self.search_stage_two = None # store time
        self.stage_two_end = False
        self.adjust_count = 0
        self.end = False
        self.prev_offset = None
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
            yaw = 0.8
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

        # Extract octagon detection
        if detections is None:
            detections = []
        if len(detections) == 0 and self.prev_detected == False:
            self.state = "search"
        

        detected_list = []
        detection_confidence = 0.65
        for det in detections:
            if det.label == "octagon":
                print(f"[DEBUG] Detected {det.label} with confidence {det.confidence}")
                if det.confidence > detection_confidence:
                    detected_list.append(det)

        # select the highest confidence octagon deteciton if multiple
        offset = None
        if len(detected_list)==0:
            offset = None
        elif len(detected_list)==1:
            self.prev_time = time.time()
            detection = detected_list[0]
            target_x = (detection.xmin + detection.xmax) / 2
            target_y = (detection.ymin + detection.ymax) / 2
            offset = target_x - self.x_midpoint
            self.prev_detected = True
            self.prev_offset = offset
            self.state = "approach"
            print(f"[DEBUG] target_x is {target_x}")
        else:  # when there are more than one octagon detection
            # Select the detection with the highest confidence
            self.prev_time = time.time()
            detection = max(detected_list, key=lambda det: det.confidence)
            target_x = (detection.xmin + detection.xmax) / 2
            target_y = (detection.ymin + detection.ymax) / 2
            offset = target_x - self.x_midpoint
            detection_confidence = detection.confidence
            self.prev_detected = True
            self.prev_offset = offset
            self.state = "approach"
            print(f"[DEBUG] Multiple octagons detected. Using highest confidence detection: {detection_confidence}")
            print(f"[DEBUG] target_x is {target_x}, target_y is {target_y}")


        if self.state == "search":
            if self.search_counter<=2:
                if self.search_stage_one is None:
                    print("[DEBUG] Searching in stage 1")
                    self.search_stage_one = time.time()
                if time.time()-self.search_stage_one > 10:
                    print(f"[DEBUG] Searching in stage one, counter is {self.search_counter}")
                    self.search_counter += 1
                    self.search_stage_one = time.time()
                if self.search_counter%2==1:
                    yaw = 1
                else:
                    yaw = -1
            else:
                if self.search_stage_two is None:
                    print(f"[DEBUG] Searching in stage two")
                    self.search_stage_two = time.time()
                
                if self.prev_offset is None:
                    yaw = 1
                elif self.prev_offset > 0 :
                    yaw= 1
                elif self.prev_offset < 0:
                    yaw = -1

        if self.state == "approach":
            if not self.stage_two_end:
                self.stage_two_end = True
                self.search_stage_two=time.time()
            print("[DEBUG] Approaching now!")
            print(f"[INFO] offset is {offset}")
            forward, yaw = self.smart_approach(offset)
            
        # Check Ending
        if self.state=="search" and self.prev_detected is None and self.search_stage_two is not None and time.time()-self.search_stage_two > 30:
           # when we had went through stage one and time out for 30 seconds
           print(f"[DEBUG] time out in searching")
           self.end = True

        # handle adjust search
        if self.state=="search" and self.prev_detected: # you are in adjust search mode when you had detection but in search mode again
            if time.time() - self.adjust_search_time > 15:
                self.end = True

        if self.state=="approach" and (offset is None) and self.prev_detected == True:
            if time.time() - self.prev_time > 2:
                if self.adjust_count <2:  # adjust to search again
                    print(f"[DEBUG] adjust and search")
                    self.state = "search"
                    self.adjust_count += 1
                    self.adjust_search_time = time.time()

                else:
                    print(f"[DEBUG] Ending with prev detected: {self.prev_detected}")
                    self.end = True
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical" : vertical, "end": self.end}, frame
