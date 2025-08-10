"""
Bin Approach CV. Finds the Bin, and approaches the Bin until it can no longer see it.
"""

import cv2
import numpy as np
import time
from typing import Tuple
import queue

class CV:
    """
    Bin Approach CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawForward"
    model = "everything" # Change later once data is collected for the platform

    def __init__(self, **config: dict): # ALWAYS add type hints to parameters and return types. It makes the code much easier to read for someone who's never seen it before.
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config (dict): Dictionary that contains the configuration of the devices on the sub.
        """
        # Camera to get the camera stream from.
        self.camera = "/auv/camera/videoOAKdRawForward"
        self.model = "bins" # Change later once data is collected for the platform

        self.config = config
        self.shape = (640, 480) # maybe self.frame or self.cam_frame would be a better var name
        self.framecenter_x = self.shape[0]/2
        self.framecenter_y = self.shape[1]/2

        self.tolerance = 120 # Pixels

        self.prev_detected = False
        self.state = "search"

        # Leonard: NOTE 
        # Most var names (except the last three) don't immediately give away their function, although when I looked
        # at the code it made sense. Calling a switch back from approach to search an "adjust" is a little bit quirky,
        # there may be a better name for it. A big thing in industry is not just functional, but also readable code. 
        # Code isn't any good if the next person coming along has no idea what's happening.  
        
        # Chase: Solved, change var name to switch_back_time
        self.switch_back_time = None
        self.stage_two_end = False
        self.switch_count = 0
        self.end = False
        self.prev_offset = None
        self.prev_time = time.time()
        self.start_time = self.prev_time
        print("[INFO] Bin Approach CV Initialization")
    
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

    def process_detection_offset(self, detection):
        """A function to calculate offset if there is detection, set state to approach"""
        self.target_x = (detection.xmin + detection.xmax) / 2
        self.target_y = (detection.ymin + detection.ymax) / 2
        self.curr_offset = self.target_x - self.framecenter_x # These var names could use some work. Both self.target_x and self.framecenter_x are technically midpoints - the former of the detection bounding box, the latter of the frame
        self.prev_detected = True
        self.prev_offset = self.curr_offset
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

        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        self.target_x = None
        self.target_y = None

        # Configure search state if there aren't detections
        if detections is None:
            detections = []
        
        # Utilize bin detections with at least 55% confidence
        detected_list = []
        detection_confidence = 0.55
        for det in detections:
            if "bin" in det.label:
                print(f"[DEBUG] Detected {det.label} with confidence {det.confidence}")
                if det.confidence > detection_confidence:
                    detected_list.append(det)

        self.curr_offset = None
        if len(detected_list)==0:
            self.curr_offset = None
        elif len(detected_list)==1:
            self.prev_time = time.time()
            detection = detected_list[0]
            self.process_detection_offset(detection=detection)
        else: 
            # select the highest confidence Bin deteciton if multiple
            self.prev_time = time.time()
            detection = max(detected_list, key=lambda det: det.confidence)
            detection_confidence = detection.confidence
            print(f"[DEBUG] Multiple Bins detected. Using highest confidence detection: {detection_confidence}")
            self.process_detection_offset(detection=detection)

        if self.state == "approach":
            print("[DEBUG] Approaching now!")
            print(f"[INFO] offset is {self.curr_offset}")
            forward, yaw = self.smart_approach(self.curr_offset)
            
        # Check Ending

        # Ending 1: Total Misison Time Out
        if time.time() - self.start_time > 120: # Arbitary set to 120 s for now, need to see what's the estimated mission time including searching
            self.end = True

        # End the script if we've lost the bins and then was searching for 15 seconds. This should result in
        # yawing for 15 seconds for a full 360 degrees. In theory, it should work if we're on top of the bins
        # (meaning that we won't find it). In practice, the model needs to be robust enough to prevent false positives.
        # Also, what if we are close to (but not on top of) the bins so that it's out of the FOV? I'll need to inspect
        # the drop script to see if you account for that.
        
        # Ending 2: Lost target durnig approaching and can't pick up detection again (Only when we have the ability to search again after lost target during approaching)
        if self.state=="search" and self.prev_detected:
            if time.time() - self.switch_back_time > 15:
                self.end = True

        # Ending 3: Lost target for 6.5 s continuously, either end the mission directly or switch back to search state
        if self.state=="approach" and (self.curr_offset is None) and self.prev_detected == True:
            lost_detection_time = time.time() - self.prev_time
            print(f"Lost detection for {lost_detection_time} s during approaching")
            if  lost_detection_time> 6.5:
                """Uncomment the following code if you want to have the ability to search again after lost target during approaching"""
                # if self.switch_count <2:  # switch back to search again
                #     print(f"[DEBUG] switch back to search state")
                #     self.state = "search"
                #     self.switch_count += 1
                #     self.switch_back_time = time.time()

                # else:
                #     print(f"[DEBUG] Ending with prev detected: {self.prev_detected}")
                #     self.end = True
                print(f"[DEBUG] Ending with prev detected: {self.prev_detected}")
                self.end = True
        
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"state": self.state, "prev_offset": self.prev_offset,"lateral": lateral, "forward": forward, "yaw": yaw, "vertical" : vertical, "end": self.end}, frame
