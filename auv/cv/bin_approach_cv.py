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

        # Most var names (except the last three) don't immediately give away their function, although when I looked
        # at the code it made sense. Calling a switch back from approach to search an "adjust" is a little bit quirky,
        # there may be a better name for it. A big thing in industry is not just functional, but also readable code. 
        # Code isn't any good if the next person coming along has no idea what's happening.
        self.swtich_back_time = None
        self.search_counter = 0
        self.search_stage_one_timestamp = None # store time
        self.search_stage_two_timestamp = None # store time
        self.stage_two_end = False
        self.switch_count = 0
        self.end = False
        self.prev_offset = None
        self.prev_time = time.time()
        
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
        self.target_x = (detection.xmin + detection.xmax) / 2
        self.self.target_y = (detection.ymin + detection.ymax) / 2
        self.curr_offset = self.target_x - self.framecenter_x # These var names could use some work. Both self.target_x and self.framecenter_x are technically midpoints - the former of the detection bounding box, the latter of the frame
        self.prev_detected = True
        self.prev_offset = self.curr_offset
        self.state = "approach"
        print(f"[DEBUG] self.target_x is {self.target_x}, self.target_y is {self.target_y}") # Why are we including the self.target_y here but not if there's one detection?

    

    # Add type hints to this function
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
        if len(detections) == 0 and self.prev_detected == False:
            self.state = "search"
        
        # Utilize bin detections with at least 55% confidence
        detected_list = []
        detection_confidence = 0.55
        for det in detections:
            if "bin" in det.label:
                print(f"[DEBUG] Detected {det.label} with confidence {det.confidence}")
                if det.confidence > detection_confidence:
                    detected_list.append(det)

        # select the highest confidence Bin deteciton if multiple
        self.curr_offset = None
        if len(detected_list)==0:
            self.curr_offset = None
        elif len(detected_list)==1:
            self.prev_time = time.time()
            detection = detected_list[0]
            self.process_detection_offset(detection=detection)

        else:  # when there are more than one Bin detection
            # Select the detection with the highest confidence
            self.prev_time = time.time()
            detection = max(detected_list, key=lambda det: det.confidence)
            detection_confidence = detection.confidence
            print(f"[DEBUG] Multiple Bins detected. Using highest confidence detection: {detection_confidence}")
            self.process_detection_offset(detection=detection)


        if self.state == "search":
            # For less than two searches, yaw left at 20% for 5 seconds then yaw right at 20% for 5 seconds
            # This program will search left for a certain number of degrees then search right
            if self.search_counter<=2:
                if self.search_stage_one_timestamp is None:
                    print("[DEBUG] Searching in stage 1")
                    self.search_stage_one_timestamp = time.time()
                if time.time()-self.search_stage_one_timestamp > 5 * (self.search_counter + 1):
                    print(f"[DEBUG] Searching in stage one, counter is {self.search_counter}")
                    self.search_counter += 1
                    self.search_stage_one_timestamp = time.time()
                if self.search_counter%2==1:
                    yaw = 0.6
                else:
                    yaw = -0.6
            else:
            # Yaw in the direction where the bin was previously seen, or right if not seen
                if self.search_stage_two_timestamp is None:
                    print(f"[DEBUG] Searching in stage two")
                    self.search_stage_two_timestamp = time.time()
                
                if self.prev_offset is None:
                    yaw = 0.6
                elif self.prev_offset > 0 :
                    yaw = 0.6
                elif self.prev_offset < 0:
                    yaw = -0.6

        if self.state == "approach":
        # If a detection exists, end search stage two and approach
            if not self.stage_two_end:
                self.stage_two_end = True
                self.search_stage_two_timestamp=time.time()
            print("[DEBUG] Approaching now!")
            print(f"[INFO] offset is {self.curr_offset}")
            forward, yaw = self.smart_approach(self.curr_offset)
            
        # Check Ending

        # I don't see any place in the script where self.prev_detected is set to None. A quick test 
        # reveals False is not None. You need a better condition since at the present state, the script will never end.

        # Also, what if we've made it over the bins in less than 30 seconds after search_stage_two begins? Substitude self.prev_detected with self.prev_offset
        if self.state=="search" and self.prev_offset is None and self.search_stage_two_timestamp is not None and time.time()-self.search_stage_two_timestamp > 30:
           # when we had went through stage one and time out for 30 seconds
           print(f"[DEBUG] time out in searching")
           self.end = True

        # End the script if we've lost the bins and then was searching for 15 seconds. This should result in
        # yawing for 15 seconds for a full 360 degrees. In theory, it should work if we're on top of the bins
        # (meaning that we won't find it). In practice, the model needs to be robust enough to prevent false positives.
        # Also, what if we are close to (but not on top of) the bins so that it's out of the FOV? I'll need to inspect
        # the drop script to see if you account for that.

        if self.state=="search" and self.prev_detected: # you are in adjust search mode when you had detection but in search mode again
            if time.time() - self.swtich_back_time > 15:
                self.end = True

        # If we lost the bins for two seconds in approach mode, change to search mode up to two times per
        # mission attempt and start a timer for 15 seconds.
        if self.state=="approach" and (self.curr_offset is None) and self.prev_detected == True:
            lost_detection_time = time.time() - self.prev_time
            print(f"Lost detection for {lost_detection_time} s during approaching")
            if  lost_detection_time> 2:
                if self.switch_count <2:  # switch back to search again
                    print(f"[DEBUG] switch back to search state")
                    self.state = "search"
                    self.switch_count += 1
                    self.swtich_back_time = time.time()

                else:
                    print(f"[DEBUG] Ending with prev detected: {self.prev_detected}")
                    self.end = True
        
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical" : vertical, "end": self.end}, frame
