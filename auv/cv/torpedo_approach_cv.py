"""
Torpedo CV. Detects torpedo target and approaches it.
"""

# Import what you need from within the package.

import numpy as np
import time

class CV:
    """
    Template CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawForward" 
    model = "everything"

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """
        self.shape = (640, 480)
        self.x_midpoint = 320
        self.config = config
        self.state = "search"
        self.end = False
        self.tolerance = 40 # How centered the object should be in px
        self.start_time = None

        print("[INFO] Torpedo CV init")

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
        vertical = 0
        yaw = 0
        
        if self.state == "search":
            for det in detections:
                if "torpedo_target" in det.label and det.confidence > 0.5:
                    print("[INFO] Torpedo target detected, moving towards it")
                    
                    if self.start_time is None:
                        self.start_time = time.time()
                        print("[INFO] Approaching started")

                    if time.time() - self.start_time < 8.0:
                        forward = 1.0  # Move forward towards the target
                        print(f"[INFO] Moving forward for ({time.time() - self.start_time:.2f}s)")                
                else:
                    yaw = 1.0 # Rotate to search for the torpedo target
                    
        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"yaw": yaw, "lateral": lateral, "forward": forward, "vertical": vertical, "end": self.end}, frame