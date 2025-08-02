"""
Bin CV. Locates the correct side of the bin (shark or sawfish), and aligns with the intention to drop the marker into the correct side of the bin.
"""

import time
import cv2
import numpy as np

class CV:
    """
    Bin CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    camera = "/auv/camera/videoOAKdRawBottom"  # Switches to bottom cam after approach
    model = "everything"  # Will be replaced dynamically depending on mission param

    def __init__(self, **config):
        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2

        self.animal_detected = False

        self.state = "rotating"

        self.aligned = False
        self.end = False
        self.drop = False

        self.last_detection_time = time.time()

        print(f"[INFO] Bin CV initialized.")


    def run(self, frame, target, detections):
        forward = 0
        lateral = 0
        vertical = 0
        yaw = 0
        bin_detection = None
        sawfish_detection = None
        shark_detection = None
        bin_detected = False
        target_animal = target
        # Step 1, filter out the detection
        for dect in detections:
            if "bin" in dect.label:
                bin_detected = True
                if dect.label=="bin":
                    bin_detection = dect
                elif dect.label=="bin_sawfish":
                    sawfish_detection = dect
                elif dect.label=="bin_shark":
                    shark_detection = dect

                self.state = "centering"
                self.last_detection_time = time.time()
            
        # Step 1.5, check timeout if we lost detection for 15 s
        if not bin_detected and time.time()-self.last_detection_time>15:
            self.end = True
            

        # Step 2, main logic for rotating and centering
        if self.state == "rotating":
            if bin_detection is not None:
                bin_length = bin_detection.xmax - bin_detection.xmin
                bin_width = bin_detection.ymax - bin_detection.ymin
                current_bin_ratio = (bin_length/bin_width)
                if abs(current_bin_ratio)<0.2:
                    self.state = "centering"
            else:
                yaw = 0.75 # continuously yaw cw to check if we are in the correct orientation

        elif self.state == "centering":
            Targets = {"bin":None,"sawfish":None,"shark":None} # each value should be [(x,y),ratio]
            
            # Extract the dimension we want
            if bin_detection is not None:
                Bin_center_x = (bin_detection.xmin+bin_detection.xmax)/2
                Bin_center_y = (bin_detection.ymax+bin_detection.ymin)/2
                Bin_length = bin_detection.xmax-bin_detection.xmin
                Bin_width = bin_detection.ymax-bin_detection.ymin
                pixToMeter = ((0.9144/Bin_length) + (0.6096/Bin_width))/2  # These numbers are the actual dimension of the bin
                Targets["bin"] = [(Bin_center_x,Bin_center_y), pixToMeter]
            if sawfish_detection is not None:
                Sawfish_center_x = (sawfish_detection.xmin+sawfish_detection.xmax)/2
                Sawfish_center_y = (sawfish_detection.ymin+sawfish_detection.ymax)/2
                Sawfish_side_length = (sawfish_detection.xmax+sawfish_detection.ymax-sawfish_detection.xmin-sawfish_detection.ymin)/2
                pixToMeter = (0.3048/Sawfish_side_length)
                Targets["sawfish"] = [(Sawfish_center_x,Sawfish_center_y), pixToMeter]

            if shark_detection is not None:
                Shark_center_x = (shark_detection.xmin+shark_detection.xmax)/2
                Shark_center_y = (shark_detection.ymin+shark_detection.ymax)/2
                Shark_side_length = (shark_detection.xmax+shark_detection.ymax-shark_detection.xmin-shark_detection.ymin)/2	
                pixToMeter = (0.3048/Shark_side_length)
                Targets["shark"] = [(Shark_center_x,Shark_center_y), pixToMeter]
        
        
        # Step 3, average out the pixToMeter factor
        Sum_pixToMeter = 0
        Counter = 0
        Sum_y = 0
        for key, value in Targets:
            if value is not None:
                Target_y = value[0][1]
                Sum_y += Target_y 
                Sum_pixToMeter += value[1]
                Counter += 1
        
        target_y = Sum_y / Counter
        target_pixToMeter = Sum_pixToMeter/Counter

        # Step 4, calculate the dropper center base on the target animal
        if Targets[target_animal] is not None:
            target_x = Targets[target_animal][0][0] - (0.275/target_pixToMeter)# extract x coordinate, 0.275 is the horizontal distance the dropper is away from the camera center
        else: # calculate the target x base on other detecitons
            if target_animal=="sawfish":
                Other_animal = "shark"
            else:
                Other_animal = "sawfish"
            target_x = None
            if Targets[Other_animal] is not None:
                Other_x = Targets[Other_animal][0][0]
                if Other_x<Targets["bin"][0][0]:
                    target_x = Targets["bin"][0][0] + (0.1524/target_pixToMeter)
                else: 
                    target_x = Targets["bin"][0][0] - (0.1524/target_pixToMeter)
            else:
                # Just use the center of the bin if we don’t detect any target animal
                target_x=Targets["bin"][0][0]

        # Step 5, Calculate pwm base on target x,y
        Offset_x = target_x - self.x_midpoint
        Offset_y = target_y - self.y_midpoint
        x_aligned = False
        y_aligned = False
        if abs(Offset_x) > 100:
            if Offset_x > 0:
                lateral = 0.5
            else:
                lateral = -0.5
        else:
            x_aligned = True
        if abs(Offset_y) > 100:
            if Offset_y > 0:
                forward = -0.5
            else:
                forward = 0.5
        else:
            y_aligned = True
        
        if x_aligned and y_aligned:
            self.end = True  # end the mission and drop the ball
            self.drop = True
        
        return {
            "lateral": lateral,
            "forward": forward,
            "yaw": yaw,
            "vertical": vertical,
            "end": self.end,
            "drop": self.drop
        }, frame