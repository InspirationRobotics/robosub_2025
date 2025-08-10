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
    model = "bins"  # Will be replaced dynamically depending on mission param

    # Again, add doc strings and type hints to all functions/methods

    def __init__(self, **config: dict):
        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2

        self.animal_detected = False

        self.state = "centering"

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
                print(f"[INFO] {dect.label} detected")
                if dect.label=="bin":
                    bin_detection = dect
                elif dect.label=="bin_sawfish":
                    sawfish_detection = dect
                elif dect.label=="bin_shark":
                    shark_detection = dect

                self.last_detection_time = time.time()

        
        Targets = {"bin":None,"sawfish":None,"shark":None} # each value should be [(x,y), pix-meter ratio]
        numDetected = 0
        # Extract the dimension we want, assuming that we have one out of three of the detection

        # These are doing a lot of the same thing. For the purposes of DRY, I might consider a helper function.
        # To make the data more organized, I would consider using either a dictionary or a NamedTuple.
        if bin_detection is not None:  # TODO consider not using the bin detection at all, we only focus on marine animals
            Bin_center_x = (bin_detection.xmin+bin_detection.xmax)/2
            Bin_center_y = (bin_detection.ymax+bin_detection.ymin)/2
            Bin_length = bin_detection.xmax-bin_detection.xmin
            Bin_width = bin_detection.ymax-bin_detection.ymin
            pixToMeter = ((0.9144/Bin_length) + (0.6096/Bin_width))/2  # These numbers are the actual dimension of the bin
            Targets["bin"] = [(Bin_center_x,Bin_center_y), pixToMeter]
            numDetected += 1
        if sawfish_detection is not None:
            Sawfish_center_x = (sawfish_detection.xmin+sawfish_detection.xmax)/2
            Sawfish_center_y = (sawfish_detection.ymin+sawfish_detection.ymax)/2
            Sawfish_side_length = (sawfish_detection.xmax+sawfish_detection.ymax-sawfish_detection.xmin-sawfish_detection.ymin)/2
            pixToMeter = (0.3048/Sawfish_side_length)
            Targets["sawfish"] = [(Sawfish_center_x,Sawfish_center_y), pixToMeter]
            numDetected += 1
        if shark_detection is not None:
            Shark_center_x = (shark_detection.xmin+shark_detection.xmax)/2
            Shark_center_y = (shark_detection.ymin+shark_detection.ymax)/2
            Shark_side_length = (shark_detection.xmax+shark_detection.ymax-shark_detection.xmin-shark_detection.ymin)/2	
            pixToMeter = (0.3048/Shark_side_length)
            Targets["shark"] = [(Shark_center_x,Shark_center_y), pixToMeter]
            numDetected += 1            
        
        # Step 1.5, check timeout if we lost detection for 15 s
        if time.time()-self.last_detection_time>15:
            print("[INFO] Timeout for bin drop mission")
            self.end = True
            
        # Step 2, main logic for rotating and centering  # TODO add a centering state before we do rotating
        if self.state == "centering":
            # Average out the center to find the target center

            # What if not all of the possible detections are on the screen (or completely on the screen)? 
            # That would bias the target center quite a bit. Is this your intention, or is it that as
            # the bin moves more into the frame, the target center would adjust appropriately? 
            # Chase: Correct! As it approach it will adjust appropriatedly
            if numDetected>0:
                sum_x = 0
                sum_y = 0
                for key, value in Targets.items():
                    if value is not None:
                        sum_x += value[0][0]
                        sum_y += value[0][1]
                
                average_x =  sum_x/numDetected
                average_y = sum_y/numDetected

                # Calculate pwm base on target x,y to align frame center with
                # target center
                Offset_x = average_x - self.x_midpoint
                Offset_y = average_y - self.y_midpoint
                x_aligned = False
                y_aligned = False
                if abs(Offset_x) > 100:
                    if Offset_x > 0:
                        lateral = 1.5
                    else:
                        lateral = -1.5
                else:
                    print("[INFO] x aligned in centering state")
                    x_aligned = True
                
                # NOTE: Larger y for pixels are lower in the frame (not higher)
                if abs(Offset_y) > 100:
                    if Offset_y > 0:
                        forward = -1.5
                    else:
                        forward = 1.5
                else:
                    print("[INFO] y aligned in centering state")
                    y_aligned = True
                
                if x_aligned and y_aligned:
                    print("[INFO] switching to rotating state")
                    self.state = "rotating"

        elif self.state == "rotating" and numDetected > 0:  # TODO Use all three detection and check if the center_y align within tolerance
            if bin_detection is not None:  
                # Shouldn't we have the length and width already from further up in the code?
                bin_length = bin_detection.xmax - bin_detection.xmin
                bin_width = bin_detection.ymax - bin_detection.ymin
                current_bin_ratio = (bin_length/bin_width)

                # I'll need to review this ratio part with you orally to understand what's happening. I have a general
                # idea from our pseudocode discussion, but I don't understand the low-level. Maybe putting a comment
                # in the code would help future readers understand?

                # Also, if we have a detection but the abs() condition is not fulfilled, it seems like we'll be sitting
                # there doing nothing for a while. We may want to pin an else statement on that.
                
                # 1.5 is the bin_length/bin_width
                if abs(current_bin_ratio-1.5)<0.16: # TODO consider decresing 0.2 because our screen ratio is 1.333
                    self.state = "finetune"
                    print("[INFO] Rotated to the correct orientation, switch to centering state")
                else:
                    yaw = 0.7
            else:
                # How do we know this will not result in having us facing opposite the intended orientation?
                # Chase: as opposite doesn't really matter, we will handle that in finetune state
                yaw = 0.75 # continuously yaw cw to check if we are in the correct orientation

        elif self.state == "finetune" and numDetected >0:
        
            # Average out the pixToMeter factor because we can not assume that we will always have bin deteciton
            Sum_pixToMeter = 0  # m/pixel
            Counter = 0
            Sum_y = 0
            for key, value in Targets.items():
                if value is not None:
                    Target_y = value[0][1]
                    Sum_y += Target_y 
                    Sum_pixToMeter += value[1]
                    Counter += 1
            
            target_pixToMeter = Sum_pixToMeter/Counter  # m/pixel
            target_y = Sum_y / Counter - 0.08/target_pixToMeter  # 0.08 m is the offset of the dropper in y direction

            # Calculate the dropper center base on the target animal
            if Targets[target_animal] is not None:
                target_x = Targets[target_animal][0][0] + (0.275/target_pixToMeter)# extract x coordinate, 0.275 is the horizontal distance the dropper is away from the camera center
            else: # calculate the target x base on other detecitons
                if target_animal=="sawfish":
                    Other_animal = "shark"
                else:
                    Other_animal = "sawfish"
                target_x = None
                try:
                    if Targets[Other_animal] is not None:
                        Other_x = Targets[Other_animal][0][0]

                        # What if there isn't a bin detection? This will crash with an index error. You may want
                        # to perform some exception handling for that.
                        if Other_x<Targets["bin"][0][0]:

                            # Offset for a quarter of bin length (or half of a half)
                            # to get to the x-center of a particular side. Assumption is we're
                            # centered on the bin center (with some tolerance).
                            target_x = Targets["bin"][0][0] + (0.1524/target_pixToMeter) + (0.275/target_pixToMeter)  # 0.1524 is 6 inches, which is half of the poster side length
                        else: 
                            target_x = Targets["bin"][0][0] - (0.1524/target_pixToMeter) + (0.275/target_pixToMeter) # 0.275m is the offset between the camera center and dropper center
                    else:
                        # Just use the center of the bin if we don’t detect any target animal
                        target_x=Targets["bin"][0][0]
                except IndexError as e:
                    print("[WARN] No bin detected, set target x to center of the screen")
                    target_x = self.x_midpoint

            # Step 5, Calculate pwm base on target x,y

            # This logic is exactly the same as the centering state (except that this time
            # we account for the dropper offset). Can we use a helper function to follow DRY?
            Offset_x = target_x - self.x_midpoint
            Offset_y = target_y - self.y_midpoint
            x_aligned = False
            y_aligned = False
            if abs(Offset_x) > 50: 
                x_aligned = False
                if Offset_x > 0:
                    lateral = 1.5
                else:
                    lateral = -1.5
            else:
                print("[INFO] x aligned")
                x_aligned = True
            if abs(Offset_y) > 50:
                y_aligned = False
                if Offset_y > 0:
                    forward = -1.5
                else:
                    forward = 1.5
            else:
                print("[INFO] y aligned")
                y_aligned = True
            
            if x_aligned and y_aligned:
                print("[INFO] Dropping the marker")
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