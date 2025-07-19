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
    model = "sawfish_bin"  # Will be replaced dynamically depending on mission param

    def __init__(self, **config):
        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2

        self.state = "approach"
        self.distance_threshold = 1.0  # feet
        self.focal_length_px = 615  # approx for OAK-D-W at 640x480

        self.physical_bin_width_ft = 1.5  # real bin width in ft
        self.aligned = False
        self.end = False

        self.target_side = config.get("target_side", "sawfish")  # 'shark' or 'sawfish'
        self.last_detection_time = time.time()

        print(f"[INFO] Bin CV initialized. Target side: {self.target_side}")

    def estimate_distance(self, bbox_width_px):
        if bbox_width_px == 0:
            return float("inf")
        return (self.physical_bin_width_ft * self.focal_length_px) / bbox_width_px

    def smart_movement(self, x, y):
        lateral = 0
        forward = 0
        tol = 20

        if x < self.x_midpoint - tol:
            lateral = -0.5
        elif x > self.x_midpoint + tol:
            lateral = 0.5

        if y < self.y_midpoint - tol:
            forward = 0.5
        elif y > self.y_midpoint + tol:
            forward = -0.5

        return forward, lateral

    def run(self, frame, target, detections):
        forward = 0
        lateral = 0
        vertical = 0
        yaw = 0

        if self.state == "approach":
            for det in detections:
                if "bin" in det.label and det.confidence > 0.5:
                    bbox_width = det.xmax - det.xmin
                    distance = self.estimate_distance(bbox_width)

                    if distance > self.distance_threshold:
                        forward = 0.3
                    else:
                        self.state = "align"
                        print("[INFO] Switched to bottom camera for alignment")
                    break
            else:
                # No bin detected
                forward = 0.3

        elif self.state == "align":
            found_target = False
            for det in detections:
                if self.target_side in det.label and det.confidence > 0.65:
                    x_mid = (det.xmin + det.xmax) / 2
                    y_mid = (det.ymin + det.ymax) / 2
                    forward, lateral = self.smart_movement(x_mid, y_mid)
                    found_target = True

                    if forward == 0 and lateral == 0:
                        self.aligned = True
                        self.end = True
                        print("[INFO] Alignment complete over target bin side")
                    break

            if not found_target:
                # Lost target during alignment
                if time.time() - self.last_detection_time > 2:
                    lateral = 0.3  # strafe to look for it
                else:
                    forward = 0.1
            else:
                self.last_detection_time = time.time()

        return {
            "lateral": lateral,
            "forward": forward,
            "yaw": yaw,
            "vertical": vertical,
            "end": self.end
        }, frame