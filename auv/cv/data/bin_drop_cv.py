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

    # Again, add doc strings and type hints to all functions/methods

    def __init__(self, **config: dict):
        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2

        self.animal_detected = False

        self.state = "centering_bin"

        self.aligned = False
        self.end = False

        self.last_detection_time = time.time()

        print(f"[INFO] Bin CV initialized.")
        
def run(self, frame, detections, target="shark"):
    forward = 0
    lateral = 0
    vertical = 0
    yaw = 0

    bin_detection = None
    shark_detection = None

    # Step 1: Filter detections
    for dect in detections:
        if "bin" in dect.label:
            if dect.label == "bin":
                bin_detection = dect
        elif "shark" in dect.label:
            shark_detection = dect

        self.last_detection_time = time.time()

    # Step 1.5: Timeout if no detection
    if time.time() - self.last_detection_time > 15:
        print("[INFO] Timeout for bin drop mission")
        self.end = True
        return {
            "lateral": lateral,
            "forward": forward,
            "yaw": yaw,
            "vertical": vertical,
            "end": self.end,
        }, frame

    # Step 2: Center with the bin first
    if self.state == "centering_bin" and bin_detection is not None:
        Bin_center_x = (bin_detection.xmin + bin_detection.xmax) / 2
        Bin_center_y = (bin_detection.ymin + bin_detection.ymax) / 2

        Offset_x = Bin_center_x - self.x_midpoint
        Offset_y = Bin_center_y - self.y_midpoint

        x_aligned = abs(Offset_x) < 80
        y_aligned = abs(Offset_y) < 80

        if not x_aligned:
            lateral = 1.0 if Offset_x > 0 else -1.0
        if not y_aligned:
            forward = -1.0 if Offset_y > 0 else 1.0

        if x_aligned and y_aligned:
            print("[INFO] Bin centered, switching to centering shark")
            self.state = "centering_shark"

    # Step 3: Center with the shark (target side)
    elif self.state == "centering_shark" and shark_detection is not None:
        Shark_center_x = (shark_detection.xmin + shark_detection.xmax) / 2
        Shark_center_y = (shark_detection.ymin + shark_detection.ymax) / 2

        Offset_x = Shark_center_x - self.x_midpoint
        Offset_y = Shark_center_y - self.y_midpoint

        x_aligned = abs(Offset_x) < 80
        y_aligned = abs(Offset_y) < 80

        if not x_aligned:
            lateral = 1.0 if Offset_x > 0 else -1.0
        if not y_aligned:
            forward = -1.0 if Offset_y > 0 else 1.0

        if x_aligned and y_aligned:
            print("[INFO] Shark centered, dropping marker")
            self.end = True

    return {
        "lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end}, frame