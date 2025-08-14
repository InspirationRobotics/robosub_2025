"""
Pole Slalom CV. Detects red poles, yaws to face it, and approaches until close.
"""
import cv2
import time
import numpy as np
import os
from auv.motion.utils import get_norm

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.shape = (640, 480)
        self.x_midpoint = 320
        self.tolerance = 40  # How centered the object should be in px
        self.config = config
        self.state = "centering"
        self.start_time = time.time()
        self.end = False
        self.reached = False
        self.prev_detect_timestamp = None
        print("[INFO] Pole Center & Approach CV initialized")

    def detect_red_pole(self, frame):
        """
        Detect red pole using HSV, return status, bbox and mask frame
        """
        crop_bottom = 80
        height = frame.shape[0]
        frame = frame[0:height - crop_bottom, :]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([155, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        red_mask_clean = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask_clean = cv2.morphologyEx(red_mask_clean, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(red_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_poles = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            
            # UNUSED rotated bounding box
            # See https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
            rotated_box = cv2.boxPoints(cv2.minAreaRect(cnt))
            rotated_box = np.int0(rotated_box)

            sorted_box_coords = sorted(rotated_box, key=lambda y_coord: y_coord[1])
            rotated_width =  get_norm(sorted_box_coords[1][0] - sorted_box_coords[0][0],
                                      sorted_box_coords[1][1] - sorted_box_coords[0][1])
            # w = rotated_width
            

            # Cutoff for width for detection to count
            if w > 20:
                aspect_ratio = h / float(w) if w > 0 else 0
                if aspect_ratio > 10:
                    red_poles.append((x, y, w, h))

        if red_poles:
            red_poles.sort(key=lambda x: x[4], reverse=True)
            x, y, w, h = red_poles[0]
            return {
                "status": True, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h
            }, red_mask_clean

        return {
            "status": False, "xmin": None, "xmax": None, "ymin": None, "ymax": None
        }, red_mask_clean

    def centering(self, detection):
        forward = 1.5
        lateral = 0
        yaw = 0
        vertical = 0
        if detection["status"]:
            self.prev_detect_timestamp = None # set timer to None
            # go forward at power 2 when detected
            forward = 1.5

            # calculate offset between screen center and target center
            pole_x_center = (detection["xmin"] + detection["xmax"]) / 2
            offset = pole_x_center - self.x_midpoint

            # movement calculation
            if abs(offset) > self.tolerance:
                lateral = 1.0 if offset > 0 else -1.0
                print(f"[INFO] Centering: offset={offset:.1f} → lateral={lateral}")
                self.reached = False
            else:
                # No lateral motion when you are within tolerance
                lateral = 0
                forward = 2

                width = detection["xmax"] - detection["xmin"]
                print(f"[INFO] Approaching: width={width:.0f} → moving forward")
                # Distance estimation to stop moving forward
                if width >= 40: 
                    self.reached = True
                    forward = 0
                    lateral = 0
                    yaw = 0
                    vertical = 0
                else:
                    self.reached = False

        else:
            if self.prev_detect_timestamp is None:
                self.prev_detect_timestamp = time.time()
            # Blindly go forward slowly when lost detection
            print("[WARN] Lost pole while centering")
            forward = 1

        return forward, lateral, yaw, vertical

    def run(self, raw_frame, target, detections):
        # Detect red pole
        detection, red_mask_clean = self.detect_red_pole(raw_frame)
        
        # Calculate movement
        forward, lateral, yaw, vertical = self.centering(detection)

        # Check for timeout
        if time.time() - self.start_time > 45: # predicted mission time is 30s
            self.end = True
            print("[INFO] Pole Slalom mission timed out - 45 seconds elapsed")
        
        # Timeout if lost detection for 
        # #########################################
        # Visualization
        frame = raw_frame.copy()
        if detection["status"] and detection["xmin"] is not None and detection["xmax"] is not None:
            x1, y1 = detection["xmin"], detection["ymin"]
            x2, y2 = detection["xmax"], detection["ymax"]
            pole_x_center = int((x1 + x2) / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (pole_x_center, 0), (pole_x_center, self.shape[1]), (255, 255, 0), 2)
            cv2.line(frame, (int(self.x_midpoint), 0), (int(self.x_midpoint), self.shape[1]), (0, 255, 0), 1)

            cv2.putText(frame, f"Width: {detection['xmax'] - detection['xmin']}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(frame, f"State: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end, "reached": self.reached}, frame