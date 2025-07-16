"""
Pole Slalom CV. Detects red poles, yaws to face it, and approaches until close.
"""

import cv2
import time
import numpy as np
import os

import cv2
import numpy as np
import time

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.shape = None  # Will set this dynamically
        self.x_midpoint = None
        self.tolerance = 40  # How centered the object should be
        self.config = config
        self.state = "searching"  # searching → centering → approaching
        self.end = False
        self.start_time = time.time()
        self.rows_completed = 0

        print("[INFO] Pole Center & Approach CV initialized")

    def detect_red_pole(self, frame):
        # Set shape dynamically on first frame
        if self.shape is None:
            height, width = frame.shape[:2]
            self.shape = (width, height)
            self.x_midpoint = width / 2
            print(f"[INFO] Frame shape set dynamically: width={width}, height={height}")

        crop_bottom = 40
        frame = frame[0:self.shape[1] - crop_bottom, :]

        # Step 1: HSV Red Mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 80, 50])
        upper_red1 = np.array([12, 255, 255])
        lower_red2 = np.array([168, 80, 50])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Step 2: Apply mask to image (keep only red regions)
        red_regions = cv2.bitwise_and(frame, frame, mask=red_mask)

        # Step 3: Convert to grayscale
        gray = cv2.cvtColor(red_regions, cv2.COLOR_BGR2GRAY)

        # Step 4: Blur + Edge Detection
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
        edges = cv2.Canny(blurred, 50, 150)

        return edges, frame

    def movement_calculation(self, detection):
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        if self.state == "initial_search":
            if detection["status"]:
                self.state = "centering"
            else:
                # Spin in place to search
                yaw = -1.5
                print("[INFO] Searching: No red pole detected → yawing")

        elif self.state == "centering":
            if detection["status"]:
                x_center = (detection["xmin"] + detection["xmax"]) / 2
                offset = x_center - self.x_midpoint

                if abs(offset) > self.tolerance:
                    lateral = -1.0 if offset > 0 else 1.0
                    print(f"[INFO] Centering: offset={offset:.1f} → lateral={lateral}")
                else:
                    print("[INFO] Centering: Pole centered → transitioning to approaching")
                    self.state = "approaching"
            else:
                print("[WARN] Lost pole while centering → reverting to searching")
                self.state = "initial_search"

        elif self.state == "approaching":
            if detection["status"]:
                area = detection["area"]
                if area < 16000:
                    forward = 2.0
                    print(f"[INFO] Approaching: area={area:.0f} → moving forward")
                else:
                    forward = 0
                    self.end = True
                    print("[INFO] Pole is close enough → stopping")
                    self.state = "strafing"
            else:
                print("[WARN] Lost pole while approaching → reverting to searching")
                self.state = "initial_search"
        
        elif self.state == "strafing":
            while time.time() - self.start_time < 1.5:  # Strafing for 1.5 seconds
                lateral = 2.0
                print(f"[INFO] Strafing: Moving laterally to come in between poles")
                self.state = "slaloming"

        elif self.state == "slaloming":
            while time.time() - self.start_time < 2:  # Slaloming for 2 seconds
                forward = 2.0
                print(f"[INFO] Slaloming: Moving forward through the poles")
                self.rows_completed += 1
                
            if self.rows_completed >= 3:
                self.end = True
                print("[INFO] Completed slalom through poles → ending")
                
            else:
                self.state = "internal_searching"
        
        elif self.state == "internal_searching":
            elapsed_time = time.time() - self.start_time
            yaw = 1.0 if int(elapsed_time / 0.5) % 2 == 0 else -1.0
            print(f"[INFO] Internal Searching: Yawing to find next pole (elapsed time: {elapsed_time:.1f}s)")
            
            if detection["status"]:
                self.state = "approaching"
                print("[INFO] Found next pole while searching → transitioning to approaching")
            else:
                self.state = "internal_searching"

        return forward, lateral, yaw, vertical

    def run(self, raw_frame, target, detections):
        detection, mask = self.detect_red_pole(raw_frame)
        forward, lateral, yaw, vertical = self.movement_calculation(detection)

        # Visualization
        frame = raw_frame.copy()
        if detection["status"]:
            x1, y1 = detection["xmin"], detection["ymin"]
            x2, y2 = detection["xmax"], detection["ymax"]
            x_center = int((x1 + x2) / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (x_center, 0), (x_center, self.shape[1]), (255, 255, 0), 2)
            cv2.line(frame, (int(self.x_midpoint), 0), (int(self.x_midpoint), self.shape[1]), (0, 255, 0), 1)
            cv2.putText(frame, f"Area: {detection['area']}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        cv2.putText(frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        return {"lateral": lateral, "forward": forward, "yaw": yaw,"vertical": vertical, "end": self.end}, frame