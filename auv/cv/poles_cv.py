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
        self.state = "initial_search"
        self.end = False
        self.start_time = time.time()
        self.rows_completed = 0

        print("[INFO] Pole Center & Approach CV initialized")

    def detect_red_pole(self, frame):
        crop_bottom = 80
        height = frame.shape[0]
        frame = frame[0:height - crop_bottom, :]

        # Step 1: Convert to HSV and create a mask for red color
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 80, 50])
        upper_red1 = np.array([12, 255, 255])
        lower_red2 = np.array([168, 80, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Step 2: Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        red_mask_clean = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask_clean = cv2.morphologyEx(red_mask_clean, cv2.MORPH_OPEN, kernel)

        # Step 3: Canny edge detection
        edges = cv2.Canny(red_mask_clean, 50, 150)

        # Step 4: Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=40, maxLineGap=10)
        vertical_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                if abs(angle) > 75:  # Near-vertical
                    vertical_lines.append((x1, y1, x2, y2))

        # Step 5: Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_poles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = h / float(w) if w != 0 else 0
                if aspect_ratio > 1.5:
                    # Check if a vertical Hough line intersects the bounding box
                    for x1, y1, x2, y2 in vertical_lines:
                        if x1 >= x and x1 <= x + w:
                            red_poles.append((x, y, w, h, area))
                            break

        if red_poles:
            red_poles.sort(key=lambda x: x[4], reverse=True)
            x, y, w, h, area = red_poles[0]
            return {
                "status": True, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h, "area": area
            }, red_mask_clean
        return {
            "status": False, "xmin": None, "xmax": None, "ymin": None, "ymax": None, "area": 0
        }, red_mask_clean
    
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
                yaw = -0.8
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
        
        if self.shape is None:
            h, w = raw_frame.shape[:2]
            self.shape = (w, h)
            self.x_midpoint = w // 2
        
        detection, red_mask_clean = self.detect_red_pole(raw_frame)
        forward, lateral, yaw, vertical = self.movement_calculation(detection)

        # Visualization
        frame = raw_frame.copy()
        if detection["status"]:
            x1, y1 = detection["xmin"], detection["ymin"]
            x2, y2 = detection["xmax"], detection["ymax"]
            x_center = int((x1 + x2) / 2)

            # Draw bounding box and center lines
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (x_center, 0), (x_center, self.shape[1]), (255, 255, 0), 2)
            cv2.line(frame, (int(self.x_midpoint), 0), (int(self.x_midpoint), self.shape[1]), (0, 255, 0), 1)

            # Draw area text
            cv2.putText(frame, f"Area: {detection['area']}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Draw state text
        cv2.putText(frame, f"State: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Optional: show red mask overlay (commented for headless runtime)
        # cv2.imshow("Red Mask (Cleaned)", red_mask_clean)

        return {
            "lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end}, frame