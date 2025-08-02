"""
Pole Slalom CV. Detects red poles, yaws to face it, and approaches until close.
"""
import cv2
import time
import numpy as np
from auv.motion import robot_control
import os

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.shape = (640, 480)
        self.x_midpoint = 320
        self.tolerance = 40  # How centered the object should be in px
        self.config = config
        self.state = "search"
        self.end = False
        self.start_time = None
        self.search_start_time = None
        self.rows_completed = 0

        print("[INFO] Pole Center & Approach CV initialized")

    def detect_red_pole(self, frame):
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
            area = cv2.contourArea(cnt)
            if area > 800:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = h / float(w) if w > 0 else 0
                if aspect_ratio > 1.5:
                    red_poles.append((x, y, w, h, area))

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

        if self.state == "search":
            if detection["status"]:
                self.state = "centering"

        elif self.state == "centering":
            if detection["status"]:
                pole_x_center = (detection["xmin"] + detection["xmax"]) / 2
                offset = pole_x_center - self.x_midpoint

                if abs(offset) > self.tolerance:
                    lateral = 1.0 if offset > 0 else -1.0
                    print(f"[INFO] Centering: offset={offset:.1f} → lateral={lateral}")
                else:
                    print("[INFO] Centering: Pole centered → transitioning to approaching")
                    self.state = "approaching"
            else:
                print("[WARN] Lost pole while centering → reverting to searching")
                self.state = "search"

        elif self.state == "approaching":
            if detection["status"]:
                area = detection["area"]
                forward = 2.0
                print(f"[INFO] Approaching: area={area:.0f} → moving forward")
                if area >= 5000:
                    self.state = "strafing"
            else:
                print("[WARN] Lost pole while approaching → reverting to searching")
                self.state = "search"

        elif self.state == "strafing":
            if self.start_time is None:
                self.start_time = time.time()
                print("[INFO] Strafing started")

            if time.time() - self.start_time < 1.5 and self.rows_completed < 2:
                lateral = 2.0
                print(f"[INFO] Strafing: Moving laterally ({time.time() - self.start_time:.2f}s)")
                
            else:
                self.start_time = None
                print("[INFO] Strafing complete → transitioning to slaloming")
                self.state = "slaloming"
                
            if self.rows_completed == 2 and time.time() - self.start_time >= 1.5:
                lateral = 2.0
                print(f"[INFO] Strafing: Moving laterally ({time.time() - self.start_time:.2f}s)")        
            else:
                self.start_time = None
                print("[INFO] Last strafe complete → transitioning to slaloming")
                self.state = "3rd slaloming"

        elif self.state == "slaloming":
            if self.start_time is None:
                self.start_time = time.time()
                print("[INFO] Slaloming started")

            if time.time() - self.start_time < 3.0:
                forward = 2.0
                print(f"[INFO] Slaloming: Moving forward ({time.time() - self.start_time:.2f}s)")
            else:
                self.start_time = None
                self.rows_completed += 1
                print(f"[INFO] Slaloming complete → rows completed: {self.rows_completed}")
                
                if self.rows_completed == 1:
                    self.state = "looking for 2nd red pole"
                elif self.rows_completed == 2:
                    self.state = "looking for 3rd red pole"

        elif self.state == "looking for 2nd red pole":
            print("[INFO] Looking for 2nd red pole")
            self.state = "2nd pole search"

        elif self.state == "2nd pole search":
            print("[INFO] Searching for 2nd red pole")
            if detection["status"]:
                self.state = "strafing"

        elif self.state == "looking for 3rd red pole":
            print("[INFO] Looking for 3rd red pole")
            self.state = "3rd pole search"

        elif self.state == "3rd pole search":
            print("[INFO] Searching for 3rd red pole")
            if detection["status"]:
                self.state = "strafing"

        elif self.state == "3rd slaloming":
            if self.start_time is None:
                self.start_time = time.time()
                print("[INFO] 3rd Slaloming started")

            if time.time() - self.start_time < 6.0:
                forward = 2.0
                print(f"[INFO] 3rd Slaloming: Moving forward ({time.time() - self.start_time:.2f}s)")
            else:
                self.start_time = None
                self.rows_completed += 1
                print(f"[INFO] 3rd Slaloming complete → rows completed: {self.rows_completed}")
                self.end = True
        
        return forward, lateral, yaw, vertical

    def run(self, raw_frame, target, detections):
        
        # Crop right half only in strafing state
        if self.state == "strafing":
            raw_frame = raw_frame[:, 320:]
            
        # # Crop left half only in strafing state
        # if self.state == "strafing":
        #     raw_frame = raw_frame[:, :320]

        detection, red_mask_clean = self.detect_red_pole(raw_frame)
        forward, lateral, yaw, vertical = self.movement_calculation(detection)

        # Determine heading control flag based on state
        heading_control = True
        
        if self.state in ["looking for 2nd red pole"]:
            heading_control = False
            search_heading = 30
            
        elif self.state in ["looking for 3rd red pole"]:
            heading_control = False
            search_heading = 330         
        else:
            heading_control = True
            search_heading = 0

        # Visualization
        frame = raw_frame.copy()
        if detection["status"] and detection["xmin"] is not None and detection["xmax"] is not None:
            x1, y1 = detection["xmin"], detection["ymin"]
            x2, y2 = detection["xmax"], detection["ymax"]
            pole_x_center = int((x1 + x2) / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (pole_x_center, 0), (pole_x_center, self.shape[1]), (255, 255, 0), 2)
            cv2.line(frame, (int(self.x_midpoint), 0), (int(self.x_midpoint), self.shape[1]), (0, 255, 0), 1)

            cv2.putText(frame, f"Area: {detection['area']}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(frame, f"State: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return {
            "lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end, "heading_control": heading_control, "search_heading": search_heading}, frame