"""
Pole Slalom CV. Detects red poles, approaches them according to gate side,
and performs repeatable slalom movement with distance-based logic.
"""

import cv2
import time
import numpy as np
import os

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.tolerance = 50
        self.row_counter = 0
        self.max_rows = 3
        self.config = config
        self.side = config.get("side", "right")  # from gate mission
        self.state = "searching"
        self.end = False
        self.depth_time = time.time()
        self.slanted_yaw = 0
        self.strafe_start_time = time.time()

        print("[INFO] Pole Slalom CV initialized")

    def calculate_distance(self, object_width_px):
        """
        Calculate the distance (in feet) to a red pole using bounding box width. (below is for Graey's OAK-D Wide)
        """
        focal_length = 2.75  # mm
        real_pole_width = 25.4  # mm
        image_width_px = 640  # px
        camera_width = 97  # mm

        if object_width_px <= 0:
            return None

        distance_mm = (focal_length * real_pole_width * image_width_px) / (object_width_px * camera_width)
        distance_ft = distance_mm / 304.8
        return distance_ft

    def detect_red_pole(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_poles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                red_poles.append((x, y, w, h, area))

        if red_poles:
            red_poles.sort(key=lambda x: x[4], reverse=True)
            x, y, w, h, area = red_poles[0]
            return {
                "status": True,
                "xmin": x,
                "xmax": x + w,
                "ymin": y,
                "ymax": y + h,
                "area": area
            }, red_mask
        return {"status": False, "xmin": None, "xmax": None, "ymin": None, "ymax": None, "area": 0}, red_mask

    def movement_calculation(self, detection):
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        if self.row_counter >= self.max_rows:
            self.end = True
            return forward, lateral, yaw, vertical

        if detection["status"]:
            x1, x2 = detection["xmin"], detection["xmax"]
            object_width_px = x2 - x1
            distance_ft = self.calculate_distance(object_width_px)
            x_center = (x1 + x2) / 2

            if self.row_counter == 0:
                # ROW 1: Straight approach
                if self.state == "searching":
                    if distance_ft and distance_ft < 4:
                        self.state = "approaching"

                elif self.state == "approaching":
                    if distance_ft > 1.0:
                        forward = 1.2
                    else:
                        forward = 0
                        self.state = "strafing"
                        self.strafe_start_time = time.time()

                elif self.state == "strafing":
                    lateral = 0.8 if self.side == "right" else -0.8
                    if time.time() - self.strafe_start_time >= 2.0:
                        self.state = "moving_forward"

                elif self.state == "moving_forward":
                    forward = 1.0
                    self.row_counter += 1
                    self.state = "searching"

            else:
                # ROW 2 & 3: Slanted approach + realign
                if self.state == "searching":
                    yaw = -0.6 if self.side == "right" else 0.6
                    if distance_ft and distance_ft < 4:
                        self.state = "approaching"
                        self.slanted_yaw = yaw

                elif self.state == "approaching":
                    forward = 1.2
                    yaw = self.slanted_yaw
                    if distance_ft <= 1.0:
                        forward = 0
                        self.state = "realign"

                elif self.state == "realign":
                    yaw = -self.slanted_yaw
                    self.state = "strafing"
                    self.strafe_start_time = time.time()

                elif self.state == "strafing":
                    lateral = 0.8 if self.side == "right" else -0.8
                    if time.time() - self.strafe_start_time >= 2.0:
                        self.state = "moving_forward"

                elif self.state == "moving_forward":
                    forward = 1.0
                    self.row_counter += 1
                    self.state = "searching"

        else:
            # No detection, fallback motion based on state
            if self.state == "searching":
                yaw = -0.6 if (self.side == "right" and self.row_counter > 0) else 0.6 if self.row_counter > 0 else 0
            elif self.state == "approaching":
                forward = 1.2
                yaw = self.slanted_yaw if self.row_counter > 0 else 0
            elif self.state == "realign":
                yaw = -self.slanted_yaw
            elif self.state == "strafing":
                lateral = 0.8 if self.side == "right" else -0.8
            elif self.state == "moving_forward":
                forward = 1.0
                
            if self.row_counter >= self.max_rows:
                self.end = True
                print("[INFO] Pole Slalom Mission complete")           
            
            return forward, lateral, yaw, vertical
        
        # Fallback return statement (in case the one above fails)
        print(f"[WARN] [movement_calculation] Unexpected logic fallthrough, returning zeros.")
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

        cv2.putText(frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Row: {self.row_counter}/{self.max_rows}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # cv2.imshow("Pole Detection", frame)
        # cv2.imshow("Red Mask", mask)

        return {
            "lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end}, frame