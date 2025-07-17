"""
Poles Slalom CV - Template Style.
Detects red poles, approaches with adaptive offset, skips when close, performs dead reckoning, then repeats.
All motion outputs are scaled and clamped for robot_control compatibility.

Note: diameter = 1 inch = 2.54 cm, should be used for pole diameter if needed.
Using heading to determine yawing direction.
"""
from ultralytics import YOLO
import time                     #for timing dead reckoning
import cv2                      #for image processing   
import numpy as np              #for numerical operations, masks, contours

# === TUNABLE PARAMETERS ===
REAL_POLE_HEIGHT_CM = 91.44         # 3 feet pole = 91.44cm, 
FOCAL_LENGTH_MM = 2.75              # OAK-D W focal length (mm)
SENSOR_HEIGHT_MM = 3.4              # IMX378 sensor height (mm)
OFFSET_MIN_PX = 120                 # Min lateral offset in px (tune!)
OFFSET_MAX_PX = 200                # Max lateral offset in px (tune!)
DEAD_RECKONING_TIME_SEC = 2.0       # Time to move forward after pole is skipped (tune!)
SEARCH_YAW_MAX_DEG = 40             # Max yaw angle for search phase
LATERAL_MAX_POWER = 1              # Maps OFFSET_MAX_PX to this power
APPROACH_FORWARD_POWER = 1.5        # Forward power while approaching
SEARCH_YAW_POWER = 1                # Power for search yaw motion

def clamp_power(val):
    """Clamp any value(mainly for offset calculation) to [-5, 5] for movement outputs."""
    return max(-5, min(5, float(val)))

def scale_offset_to_power(offset_px, max_px=OFFSET_MAX_PX, max_power=LATERAL_MAX_POWER):
    """Scale pixel offset to robot power command [-max_power, max_power].""" 
    return clamp_power(max_power * offset_px / float(max_px))

class CV:
    """
    Red pole slalom CV class. Template style for easy integration.
    """
    camera = "/auv/camera/videoOAKdRawForward"   # Update to camera/video path in config

    def __init__(self, **config):
        #self.yolo = YOLO('D:/CV_data/YOLO model/Model_red_pole_3692.pt')  # (optional) load YOLO model
        self.side = config.get("side", "left")      # 'left' or 'right'
        self.passed_poles = 0                       # How many red poles have been passed
        self.state = "search"
        self.search_yaw_dir = 1
        self.yaw_accum = 0
        self.dead_reckoning_timer = None
        self.dead_reckoning_time = DEAD_RECKONING_TIME_SEC
        self.offset_min_px = OFFSET_MIN_PX
        self.offset_max_px = OFFSET_MAX_PX
        self.focal_px = None
        self.frame_height = None
        self.frame_width = None
        print(f"[INFO] Poles Slalom CV init, offset side: {self.side}")

    def estimate_distance_cm(self, pole_pixel_height):
        if pole_pixel_height <= 0 or self.focal_px is None:
            return None
        return (REAL_POLE_HEIGHT_CM * self.focal_px) / pole_pixel_height

    def get_largest_red_pole(self, red_poles):
        if not red_poles:
            return None
        return max(red_poles, key=lambda b: b[3])   # Largest by height

    def calc_offset_px(self, bbox_h):
        """Calculate lateral offset in pixels based on bounding box height."""
        if self.frame_height is None:
            return self.offset_min_px
        ratio = min(bbox_h / self.frame_height, 1.0)
        return int(self.offset_min_px + (self.offset_max_px - self.offset_min_px) * ratio)
    """
    def detect_red_poles(self, frame):
        # Run YOLO inference
        results = self.yolo(frame)
        red_poles = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w, h = x2 - x1, y2 - y1
                conf = float(box.conf[0])
                if conf > 0.5:  # Only use high-confidence detections
                    red_poles.append((x1, y1, w, h))
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)  # (optional) no mask for YOLO
        return red_poles, mask
    """
    """"Used for custom red pole detection if needed, but YOLOv8 should be sufficient.
    # Detect red poles using color thresholding"""
    
    def detect_red_poles(self, frame):
        blurred = cv2.GaussianBlur(frame, (5,5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 80, 50])
        upper_red1 = np.array([12, 255, 255])
        lower_red2 = np.array([168, 80, 50])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_poles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = h / float(w + 1e-5)
            if area > 200 and aspect > 2.0 and w < 100:
                red_poles.append((x, y, w, h))
        return red_poles, mask

    def draw_overlay(self, frame, bbox, distance_m, offset_px):
        if bbox:
            x, y, w, h = bbox
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
            cv2.putText(frame, f"h={h}px", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.putText(frame, f"Dist: {distance_m:.2f}m", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            cv2.putText(frame, f"Offset: {offset_px}px", (x, y+h+55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        cv2.putText(frame, f"Targeting red pole {self.passed_poles+1}/3", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,200,0), 3)
        cv2.putText(frame, f"State: {self.state}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)
        cv2.putText(frame, "(Debug overlays: remove for competition!)", (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,50,50), 1)

    def run(self, frame, target, detections):
        # Set frame shape/focal on first run
        if self.frame_height is None or self.frame_width is None:
            self.frame_height = frame.shape[0]
            self.frame_width = frame.shape[1]
            self.focal_px = (FOCAL_LENGTH_MM / SENSOR_HEIGHT_MM) * self.frame_height
            print(f"[INFO] Frame: {self.frame_width}x{self.frame_height}, focal px: {self.focal_px:.2f}")

        red_poles, mask = self.detect_red_poles(frame)
        bbox = self.get_largest_red_pole(red_poles)
        bbox_h = bbox[3] if bbox else 0
        distance_cm = self.estimate_distance_cm(bbox_h)
        distance_m = distance_cm / 100 if distance_cm else None
        offset_px = self.calc_offset_px(bbox_h)

        # Correct offset sign according to robot_control.py logic!
        if self.side == "left":
            # AUV keeps pole on the left → move right → lateral > 0
            lateral_power = clamp_power(scale_offset_to_power(offset_px))
        else:
            # AUV keeps pole on the right → move left → lateral < 0
            lateral_power = clamp_power(scale_offset_to_power(-offset_px))

        forward_power = clamp_power(APPROACH_FORWARD_POWER)

        # === STATE MACHINE ===
        if self.state == "done":
            motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": True, "pole_idx": self.passed_poles, "state": self.state}
        elif self.state == "search":
            if bbox:
                self.state = "approach"
                """     If we found a pole, switch to approach state.       """
                print(f"[INFO] Red pole {self.passed_poles+1} targeted (search → approach)")
                motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": False, "pole_idx": self.passed_poles, "state": self.state}
            else:
                self.yaw_accum += self.search_yaw_dir * 0.5
                if abs(self.yaw_accum) >= SEARCH_YAW_MAX_DEG:
                    self.search_yaw_dir *= -1                   #either +1 or -1, indicating yaw right or left
                motion = {
                    "forward": 0,
                    "lateral": 0,
                    "yaw": clamp_power(self.search_yaw_dir * SEARCH_YAW_POWER),
                    "end": False,
                    "pole_idx": self.passed_poles,
                    "state": self.state,
                }
        elif self.state == "approach":
            """     Approach the pole diagonally with adaptive offset.       """
            if bbox:
                if bbox_h >= self.frame_height:
                    """when pole's pixel height is larger than frame height, assueme we are close enough, switch to dead reckoning"""
                    print(f"[INFO] Red pole {self.passed_poles+1} passed (approach → dead_reckoning)")
                    self.state = "dead_reckoning"
                    self.dead_reckoning_timer = time.time()
                    motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": False, "pole_idx": self.passed_poles, "state": self.state}
                else:
                    motion = {
                        "forward": forward_power,
                        "lateral": lateral_power,
                        "yaw": 0,
                        "end": False,
                        "pole_idx": self.passed_poles,
                        "state": self.state,
                    }
            else:
                print("[INFO] Lost pole during approach. Switching to search.")
                self.state = "search"
                motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": False, "pole_idx": self.passed_poles, "state": self.state}
        elif self.state == "dead_reckoning":
            """     Perform dead reckoning after skipping a pole.       """
            elapsed = time.time() - self.dead_reckoning_timer
            if elapsed < self.dead_reckoning_time:
                motion = {
                    "forward": clamp_power(1.0),
                    "lateral": 0,
                    "yaw": 0,
                    "end": False,
                    "pole_idx": self.passed_poles,
                    "state": self.state
                }
            else:
                self.passed_poles += 1
                if self.passed_poles >= 3:
                    print("[INFO] All red poles passed, mission done!")
                    self.state = "done"
                    motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": True, "pole_idx": self.passed_poles, "state": self.state}
                else:
                    print(f"[INFO] Executed dead reckoning after pole {self.passed_poles}, scanning for next red pole.")
                    self.state = "search"
                    self.yaw_accum = 0
                    self.search_yaw_dir = 1
                    motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": False, "pole_idx": self.passed_poles, "state": self.state}
        else:
            motion = {"forward": 0, "lateral": 0, "yaw": 0, "end": False, "pole_idx": self.passed_poles, "state": self.state}

        # Debug overlays (remove for competition)
        vis_frame = frame.copy()
        self.draw_overlay(vis_frame, bbox, distance_m or 0, offset_px)

        return motion, vis_frame

if __name__ == "__main__":
    # For testing/unit test purposes only (use your real camera or video file)
    cap = cv2.VideoCapture("C:/Users/huytr/Documents/GitHub/robosub_2025-dev_branch_test/robosub_2025-dev_branch/auv/yolomodel_train/CV_training_data/Poles_2.mp4")
    cv = CV(side="right")  # Change to "left" or "right" if needed

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        result, vis = cv.run(frame, None, None)
        print(f"[INFO] {result}")
        cv2.imshow("Poles Slalom CV", vis)
        if cv2.waitKey(10) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
