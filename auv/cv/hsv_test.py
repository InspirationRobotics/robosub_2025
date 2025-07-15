"""
Pole Slalom CV Viewer.
Detects red poles in a video or image using HSV and shows bounding boxes.
"""

import cv2
import numpy as np
import os

class CV:
    def __init__(self, source_path, **config):
        self.source_path = source_path
        self.x_midpoint = 2880 / 2
        self.tolerance = 40
        self.config = config
        print("[INFO] Initialized with:", self.source_path)

    def detect_red_pole(self, frame):
        height, width = frame.shape[:2]
        print(height, width)
        cropped_frame = frame[0:min(1300, height), 0:min(2880, width),]
        bgr_frame = cropped_frame[:, :, :3]
        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([20, 0, 0])
        upper_red1 = np.array([255, 255, 50])
        # lower_red2 = np.array([170, 120, 70])
        # upper_red2 = np.array([180, 255, 255])

        red_mask = cv2.inRange(hsv, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # red_mask = cv2.bitwise_or(mask1, mask2)

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

    def process_image(self, image_path):
        image = cv2.imread(image_path)
        if image is None:
            print("[ERROR] Could not read image:", image_path)
            return
        detection, mask = self.detect_red_pole(image)
        if detection["status"]:
            cv2.rectangle(image, (detection["xmin"], detection["ymin"]),
                          (detection["xmax"], detection["ymax"]), (0, 255, 0), 2)
            cv2.putText(image, f"Area: {detection['area']}", (detection["xmin"], detection["ymin"] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Image Detection", image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def process_video(self):
        cap = cv2.VideoCapture(self.source_path)
        if not cap.isOpened():
            print("[ERROR] Failed to open video file:", self.source_path)
            return

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            detection, mask = self.detect_red_pole(frame)
            if detection["status"]:
                cv2.rectangle(frame, (detection["xmin"], detection["ymin"]),
                              (detection["xmax"], detection["ymax"]), (0, 255, 0), 2)
                cv2.putText(frame, f"Area: {detection['area']}", (detection["xmin"], detection["ymin"] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Red Pole Detection", frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def run(self):
        ext = os.path.splitext(self.source_path)[1].lower()
        if ext in [".jpg", ".jpeg", ".png", ".bmp"]:
            self.process_image(self.source_path)
        elif ext in [".mp4", ".avi", ".mov", ".mkv"]:
            self.process_video()
        else:
            print("[ERROR] Unsupported file type:", ext)


if __name__ == "__main__":
    # path = input("Enter path to image or video file: ").strip()
    path = "/Users/avikaprasad/Downloads/poles.mp4"
    cv = CV(path)
    cv.run()