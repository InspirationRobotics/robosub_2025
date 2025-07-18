import cv2
import numpy as np
import os

class CV:
    def __init__(self, **config):
        self.shape = None
        self.config = config
        print("[INFO] Edge Detection CV initialized.")

    def detect_red_edges(self, frame):
        if self.shape is None:
            h, w = frame.shape[:2]
            self.shape = (w, h)

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

    def run_on_frame(self, frame):
        detection, edge_mask = self.detect_red_edges(frame)

        overlay = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        result = cv2.addWeighted(frame, 0.8, overlay, 0.5, 0)

        if detection["status"]:
            x1, y1 = detection["xmin"], detection["ymin"]
            x2, y2 = detection["xmax"], detection["ymax"]
            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(result, f"Area: {detection['area']}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return result, edge_mask

def test_input(input_path):
    image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")
    detector = CV()

    if input_path.lower().endswith(image_extensions):
        frame = cv2.imread(input_path)
        if frame is None:
            print(f"[ERROR] Could not load image: {input_path}")
            return
        result, edge_mask = detector.run_on_frame(frame)
        cv2.imshow("Edge Detection Overlay", result)
        cv2.imshow("Edge Mask", edge_mask)
        print("[INFO] Press any key to exit image display.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = cv2.VideoCapture(input_path)
        if not cap.isOpened():
            print(f"[ERROR] Could not open video/stream: {input_path}")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print("[INFO] End of video or stream error.")
                break

            result, edge_mask = detector.run_on_frame(frame)
            cv2.imshow("Edge Detection Overlay", result)
            cv2.imshow("Edge Mask", edge_mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Quitting...")
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    path = "/Users/avikaprasad/Desktop/pole.png"  # Replace with your image, video, or stream path
    test_input(path)