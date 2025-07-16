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
            print(f"[INFO] Frame shape: width={w}, height={h}")

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

    def run_on_frame(self, frame):
        edge_mask, cropped = self.detect_red_edges(frame)

        # Overlay edges on the cropped original
        overlay = cv2.cvtColor(edge_mask, cv2.COLOR_GRAY2BGR)
        result = cv2.addWeighted(cropped, 0.8, overlay, 0.5, 0)

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
    path = "/Users/avikaprasad/Desktop/pole_main.png"  # Can be image, video, or RTSP stream
    test_input(path)