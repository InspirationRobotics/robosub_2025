import numpy as np
import cv2
from ultralytics import YOLO
from .bounding_box import BoundingBox 

class ObjectDetector:
    def detect_objects(self, frame):
        """Base class for object detectors."""
        raise NotImplementedError("Subclasses must implement this method.")
        
class HSVDetector(ObjectDetector):
    """Detect objects in an image using HSV color thresholding."""
    def __init__(self, lower_hsv, upper_hsv):
        """
        lower_hsv, upper_hsv: 3-element lists or arrays [H, S, V] 
        Example: lower_hsv = [0, 100, 100], upper_hsv = [10, 255, 255] for red color detection.
        """
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)  # dtype=np.uint8 forces the array to use 8-bit unsigned integers (0-255 range)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)  # required by OpenCV for color thresholding
    def detect_objects(self, frame):
        """
        frame: Frame object as input 
        Returns: list of (x, y, w, h, label)
        """
        #convert BGR to HSV
        hsv = cv2.cvtColor(frame.image_data, cv2.COLOR_BGR2HSV)  # Convert the frame to HSV color space
        
        #create mask
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv) # Create a binary mask where pixels within the HSV range are white (255) and others are black (0)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Find contours in the mask / RETR_EXTERNAL retrieves only the external contours, and CHAIN_APPROX_SIMPLE compresses the contour points for efficiency
        boxes = []  # List to store bounding boxes and labels
        
        # Loop through contours and create bounding boxes
        for cnt in contours:
            area = cv2.contourArea(cnt)                             # Calculate the area of the contour
            if area > 100:                                          # Filter out small contours
                x, y, w, h = cv2.boundingRect(cnt)                  # Get the bounding rectangle for the contour
                boxes.append(BoundingBox(x, y, w, h, "object"))     # Append the bounding box and label to the list
        return boxes                                                # Return the list of bounding boxes
            
class YOLODetector(ObjectDetector):
    def __init__(self, model_path):
        """
        model_path: Path to the YOLOv8 model file(best.pt).
        Example: model_path = "C:/Users/HOME/Documents/GitHub/CV_data/Trained model/yolov8-custom/weights/best.pt"
        """
        self.model = YOLO(model_path)
    def detect_objects(self, frame, conf=0.25):
        """
        frame: Frame object (must have .image_data as a numpy array)
        conf: confidence threshold for detections
        Returns: list of (x, y, w, h, class_label)
        """
        # YOLO expects images as numpy arrays in BGR format
        results = self.model.predict(frame.image_data, conf=conf)           # Run the model on the image data
        boxes = []

        #Loop through the detections (results[0] for first image):
        for box, cls in zip(results[0].boxes.xyxy, results[0].boxes.cls):   #zip() help loop thru coordinates the same time / boxes.xyxy gives the bounding box coordinates in the format [x1, y1, x2, y2] / boxes.cls gives the class IDs for each detection (slalom red, slalom white,etc.)
            x1, y1, x2, y2 = box.cpu().numpy().astype(int)                  # Convert the bounding box coordinates to integers
            w, h = x2 - x1, y2 - y1                                         # Calculate width and height of the bounding box (x1, y1) is the top-left corner and (x2, y2) is the bottom-right corner)
            class_id = int(cls)
            #get class name from model 
            class_label = self.model.model.names[class_id] if hasattr(self.model.model, 'names') else str(class_id) # Get the class label from the model's names attribute, if available; otherwise, use the class ID as a string (0, 1, 2, etc.)
            boxes.append(BoundingBox(x1, y1, w, h, class_label))            # Append the bounding box and class label to the list
        return boxes
