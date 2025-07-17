import os
from .bounding_box import BoundingBox
import cv2
class LabelExporter:
    """
    Handles saving images and YOLO-format labels for object detection datasets.
    """

    def __init__(self, images_dir, labels_dir):
        """
        Args:
            images_dir: Folder to save images.
            labels_dir: Folder to save YOLO label .txt files.
        """
        self.images_dir = images_dir
        self.labels_dir = labels_dir
        os.makedirs(images_dir, exist_ok=True)
        os.makedirs(labels_dir, exist_ok=True)

    def export_images_and_labels(self, frames, class_map):
        """
        Args:
            frames: List of Frame objects. Each should have .image_data and .bounding_boxes.
            class_map: Dict mapping class label (str) to class id (int), e.g., {"pole": 0, "person": 1}
        """
        for frame in frames:
            # Save image
            image_filename = f"frame_{frame.frame_number:04d}.jpg"
            label_filename = f"frame_{frame.frame_number:04d}.txt"
            image_path = os.path.join(self.images_dir, image_filename)
            label_path = os.path.join(self.labels_dir, label_filename)
            cv2.imwrite(image_path, frame.image_data)
            # Save label file in YOLO format
            if hasattr(frame, 'bounding_boxes'):
                with open(label_path, 'w') as f:
                    for box in frame.bounding_boxes:
                        if box.label not in class_map:
                            continue  # Skip unknown class
                        class_id = class_map[box.label]
                        # Set image size for normalization
                        box.image_width = frame.image_data.shape[1]
                        box.image_height = frame.image_data.shape[0]
                        f.write(box.to_yolo_format(class_id) + "\n")
            else:
                open(label_path, 'w').close()  # Create empty file if no detections
