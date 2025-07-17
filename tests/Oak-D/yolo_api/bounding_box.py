class BoundingBox:
    """
    Represents a bounding box with coordinates, class label, and optional confidence.
    Provides utilities for export (YOLO format), drawing, and normalization.
    """ 

    def __init__(self, x, y, w, h, label, confidence=None, image_width=None, image_height=None):
        """
        Args:
            x, y       : Top-left corner (pixel)
            w, h       : Width and height (pixel)
            label      : Class label (string or int)
            confidence : Optional detection confidence (float)
            image_width, image_height : Dimensions of image (for normalization)
        """
        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)
        self.label = label
        self.confidence = confidence
        self.image_width = image_width
        self.image_height = image_height

    def to_yolo_format(self, class_id):
        """
        Converts the bounding box to YOLO TXT label format (class_id, center_x, center_y, width, height).
        Returns: String (e.g. "0 0.5 0.5 0.1 0.2")
        """
        assert self.image_width and self.image_height, "Image size must be set for normalization" #assert to ensure image dimensions are provided for normalization
        # YOLO requires center coordinates and width/height, all normalized to [0, 1]
        x_center = (self.x + self.w / 2) / self.image_width  # Center x coordinate normalized
        y_center = (self.y + self.h / 2) / self.image_height # Center y coordinate normalized
        w_norm = self.w / self.image_width                   # Width normalized
        h_norm = self.h / self.image_height                  # Height normalized 
        return f"{class_id} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}"    # Format: "class_id center_x center_y width height"

    def export_label(self, class_id, label_path): #use for saving YOLO labels for training / drawing result for visual debugging / calculating overlap between predictions and ground truth
        """
        Saves the YOLO-format label for this bounding box to a .txt file.
        Args:
            class_id   : Integer class index for YOLO label.
            label_path : Path to save .txt label (appends if file exists).
        """
        label_line = self.to_yolo_format(class_id)
        with open(label_path, "a") as f:
            f.write(label_line + "\n")

    def draw_on_image(self, image, color=(0,255,0), thickness=2, show_label=True):
        """
        Draws the bounding box on the given image (in-place).
        Args:
            image     : Image (numpy array, BGR)
            color     : Box color (B, G, R)
            thickness : Box border thickness
            show_label: Whether to show label text on the box
        """
        import cv2
        top_left = (self.x, self.y)
        bottom_right = (self.x + self.w, self.y + self.h)
        cv2.rectangle(image, top_left, bottom_right, color, thickness)
        if show_label and self.label is not None:
            text = f"{self.label}"
            if self.confidence is not None:
                text += f" {self.confidence:.2f}"
            cv2.putText(
                image, text, (self.x, self.y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, lineType=cv2.LINE_AA
            )

    def area(self):
        """Returns the area (pixels^2) of the bounding box."""
        return self.w * self.h

    def iou(self, other): #to evaluate accuracy: high overlap (high IoU) means good prediction
        """
        Computes Intersection over Union (IoU) with another bounding box.
        Args:
            other : BoundingBox
        Returns:
            IoU value (float in [0,1])
        """
        # Determine coordinates of intersection rectangle
        xi1 = max(self.x, other.x)
        yi1 = max(self.y, other.y)
        xi2 = min(self.x + self.w, other.x + other.w)
        yi2 = min(self.y + self.h, other.y + other.h)
        inter_width = max(xi2 - xi1, 0)
        inter_height = max(yi2 - yi1, 0)
        inter_area = inter_width * inter_height
        union_area = self.area() + other.area() - inter_area
        return inter_area / union_area if union_area != 0 else 0.0

    def as_tuple(self):
        """Returns (x, y, w, h, label) as a tuple."""
        return (self.x, self.y, self.w, self.h, self.label)
