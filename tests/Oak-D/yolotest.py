from ultralytics import YOLO
import cv2

# Load model (.pt file)
model = YOLO(r"C:\Users\chase\OneDrive\Desktop\best.pt")  # Replace with your custom model if needed

# Load image
img = cv2.imread(r"C:\Users\chase\OneDrive\Pictures\Screenshots\Screenshot 2025-07-14 144721.png")  # Replace with your image path

# Run detection
results = model(img)

# Visualize results on image
annotated = results[0].plot()

# Show result
cv2.imshow("Detection", annotated)
cv2.waitKey(0)
cv2.destroyAllWindows()
