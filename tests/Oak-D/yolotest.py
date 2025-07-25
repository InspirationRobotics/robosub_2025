from ultralytics import YOLO
import cv2

# Load the YOLO model
model = YOLO(r"c:\Users\chase\Downloads\full_model_with_new_octagon_table.pt")  # Your custom model

# Open the default webcam (index 0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection on the frame
    results = model(frame)

    # Annotate detections
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLO Detection", annotated_frame)

    # Exit loop on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
