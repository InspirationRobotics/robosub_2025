from ultralytics import YOLO
import cv2

# Load the YOLO model
model = YOLO(r"c:\Users\chase\Downloads\full_model_with_new_octagon_table.pt")  # Your custom model

# Path to your video file
video_path = r"C:\Users\chase\Downloads\vlc-record-2025-07-16-18h51m55s-rtsp___192.168.2.2_8554_test-.mp4" # Change this to your actual video path
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

while True:
    # Read a frame from the video
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
