"""This script tests a trained YOLOv8 model on a video file.
It loads the model, runs object detection on the video,
and saves the output video with detected objects highlighted.
Afterward, it plays back the output video in a pop-up window."""

from ultralytics import YOLO
import cv2
import os

# --- Edit these paths as needed ---
MODEL_PATH = 'C:/Users/HOME/Documents/GitHub/robosub_2025-dev_branch_test/robosub_2025-dev_branch/runs/train/yolov8_custom2/weights/best.pt'
VIDEO_PATH = 'C:/Users/HOME/Documents/GitHub/CV_data/20250704_all_2.mp4'

# 1. Run detection and save the output video
model = YOLO(MODEL_PATH)
results = model.predict(
    source=VIDEO_PATH,
    save=True,           # Save the output video with boxes drawn
    conf=0.25,           # Confidence threshold
    show=True            # Show live inference if your environment supports it
)

print("Detection done. Now playing the output video...")

# 2. Find the output video path (YOLO saves in runs/detect/predict/ by default)
filename = os.path.basename(VIDEO_PATH)
output_video_path = os.path.join('runs', 'detect', 'predict', filename)
if not os.path.exists(output_video_path):
    # Sometimes YOLO makes new folders like 'predict2' for subsequent runs
    detect_dir = os.path.join('runs', 'detect')
    subdirs = sorted([d for d in os.listdir(detect_dir) if d.startswith('predict')], reverse=True)
    for sub in subdirs:
        possible_path = os.path.join(detect_dir, sub, filename)
        if os.path.exists(possible_path):
            output_video_path = possible_path
            break

print(f"Output video: {output_video_path}")

# 3. Play the saved video
cap = cv2.VideoCapture(output_video_path)
if not cap.isOpened():
    print("Failed to open output video for playback.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow('YOLOv8 Output Video', frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            print("Playback interrupted by user.")
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Playback finished.")


