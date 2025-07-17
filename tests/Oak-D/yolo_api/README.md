# YOLO Object Detection Pipeline

A modular Python framework to automate the process of extracting frames from videos, labeling objects (using HSV or YOLO models), exporting YOLO-format datasets, and training YOLOv8 models. Supports seamless integration with Roboflow.

---

## Features

- Split videos into frames for dataset creation
- Automatic object detection using HSV color or YOLOv8 models
- Bounding box annotation and YOLO-format label export
- Batch dataset upload/download to/from Roboflow
- End-to-end YOLOv8 model training pipeline in Python

---

## Folder Structure

yolo_api/
│
├── video.py # Video and Frame classes
├── detectors.py # ObjectDetector, HSVDetector, YOLODetector
├── bounding_box.py # BoundingBox class
├── exporter.py # LabelExporter class
├── roboflow_api.py # RoboflowAPI class
├── trainer.py # YOLOTrainer class
├
├── run_detection_and_export.py 
├── train_model.py