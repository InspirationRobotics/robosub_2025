import os
from roboflow import Roboflow
from ultralytics import YOLO

class RoboflowAPI:
    """
    Handles Roboflow project interaction for uploading datasets and downloading YOLOv8 training code.
    """
    def __init__(self, api_key, workspace, project, version=1):
        self.rf = Roboflow(api_key=api_key)
        self.project = self.rf.workspace(workspace).project(project)
        self.version = self.project.version(version)

    def upload_dataset(self, images_dir, labels_dir):
        """
        Uploads images and labels from the given directories to the Roboflow project.
        """
        # Roboflow expects a zip file in YOLO format
        import shutil
        import zipfile
        import tempfile

        with tempfile.TemporaryDirectory() as temp_dir:
            # Copy images and labels to temp_dir
            images_out = os.path.join(temp_dir, "images")
            labels_out = os.path.join(temp_dir, "labels")
            shutil.copytree(images_dir, images_out)
            shutil.copytree(labels_dir, labels_out)
            zip_path = os.path.join(temp_dir, "upload.zip")
            with zipfile.ZipFile(zip_path, 'w') as zipf:
                for root, _, files in os.walk(temp_dir):
                    for file in files:
                        if file.endswith(".jpg") or file.endswith(".txt"):
                            filepath = os.path.join(root, file)
                            zipf.write(filepath, os.path.relpath(filepath, temp_dir))
            # Upload to Roboflow
            self.version.upload(zip_path, num_retry_uploads=3)
            print("Dataset uploaded to Roboflow.")

    def download_yolov8_code(self, export_dir):
        """
        Downloads YOLOv8-ready dataset code from Roboflow for training.
        """
        dataset = self.version.download("yolov8", location=export_dir)
        print(f"YOLOv8 code and dataset downloaded to {export_dir}.")
        return dataset
