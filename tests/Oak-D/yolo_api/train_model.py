from .trainer import YOLOTrainer

# ----------- CONFIGURATION -----------
YOLO_MODEL_ARCH = "yolov8n.pt"               # Use your chosen base model
YOLO_DATA_YAML = "C:/Users/HOME/Documents/GitHub/CV_data/test model/data.yaml"  # Downloaded from Roboflow
YOLO_EPOCHS = 20
# --------------------------------------

def main():
    print("Training YOLOv8 model...")
    trainer = YOLOTrainer(
        model_arch=YOLO_MODEL_ARCH,
        data_yaml=YOLO_DATA_YAML,
        epochs=YOLO_EPOCHS
    )
    trainer.train_model()
    print("Training complete.")

if __name__ == "__main__":
    main()
