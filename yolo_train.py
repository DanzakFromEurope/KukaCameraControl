from ultralytics import YOLO
import os
import torch

def train_model():
    # Check if MPS is available
    if torch.backends.mps.is_available():
        device = 'mps'
        print("Using Apple MPS acceleration!")
    elif torch.cuda.is_available():
        device = 0
        print("Using NVIDIA GPU acceleration!")
    else:
        device = 'cpu'
        print("Using CPU (might be slow).")

    # 1. Load the base model (Nano size, OBB version)
    model = YOLO('yolov8n-obb.pt')

    # 2. Define path to your data.yaml
    data_file = os.path.abspath("cube_dataset/data.yaml")

    # 3. Train
    # device=device passes 'mps' to the trainer
    results = model.train(data=data_file, epochs=100, imgsz=640, device=device)

    print("Training done!")

if __name__ == '__main__':
    train_model()
