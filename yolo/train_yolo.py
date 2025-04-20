from ultralytics import YOLO
from pathlib import Path
import os

# Get the directory where this script is located
repo_root = Path(__file__).parent  # Automatically detects repo root
os.chdir(repo_root)  # Change to that directory

# Verify the working directory
print("Current working directory:", os.getcwd())

# Load a model
model = YOLO("yolo11n.pt")  # load a pretrained model (recommended for training)

results = model.train(data=str(repo_root / "bowling_pin_dataset/data.yaml"), epochs=100, imgsz=640)