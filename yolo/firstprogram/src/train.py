from ultralytics import YOLO

# Load a model
model = YOLO("yolo26n.pt")

# Train the model on the dataset for 300 epochs
results = model.train(
    data="/home/xdc/vscode/armor/datesets/armor.yaml", 
    epochs=300,
    batch=16,
    project="/home/xdc/vscode/runs"  # Save directory
)