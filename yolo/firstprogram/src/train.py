from ultralytics import YOLO

# Load a model
model = YOLO("firstprogram/best.pt")

# Train the model on the dataset for 300 epochs
results = model.train(
    data="/home/xdc/fzsd_robomaster/yolo/firstprogram/datasets/armor.yaml",  # Dataset configuration file
    epochs=200,
    batch=16,
    device='cpu',
    project="/home/xdc/fzsd_robomaster/yolo/firstprogram/runs/train"  # Save directory
)