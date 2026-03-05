from ultralytics import YOLO

#Load a pretrained YOLO model
model = YOLO("yolo/volleyball/yolo26n.pt")

#Train a new YOLO model on your own dataset
results = model.train(
    data="yolo/volleyball/datasets/data.yaml",
    epochs=100,
    batch=16,
    imgsz=416,
    device='cpu',
    project="yolo/volleyball/runs/train",
    name="exp")