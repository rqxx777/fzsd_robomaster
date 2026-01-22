from ultralytics import YOLO

model = YOLO("/home/lby/yolo/runs/detect/train7/weights/best.pt")  
model.export(format="onnx", opset=12)
