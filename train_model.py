from ultralytics import YOLO

model = YOLO("yolo26n.pt")
results = model.train(
    data=r"/home/ming/Desktop/yolo/armour/data.yaml",
    epochs=300,
    batch=2,
    cache=False,
    imgsz=640)
