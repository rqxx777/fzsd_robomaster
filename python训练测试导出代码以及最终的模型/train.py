from ultralytics import YOLO

def main():
    model = YOLO("/home/lby/yolo/runs/detect/train6/weights/best.pt")

    model.train(
        data="/home/lby/dataset/armor.v4i.yolov8/data.yaml",
        epochs=50,        
        imgsz=960,        
        batch=8,          
        device=0,
        workers=0,        
        resume=False
    )

if __name__ == "__main__":
    main()
