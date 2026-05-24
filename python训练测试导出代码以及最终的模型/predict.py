from ultralytics import YOLO

def main():
    model = YOLO("/home/lby/yolo/runs/detect/train7/weights/best.pt")

    model.predict(
        source=0,            # 0=默认摄像头
    conf=0.25,          # 置信度阈值
    show=True,          # 显示窗口
    show_labels=True,   # 显示标签
    show_conf=True,     # 显示置信度
    save=False,         # 不保存
    verbose=False       # 不输出详细信息             
    )

if __name__ == "__main__":
    main()
