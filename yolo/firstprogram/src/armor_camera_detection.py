import cv2
from ultralytics import YOLO

def main():
    # 1. 加载训练好的YOLO模型
    try:
        model = YOLO("armor2.pt")  # 加载模型
        print("模型加载成功")
    except Exception as e:
        print(f"模型加载失败: {e}")
        return

    # 2. 初始化摄像头
    cap = cv2.VideoCapture(0)  # 0表示默认摄像头
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("实时装甲板检测已启动(按Q键退出)...")

    try:
        while True:
            # 3. 读取摄像头帧
            ret, frame = cap.read()
            if not ret:
                print("无法获取视频帧")
                break

            # 4. 使用模型进行预测
            results = model.predict(
                source=frame,  # 输入帧
                conf=0.5,      # 置信度阈值
                imgsz=640,     # 推理尺寸
                verbose=False  # 不显示详细信息
            )

            # 5. 可视化结果
            annotated_frame = results[0].plot()  # 获取带标注的帧
            cv2.imshow('Armor Detection', annotated_frame)

            # 6. 退出检测
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 7. 释放资源
        cap.release()
        cv2.destroyAllWindows()
        print("程序已退出")

if __name__ == "__main__":
    main()
