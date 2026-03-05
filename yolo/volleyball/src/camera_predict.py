from ultralytics import YOLO
import cv2
from PIL import Image
import numpy as np

def main():
   #Load a pretrained YOLO model
    try:
        model = YOLO("")  
        print("模型加载成功")
    except Exception as e:
        print(f"模型加载失败: {e}")
        return

    # 初始化摄像头
    cap = cv2.VideoCapture(0) 
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    try:
        while True:
            # 读取摄像头帧
            ret, frame = cap.read()
            if not ret:
                print("无法获取视频帧")
                break

            # 使用模型进行预测
            results = model.predict(
                source=frame,  # 输入帧
                conf=0.3,      # 置信度阈值
                verbose=True, # 显示详细信息
                show=False
            )

            # 可视化结果
            annotated_frame = results[0].plot()  # 获取带标注的帧
            cv2.imshow('Armor Detection', annotated_frame)

            # 退出检测
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()
        print("程序已退出")

if __name__ == "__main__":
    main()
