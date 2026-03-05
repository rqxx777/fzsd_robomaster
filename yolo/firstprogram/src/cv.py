import cv2
import os
from ultralytics import YOLO

# 创建保存目录
os.makedirs('frames', exist_ok=True)

# 打开视频文件
cap = cv2.VideoCapture(os.path.expanduser('yolo/firstprogram/demo_vedio.mp4'))  # 替换为你的视频文件路径

if not cap.isOpened():
    print("Error: Cannot open video file.")
    exit()

frame_count = 0
saved_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 每60帧保存一张
    if frame_count % 60 == 0:
        cv2.imwrite(f'frames/frame_{frame_count}.jpg', frame)
        saved_count += 1
        print(f"Saved frame {frame_count}")

    frame_count += 1

cap.release()
print(f"Total frames processed: {frame_count}, Saved: {saved_count}")
