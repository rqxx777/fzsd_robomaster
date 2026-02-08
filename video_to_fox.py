import cv2
import foxglove
import time
import numpy as np
from foxglove.schemas import RawImage, Timestamp
from foxglove.channels import RawImageChannel

# === 1. 设置视频路径和Foxglove连接 ===
video_path = "/home/ming/Desktop/yolo/tracking_output.mp4"  # 替换为你的MP4路径
server = foxglove.start_server()
image_channel = RawImageChannel("/camera_video")  # 定义话题

# === 2. 打开视频文件 ===
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("错误：无法打开视频文件！")
    exit()

# 获取视频的原始帧率，用于控制播放速度
fps = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = 30  # 如果获取失败，使用默认帧率
frame_delay = 1.0 / fps

print(f"开始发送视频: {video_path}, 帧率: {fps:.2f} FPS")

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("视频播放完毕。")
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = frame_rgb.shape[:2]

        # 构建Timestamp对象
        current_ns = int(time.time() * 1e9)
        timestamp_obj = Timestamp(sec=current_ns // 1_000_000_000,
                                  nsec=current_ns % 1_000_000_000)

        # 构建并发送RawImage消息
        message = RawImage(
            timestamp=timestamp_obj,
            frame_id="camera",
            width=width,
            height=height,
            encoding="rgb8",
            step=width * 3,
            data=frame_rgb.tobytes()
        )
        image_channel.log(message)

        # 根据视频帧率控制发送间隔，模拟实时播放
        time.sleep(frame_delay)

except KeyboardInterrupt:
    print("\n用户中断发送。")
finally:
    cap.release()
    print("视频发送结束。")