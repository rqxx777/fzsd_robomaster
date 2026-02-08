import foxglove
import time
import numpy as np
# 关键：导入所需的Schema和类
from foxglove.schemas import RawImage, Timestamp
from foxglove.channels import RawImageChannel

server = foxglove.start_server()
print("服务器已启动，开始持续发送测试图像...")

# 创建测试图像
height, width = 100, 100
red_channel = np.full((height, width), 255, dtype=np.uint8)
green_channel = np.full((height, width), 50, dtype=np.uint8)
blue_channel = np.full((height, width), 50, dtype=np.uint8)
test_image = np.stack([red_channel, green_channel, blue_channel], axis=-1)

# 创建RawImage数据通道
image_channel = RawImageChannel("/test_image")

try:
    while True:
        # 1. 创建 Timestamp 对象（纳秒）
        current_ns = int(time.time() * 1e9)
        timestamp_obj = Timestamp(sec=current_ns // 1_000_000_000,
                                  nsec=current_ns % 1_000_000_000)
        
        # 2. 使用Timestamp对象和其他参数构建RawImage消息
        message = RawImage(
            timestamp=timestamp_obj,  # 使用Timestamp对象，而非整数
            frame_id="test_frame",
            width=width,
            height=height,
            encoding="rgb8",
            step=width * 3,
            data=test_image.tobytes()  # 确保是bytes
        )
        
        # 3. 通过专用Channel发送
        image_channel.log(message)
        
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\n用户中断，停止发送。")