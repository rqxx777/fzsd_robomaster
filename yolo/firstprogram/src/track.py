import cv2
import numpy as np
from ultralytics import YOLO
from ekf_data_logger import create_ekf_datalogger

class EKF:
    """扩展卡尔曼滤波用于2D目标跟踪"""
    def __init__(self, dt=1.0, process_noise=1.0, measurement_noise=10.0):
        # 状态: [x, y, vx, vy]
        self.state = np.zeros(4)
        self.covariance = np.eye(4) * 100  # 初始协方差
        
        # 状态转移矩阵 (匀速模型)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # 过程噪声协方差
        self.Q = np.eye(4) * process_noise
        
        # 测量矩阵 (只测量位置)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # 测量噪声协方差
        self.R = np.eye(2) * measurement_noise
        
    def predict(self):
        """预测步骤"""
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        return self.state[:2]  # 返回预测位置
    
    def update(self, measurement):
        """更新步骤"""
        # 测量值 (x, y)
        z = np.array(measurement)
        
        # 计算卡尔曼增益
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        # 更新状态和协方差
        y = z - self.H @ self.state
        self.state = self.state + K @ y
        self.covariance = (np.eye(4) - K @ self.H) @ self.covariance
        
        return self.state[:2]

class EKF3D:
    """扩展卡尔曼滤波用于3D目标跟踪"""
    def __init__(self, dt=1.0, process_noise=1.0, measurement_noise=10.0):
        # 状态: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 100  # 初始协方差
        
        # 状态转移矩阵 (匀速模型)
        self.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # 过程噪声协方差
        self.Q = np.eye(6) * process_noise
        
        # 测量矩阵 (测量x, y, z)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # 测量噪声协方差
        self.R = np.eye(3) * measurement_noise
        
    def predict(self):
        """预测步骤"""
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        return self.state[:3]  # 返回预测位置 (x, y, z)
    
    def update(self, measurement):
        """更新步骤"""
        # 测量值 (x, y, z)
        z = np.array(measurement)
        
        # 计算卡尔曼增益
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        # 更新状态和协方差
        y = z - self.H @ self.state
        self.state = self.state + K @ y
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance
        
        return self.state[:3]

def estimate_depth(box, frame_width, frame_height):
    """
    深度估计占位符函数
    实际应用中应替换为真实的深度估计模型（如MiDaS、Depth Anything等）
    或使用立体视觉、传感器融合等方法
    
    Args:
        box: 边界框 [x1, y1, x2, y2]
        frame_width: 帧宽度
        frame_height: 帧高度
        
    Returns:
        float: 估计的深度值（z坐标）
    """
    # 简单启发式：基于边界框大小估计深度（框越大，距离越近）
    box_width = box[2] - box[0]
    box_height = box[3] - box[1]
    box_area = box_width * box_height
    frame_area = frame_width * frame_height
    
    # 假设目标实际大小已知，这里使用简单比例
    # 这里使用一个虚拟常数，实际需要校准
    depth = 1000.0 / (box_area / frame_area + 0.001)  # 虚拟公式
    
    # 限制深度范围
    depth = np.clip(depth, 0.1, 10000.0)
    
    return depth

def main():
    import os
    current_dir = os.getcwd()
    print(f"当前工作目录: {current_dir}")
    
    
    # 加载训练好的模型 - 使用相对于yolo目录的路径
    model_path = "yolo/firstprogram/armor.pt"
    print(f"加载模型: {model_path}")
    model = YOLO(model_path)
    
    # 视频路径 - 使用相对于yolo目录的路径
    video_path = "yolo/firstprogram/demo_vedio.mp4"
    output_path = "yolo/firstprogram/src/detected_demo.mp4" 
    
    print(f"输入视频: {video_path}")
    print(f"输出视频: {output_path}")
    
    # 打开视频
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("无法打开视频文件")
        return
    
    # 获取视频属性
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # 创建VideoWriter保存结果
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # 初始化EKF数据记录器
    with create_ekf_datalogger(output_dir="firstprogram/src", filename_prefix="ekf_predictions", export_rosbag=True) as data_logger:
        # 存储每个跟踪目标的EKF实例
        ekf_trackers = {}
        
        print("开始EKF预测跟踪...")
        print("EKF预测数据将自动保存到CSV和JSON文件中，可用于foxglove可视化")
        print("ROS2 bag (rosbag2) 导出功能已启用，数据将自动保存为.db3格式")
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # 增加帧计数器
            data_logger.increment_frame()
            
            # 使用跟踪模式 (默认使用bytetrack)
            results = model.track(
                source=frame,
                conf=0.3,      # 置信度阈值
                imgsz=640,     # 推理尺寸
                persist=True,  # 保持跟踪ID
                verbose=False
            )
            
            # 获取带标注的帧
            annotated_frame = results[0].plot()
            
            # 如果有检测结果
            if results[0].boxes is not None and results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()  # 边界框 [x1, y1, x2, y2]
                track_ids = results[0].boxes.id.cpu().numpy().astype(int)  # 跟踪ID
                
                for box, track_id in zip(boxes, track_ids):
                    # 计算边界框中心 (2D)
                    cx = (box[0] + box[2]) / 2
                    cy = (box[1] + box[3]) / 2
                    
                    # 估计深度 (z坐标)
                    cz = estimate_depth(box, width, height)
                    
                    # 获取或创建该目标的3D EKF跟踪器
                    if track_id not in ekf_trackers:
                        ekf_trackers[track_id] = EKF3D(dt=1.0/fps if fps > 0 else 1.0/30)
                        # 初始化状态 [x, y, z, vx, vy, vz]
                        ekf_trackers[track_id].state = np.array([cx, cy, cz, 0, 0, 0])
                    
                    # 更新EKF (使用3D测量值)
                    ekf_trackers[track_id].update([cx, cy, cz])
                    
                    # 预测下一帧位置 (3D)
                    predicted_pos = ekf_trackers[track_id].predict()
                    pred_x, pred_y, pred_z = predicted_pos
                    
                    # 记录预测数据到CSV文件 (包含3D坐标)
                    data_logger.log_prediction(
                        track_id=track_id,
                        pred_x=pred_x,
                        pred_y=pred_y,
                        pred_z=pred_z,
                        meas_x=cx,
                        meas_y=cy,
                        meas_z=cz
                    )
                    
                    # 在帧上绘制预测位置 (红色圆点) - 2D投影
                    cv2.circle(annotated_frame, (int(pred_x), int(pred_y)), 5, (0, 0, 255), -1)
                    
                    # 绘制预测文本 (包含深度信息)
                    depth_text = f"Z:{pred_z:.1f}"
                    cv2.putText(annotated_frame, f"ID:{track_id} Pred {depth_text}", 
                    (int(box[0]), int(box[1]) + 20),  # 将文本位置向下移动20像素
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            
            # 显示和保存帧
            cv2.imshow("Armor Plate Detection with EKF Prediction", annotated_frame)
            out.write(annotated_frame)
            
            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    print(f"EKF预测跟踪完成。结果保存到: {output_path}")
    print(f"EKF预测数据已保存到CSV和JSON文件，可用于foxglove可视化")

if __name__ == "__main__":
    main()
