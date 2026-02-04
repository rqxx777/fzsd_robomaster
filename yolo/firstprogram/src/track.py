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

def main():
    import os
    current_dir = os.getcwd()
    print(f"当前工作目录: {current_dir}")
    print(f"期望的目录: /home/xdc/fzsd_robomaster/yolo")
    
    # 检查是否在正确的目录
    if not current_dir.endswith("/yolo"):
        print("警告：可能不在正确的目录运行！")
        print("建议切换到: cd /home/xdc/fzsd_robomaster/yolo")
    
    # 加载训练好的模型 - 使用相对于yolo目录的路径
    model_path = "firstprogram/best.pt"
    print(f"加载模型: {model_path}")
    model = YOLO(model_path)
    
    # 视频路径 - 使用相对于yolo目录的路径
    video_path = "firstprogram/demo_vedio.mp4"
    output_path = "firstprogram/src/detected_demo.mp4"  # 输出到src目录
    
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
    with create_ekf_datalogger(output_dir="firstprogram/src", filename_prefix="ekf_predictions") as data_logger:
        # 存储每个跟踪目标的EKF实例
        ekf_trackers = {}
        
        print("开始EKF预测跟踪...")
        print("EKF预测数据将自动保存到CSV文件中，可用于foxglove可视化")
        
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
                    # 计算边界框中心
                    cx = (box[0] + box[2]) / 2
                    cy = (box[1] + box[3]) / 2
                    
                    # 获取或创建该目标的EKF跟踪器
                    if track_id not in ekf_trackers:
                        ekf_trackers[track_id] = EKF(dt=1.0/fps if fps > 0 else 1.0/30)
                        # 初始化状态
                        ekf_trackers[track_id].state = np.array([cx, cy, 0, 0])
                    
                    # 更新EKF
                    ekf_trackers[track_id].update([cx, cy])
                    
                    # 预测下一帧位置
                    predicted_pos = ekf_trackers[track_id].predict()
                    pred_x, pred_y = predicted_pos
                    
                    # 记录预测数据到CSV文件
                    data_logger.log_prediction(
                        track_id=track_id,
                        pred_x=pred_x,
                        pred_y=pred_y,
                        meas_x=cx,
                        meas_y=cy
                    )
                    
                    # 在帧上绘制预测位置 (红色圆点)
                    cv2.circle(annotated_frame, (int(pred_x), int(pred_y)), 5, (0, 0, 255), -1)
                    
                    # 绘制预测文本
                    cv2.putText(annotated_frame, f"ID:{track_id} Pred", 
                               (int(box[0]), int(box[1]) - 10), 
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
    print(f"EKF预测数据已保存到CSV文件，可用于foxglove可视化")

if __name__ == "__main__":
    main()
