import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
from ultralytics import YOLO
import os

class VisualEKF:
    """
    基于EKF的视觉目标跟踪器，使用恒定加速度模型
    通过增大过程噪声Q来处理非恒定加速度
    """
    
    def __init__(self, dt=1.0, use_adaptive_Q=False):
        """
        初始化EKF跟踪器
        
        Args:
            dt: 时间步长（秒/帧）
            use_adaptive_Q: 是否启用自适应Q（根据速度动态调整）
        """
        self.dt = dt
        self.use_adaptive_Q = use_adaptive_Q
        
        # ==================== 状态定义 ====================
        # 状态向量: [cx, cy, vx, vy, ax, ay, w, h]^T
        # cx,cy: 边界框中心坐标 (像素)
        # vx,vy: 中心点速度 (像素/帧)
        # ax,ay: 中心点加速度 (像素/帧²)
        # w, h: 边界框宽度和高度 (像素)
        self.dim_state = 8  # 状态维度
        self.dim_obs = 4    # 观测维度 [cx, cy, w, h]
        
        # ==================== EKF参数初始化 ====================
        # 状态转移矩阵 F (8x8) - 恒定加速度模型
        self.F = self._create_F_matrix(dt)
        
        # 观测矩阵 H (4x8) - 我们观测位置和尺寸
        self.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],  # 观测 cx
            [0, 1, 0, 0, 0, 0, 0, 0],  # 观测 cy
            [0, 0, 0, 0, 0, 0, 1, 0],  # 观测 w
            [0, 0, 0, 0, 0, 0, 0, 1]   # 观测 h
        ])
        
        # ==================== 噪声协方差矩阵 ====================
        # 加速度变化越剧烈，Q[4,4]和Q[5,5]应该越大
        
        # 过程噪声协方差 Q (8x8)
        # 对角线元素分别对应: [cx, cy, vx, vy, ax, ay, w, h]的噪声方差
        # 加速度(ax, ay)的噪声方差设置得比较大

        self.Q_base = np.diag([
            0.1,    # cx 位置噪声
            0.1,    # cy 位置噪声
            1.0,    # vx 速度噪声
            1.0,    # vy 速度噪声
            100.0,  # ax 加速度噪声
            100.0,  # ay 加速度噪声
            0.01,   # w  宽度噪声
            0.01    # h  高度噪声
        ])
        
        # 观测噪声协方差 R (4x4)
        # 描述YOLO检测的不确定性：位置噪声 > 尺寸噪声
        self.R = np.diag([
            25.0,   # cx 观测噪声 (像素)
            25.0,   # cy 观测噪声 (像素)
            4.0,    # w  观测噪声 (像素)
            4.0     # h  观测噪声 (像素)
        ])
        
        # 状态估计协方差 P (8x8) - 初始不确定性
        self.P = np.diag([
            50.0,   # cx 初始不确定性
            50.0,   # cy 初始不确定性
            25.0,   # vx 初始不确定性
            25.0,   # vy 初始不确定性
            100.0,  # ax 初始不确定性
            100.0,  # ay 初始不确定性
            10.0,   # w  初始不确定性
            10.0    # h  初始不确定性
        ])
        
        # 状态向量
        self.x = np.zeros(self.dim_state)
        
        # 跟踪状态
        self.is_initialized = False
        self.miss_count = 0
        self.max_miss_frames = 10  # 最大允许丢失帧数
        
        # 历史记录
        self.state_history = []
        self.observation_history = []
        self.prediction_history = []
        
    def _create_F_matrix(self, dt):
        """创建状态转移矩阵 F（恒定加速度模型）"""
        F = np.eye(self.dim_state)
        
        # 位置更新: p_new = p + v*dt + 0.5*a*dt^2
        F[0, 2] = dt        # cx 受 vx 影响
        F[0, 4] = 0.5*dt**2 # cx 受 ax 影响
        F[1, 3] = dt        # cy 受 vy 影响
        F[1, 5] = 0.5*dt**2 # cy 受 ay 影响
        
        # 速度更新: v_new = v + a*dt
        F[2, 4] = dt        # vx 受 ax 影响
        F[3, 5] = dt        # vy 受 ay 影响
        
        # 宽度和高度：假设变化缓慢（对角线元素已为1）
        
        return F
    
    def _get_adaptive_Q(self, speed):
        """
        根据当前速度自适应调整Q矩阵
        速度越快，模型不确定性越大，需要更大的Q
        
        Args:
            speed: 当前估计的速度幅值 (像素/帧)
        """
        Q = self.Q_base.copy()
        
        if self.use_adaptive_Q and speed > 0:
            # 速度越快，加速度噪声越大
            speed_factor = min(3.0, 1.0 + speed / 50.0)
            Q[4, 4] = self.Q_base[4, 4] * speed_factor
            Q[5, 5] = self.Q_base[5, 5] * speed_factor
            
            # 速度越快，位置和速度噪声也适当增加
            Q[0, 0] = self.Q_base[0, 0] * (1.0 + speed / 100.0)
            Q[1, 1] = self.Q_base[1, 1] * (1.0 + speed / 100.0)
            Q[2, 2] = self.Q_base[2, 2] * (1.0 + speed / 50.0)
            Q[3, 3] = self.Q_base[3, 3] * (1.0 + speed / 50.0)
        
        return Q
    
    def initialize(self, initial_observation):
        """
        使用第一帧观测初始化EKF
        
        Args:
            initial_observation: [cx, cy, w, h]
        """
        if initial_observation is None:
            raise ValueError("初始观测不能为None")
        
        cx, cy, w, h = initial_observation
        
        # 初始化状态：位置已知，速度加速度设为0，尺寸已知
        self.x = np.array([cx, cy, 0, 0, 0, 0, w, h], dtype=np.float64)
        
        # 记录初始观测
        self.observation_history.append(initial_observation)
        self.state_history.append(self.x.copy())
        
        self.is_initialized = True
        print(f"EKF已初始化: 位置({cx:.1f}, {cy:.1f}), 尺寸({w:.1f}x{h:.1f})")
    
    def predict(self):
        """EKF预测步骤"""
        if not self.is_initialized:
            return None
        
        # 计算当前速度（用于自适应Q）
        speed = np.sqrt(self.x[2]**2 + self.x[3]**2)
        
        # 获取过程噪声（可能自适应调整）
        Q = self._get_adaptive_Q(speed)
        
        # 状态预测: x_pred = F * x
        x_pred = self.F @ self.x
        
        # 协方差预测: P_pred = F * P * F^T + Q
        P_pred = self.F @ self.P @ self.F.T + Q
        
        # 保存预测值
        self.prediction_history.append(x_pred.copy())
        
        return x_pred, P_pred
    
    def update(self, observation):
        """
        EKF更新步骤
        
        Args:
            observation: [cx, cy, w, h] 或 None（如果检测失败）
        Returns:
            更新后的状态估计
        """
        if not self.is_initialized:
            return None
        
        # 预测步骤
        x_pred, P_pred = self.predict()
        
        # 保存预测值到状态变量（如果更新失败，则使用预测值）
        self.x = x_pred
        self.P = P_pred
        self.miss_count += 1
        
        # 如果没有观测，只进行预测
        if observation is None:
            # 连续丢失太多帧，重置跟踪器
            if self.miss_count > self.max_miss_frames:
                print("警告：目标丢失超过最大帧数，跟踪可能已失效")
            return self.x
        
        # 有观测值，进行更新
        self.miss_count = 0
        self.observation_history.append(observation)
        
        # 观测向量
        z = np.array(observation, dtype=np.float64)
        
        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 计算观测残差（innovation）
        y = z - self.H @ self.x
        
        # 状态更新
        self.x = self.x + K @ y
        
        # 协方差更新 (Joseph形式，数值更稳定)
        I = np.eye(self.dim_state)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T
        
        # 保存更新后的状态
        self.state_history.append(self.x.copy())
        
        return self.x
    
    def get_state(self):
        """获取当前状态估计"""
        if not self.is_initialized:
            return None
        
        # 返回：位置(cx, cy), 速度(vx, vy), 加速度(ax, ay), 尺寸(w, h)
        return {
            'position': (self.x[0], self.x[1]),
            'velocity': (self.x[2], self.x[3]),
            'acceleration': (self.x[4], self.x[5]),
            'size': (self.x[6], self.x[7]),
            'speed': np.sqrt(self.x[2]**2 + self.x[3]**2),
            'accel_magnitude': np.sqrt(self.x[4]**2 + self.x[5]**2)
        }
    
    def get_trajectory(self):
        """获取完整轨迹历史"""
        if len(self.state_history) == 0:
            return []
        
        return np.array(self.state_history)


def extract_observations_from_video(video_path, model_path, target_class=0, conf_threshold=0.5):
    """
    使用YOLO模型从视频中提取目标观测
    
    Args:
        video_path: 视频文件路径
        model_path: YOLO模型权重路径
        target_class: 要跟踪的目标类别ID
        conf_threshold: 置信度阈值
    
    Returns:
        observations: 每帧的观测列表，每个元素为 [cx, cy, w, h] 或 None
        frames: 视频帧列表（用于可视化）
    """
    print(f"正在从视频中提取观测: {video_path}")
    
    # 加载YOLO模型
    model = YOLO(model_path)
    
    # 打开视频
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"无法打开视频文件: {video_path}")
    
    observations = []
    frames = []
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame_count += 1
        frames.append(frame)
        
        # 运行YOLO检测
        results = model(frame, conf=conf_threshold, verbose=False)[0]
        
        # 查找目标类别的检测
        target_detection = None
        if results.boxes is not None and len(results.boxes) > 0:
            for i in range(len(results.boxes)):
                cls_id = int(results.boxes.cls[i])
                conf = float(results.boxes.conf[i])
                
                if cls_id == target_class and conf >= conf_threshold:
                    # 获取边界框
                    x1, y1, x2, y2 = map(float, results.boxes.xyxy[i])
                    
                    # 计算中心点、宽度和高度
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    w = x2 - x1
                    h = y2 - y1
                    
                    target_detection = [cx, cy, w, h]
                    break
        
        observations.append(target_detection)
        
        # 每处理50帧打印一次进度
        if frame_count % 50 == 0:
            print(f"  已处理 {frame_count} 帧...")
    
    cap.release()
    print(f"观测提取完成，共 {frame_count} 帧")
    print(f"有效检测帧数: {sum(1 for obs in observations if obs is not None)}")
    
    return observations, frames


def visualize_tracking_results(frames, observations, ekf_states, output_path="tracking_result.mp4"):
    """
    可视化跟踪结果并保存为视频
    
    Args:
        frames: 原始视频帧列表
        observations: 原始观测列表
        ekf_states: EKF估计的状态历史
        output_path: 输出视频路径
    """
    print("正在生成可视化结果...")
    
    if len(frames) == 0:
        print("错误：没有视频帧")
        return
    
    # 获取视频参数
    height, width = frames[0].shape[:2]
    fps = 30  # 假设30fps
    
    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # 颜色定义
    COLOR_OBSERVATION = (0, 0, 255)     # 红色: 原始观测
    COLOR_EKF_ESTIMATE = (0, 255, 0)    # 绿色: EKF估计
    COLOR_TRAJECTORY = (255, 255, 0)    # 青色: 轨迹
    
    # 轨迹点存储
    trajectory_points = []
    
    for i, frame in enumerate(frames):
        # 复制原始帧
        display_frame = frame.copy()
        
        # 获取当前帧的原始观测
        obs = observations[i] if i < len(observations) else None
        
        # 获取当前帧的EKF估计状态
        ekf_state = ekf_states[i] if i < len(ekf_states) else None
        
        # 绘制原始观测（如果存在）
        if obs is not None:
            cx, cy, w, h = obs
            x1, y1 = int(cx - w/2), int(cy - h/2)
            x2, y2 = int(cx + w/2), int(cy + h/2)
            
            # 绘制观测框
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), COLOR_OBSERVATION, 2)
            cv2.putText(display_frame, f"Obs", (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_OBSERVATION, 2)
        
        # 绘制EKF估计（如果存在）
        if ekf_state is not None:
            cx_est, cy_est = ekf_state[0], ekf_state[1]
            w_est, h_est = ekf_state[6], ekf_state[7]
            x1_est, y1_est = int(cx_est - w_est/2), int(cy_est - h_est/2)
            x2_est, y2_est = int(cx_est + w_est/2), int(cy_est + h_est/2)
            
            # 绘制EKF估计框
            cv2.rectangle(display_frame, (x1_est, y1_est), (x2_est, y2_est), COLOR_EKF_ESTIMATE, 2)
            
            # 绘制估计位置点
            #cv2.circle(display_frame, (int(cx_est), int(cy_est)), 5, COLOR_EKF_ESTIMATE, -1)
            
            # 显示速度和加速度信息
            vx, vy = ekf_state[2], ekf_state[3]
            ax, ay = ekf_state[4], ekf_state[5]
            speed = np.sqrt(vx**2 + vy**2)
            accel = np.sqrt(ax**2 + ay**2)
            
            info_text = f"EKF: v={speed:.1f}, a={accel:.1f}"
            cv2.putText(display_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_EKF_ESTIMATE, 2)
            
            # 保存轨迹点
            
            #trajectory_points.append((int(cx_est), int(cy_est)))
        
        # 绘制轨迹
        #for j in range(1, len(trajectory_points)):
            #cv2.line(display_frame, trajectory_points[j-1], trajectory_points[j], 
                    #COLOR_TRAJECTORY, 2)
        
        # 显示帧计数器
        cv2.putText(display_frame, f"Frame: {i+1}/{len(frames)}", (10, height-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 写入视频帧
        out.write(display_frame)
        
        # 实时显示（按q退出）
        cv2.imshow("EKF Tracking", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    out.release()
    cv2.destroyAllWindows()
    print(f"可视化结果已保存到: {output_path}")


def plot_tracking_statistics(observations, ekf_states):
    """
    绘制跟踪统计图表
    
    Args:
        observations: 原始观测列表
        ekf_states: EKF估计的状态历史
    """
    # 转换为numpy数组
    ekf_array = np.array(ekf_states)
    
    # 提取观测值（只处理有效观测）
    obs_cx = [obs[0] for obs in observations if obs is not None]
    obs_cy = [obs[1] for obs in observations if obs is not None]
    
    # 创建图表
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    
    # 1. XY轨迹图
    axes[0, 0].plot(obs_cx, obs_cy, 'ro', markersize=3, alpha=0.5, label='原始观测')
    axes[0, 0].plot(ekf_array[:, 0], ekf_array[:, 1], 'g-', linewidth=2, label='EKF估计')
    axes[0, 0].set_xlabel('X位置 (像素)')
    axes[0, 0].set_ylabel('Y位置 (像素)')
    axes[0, 0].set_title('目标运动轨迹')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # 2. X位置随时间变化
    frame_numbers = range(len(ekf_array))
    axes[0, 1].plot(frame_numbers, ekf_array[:, 0], 'b-', linewidth=2, label='X位置')
    axes[0, 1].set_xlabel('帧数')
    axes[0, 1].set_ylabel('X位置 (像素)')
    axes[0, 1].set_title('X位置随时间变化')
    axes[0, 1].grid(True)
    
    # 3. Y位置随时间变化
    axes[1, 0].plot(frame_numbers, ekf_array[:, 1], 'r-', linewidth=2, label='Y位置')
    axes[1, 0].set_xlabel('帧数')
    axes[1, 0].set_ylabel('Y位置 (像素)')
    axes[1, 0].set_title('Y位置随时间变化')
    axes[1, 0].grid(True)
    
    # 4. 速度幅值随时间变化
    speed = np.sqrt(ekf_array[:, 2]**2 + ekf_array[:, 3]**2)
    axes[1, 1].plot(frame_numbers, speed, 'g-', linewidth=2, label='速度')
    axes[1, 1].set_xlabel('帧数')
    axes[1, 1].set_ylabel('速度 (像素/帧)')
    axes[1, 1].set_title('目标速度变化')
    axes[1, 1].grid(True)
    
    # 5. 加速度幅值随时间变化
    acceleration = np.sqrt(ekf_array[:, 4]**2 + ekf_array[:, 5]**2)
    axes[2, 0].plot(frame_numbers, acceleration, 'm-', linewidth=2, label='加速度')
    axes[2, 0].set_xlabel('帧数')
    axes[2, 0].set_ylabel('加速度 (像素/帧²)')
    axes[2, 0].set_title('目标加速度变化')
    axes[2, 0].grid(True)
    
    # 6. 边界框尺寸变化
    axes[2, 1].plot(frame_numbers, ekf_array[:, 6], 'c-', linewidth=2, label='宽度')
    axes[2, 1].plot(frame_numbers, ekf_array[:, 7], 'y-', linewidth=2, label='高度')
    axes[2, 1].set_xlabel('帧数')
    axes[2, 1].set_ylabel('尺寸 (像素)')
    axes[2, 1].set_title('边界框尺寸变化')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('tracking_statistics.png', dpi=150)
    plt.show()


def main():
    """主函数：演示EKF视觉跟踪流程"""
    print("=== EKF视觉目标跟踪系统（增大Q处理非恒定加速度）===")
    
    # ==================== 配置参数 ====================
    VIDEO_PATH = r"/home/ming/Desktop/yolo/armour/Video_1768570695685_454_1(1).avi"  # 替换为你的视频路径
    MODEL_PATH = r"/home/ming/Desktop/yolo/runs/detect/train3/weights/best.pt"   # 替换为你的YOLO模型路径
    TARGET_CLASS = 0               # 要跟踪的目标类别ID
    
    # ==================== 步骤1: 提取观测 ====================
    print("\n步骤1: 从视频中提取观测...")
    
    # 如果没有实际视频，创建一个模拟观测序列用于演示
    use_simulation = not os.path.exists(VIDEO_PATH)
    
    if use_simulation:
        print("未找到视频文件，使用模拟数据...")
        # 创建模拟观测序列（圆周运动 + 噪声 + 部分丢失）
        num_frames = 200
        t = np.linspace(0, 4*np.pi, num_frames)
        
        # 模拟轨迹：变加速圆周运动
        center_x, center_y = 320, 240
        radius = 100
        cx = center_x + radius * np.sin(t) + 5 * np.sin(2*t)  # 添加次级振动
        cy = center_y + radius * np.cos(t) + 5 * np.cos(3*t)
        
        # 添加随机丢失（模拟10%的帧丢失目标）
        np.random.seed(42)
        observations = []
        for i in range(num_frames):
            if np.random.random() < 0.9:  # 90%的检测成功率
                # 添加观测噪声
                noise_cx = np.random.normal(0, 8)
                noise_cy = np.random.normal(0, 8)
                w = 60 + np.random.normal(0, 3)
                h = 40 + np.random.normal(0, 3)
                observations.append([cx[i] + noise_cx, cy[i] + noise_cy, w, h])
            else:
                observations.append(None)
        
        # 创建模拟帧（用于可视化）
        frames = [np.zeros((480, 640, 3), dtype=np.uint8) for _ in range(num_frames)]
        
        print(f"模拟数据生成完成: {num_frames}帧, 有效观测{sum(1 for o in observations if o is not None)}个")
    else:
        # 从实际视频提取观测
        observations, frames = extract_observations_from_video(
            VIDEO_PATH, MODEL_PATH, TARGET_CLASS
        )
    
    # ==================== 步骤2: 初始化EKF跟踪器 ====================
    print("\n步骤2: 初始化EKF跟踪器...")
    
    # 创建EKF跟踪器（启用自适应Q）
    ekf_tracker = VisualEKF(dt=1.0, use_adaptive_Q=True)
    
    # 查找第一个有效观测用于初始化
    init_obs = None
    for obs in observations:
        if obs is not None:
            init_obs = obs
            break
    
    if init_obs is None:
        raise ValueError("视频中没有检测到目标！")
    
    # 使用第一个有效观测初始化EKF
    ekf_tracker.initialize(init_obs)
    
    # ==================== 步骤3: 运行EKF跟踪 ====================
    print("\n步骤3: 运行EKF跟踪...")
    
    ekf_states = []
    
    # 处理第一帧后的所有观测
    for i, obs in enumerate(observations[1:], 1):
        # 更新EKF
        state = ekf_tracker.update(obs)
        ekf_states.append(state)
        
        # 每50帧打印一次状态
        if i % 50 == 0 and state is not None:
            current_state = ekf_tracker.get_state()
            print(f"  帧 {i}: 位置({current_state['position'][0]:.1f}, {current_state['position'][1]:.1f}), "
                  f"速度{current_state['speed']:.1f}, 加速度{current_state['accel_magnitude']:.1f}")
    
    print(f"跟踪完成，共处理 {len(observations)} 帧")
    
    # ==================== 步骤4: 可视化结果 ====================
    print("\n步骤4: 生成可视化结果...")
    
    # 确保状态历史长度与观测长度一致
    if len(ekf_states) < len(observations):
        # 第一帧只有初始化，没有预测/更新
        ekf_states.insert(0, ekf_tracker.x.copy())
    
    # 可视化跟踪结果
    visualize_tracking_results(frames, observations, ekf_states, "tracking_output.mp4")
    
    # 绘制统计图表
    plot_tracking_statistics(observations, ekf_states)
    
    # ==================== 步骤5: 性能评估 ====================
    print("\n步骤5: 性能评估...")
    
    # 计算有效观测帧的估计误差
    errors = []
    for i, (obs, state) in enumerate(zip(observations, ekf_states)):
        if obs is not None and state is not None:
            # 计算位置误差
            obs_cx, obs_cy = obs[0], obs[1]
            est_cx, est_cy = state[0], state[1]
            error = np.sqrt((obs_cx - est_cx)**2 + (obs_cy - est_cy)**2)
            errors.append(error)
    
    if errors:
        print(f"位置估计误差统计:")
        print(f"  平均误差: {np.mean(errors):.2f} 像素")
        print(f"  误差标准差: {np.std(errors):.2f} 像素")
        print(f"  最大误差: {np.max(errors):.2f} 像素")
        print(f"  最小误差: {np.min(errors):.2f} 像素")
    
    print("\n=== EKF跟踪完成 ===")


if __name__ == "__main__":
    main()