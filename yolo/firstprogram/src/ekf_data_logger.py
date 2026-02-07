import csv
import time
from datetime import datetime
from pathlib import Path


class EKFDatalogger:
    """EKF预测数据记录器，用于保存和导出预测位置数据"""
    
    def __init__(self, output_dir=".", filename_prefix="ekf_predictions", export_rosbag=False):
        """
        初始化数据记录器
        
        Args:
            output_dir: 输出目录路径
            filename_prefix: 文件名前缀
            export_rosbag: 是否自动导出ROS bag格式（默认False）
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = self.output_dir / f"{filename_prefix}_{timestamp}.csv"
        
        # 初始化CSV写入器
        self.csv_file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        # 写入表头
        self.writer.writerow([
            'frame_number',
            'timestamp',
            'track_id',
            'pred_x',
            'pred_y',
            'meas_x',
            'meas_y',
            'frame_time'
        ])
        
        self.frame_count = 0
        self.start_time = time.time()
        self.export_rosbag = export_rosbag
        print(f"EKF数据记录器已初始化，数据将保存到: {self.filename}")
        if export_rosbag:
            print("ROS bag导出功能已启用")
    
    def log_prediction(self, track_id, pred_x, pred_y, meas_x=None, meas_y=None):
        """
        记录一次预测数据
        
        Args:
            track_id: 目标跟踪ID
            pred_x: 预测的x坐标
            pred_y: 预测的y坐标
            meas_x: 实际测量的x坐标（可选）
            meas_y: 实际测量的y坐标（可选）
        """
        current_time = time.time()
        frame_time = current_time - self.start_time
        
        self.writer.writerow([
            self.frame_count,
            current_time,
            track_id,
            pred_x,
            pred_y,
            meas_x if meas_x is not None else '',
            meas_y if meas_y is not None else '',
            frame_time
        ])
        
        # 确保数据立即写入文件
        self.csv_file.flush()
    
    def increment_frame(self):
        """增加帧计数器"""
        self.frame_count += 1
    
    def close(self):
        """关闭记录器，保存文件并自动导出JSON格式"""
        if not self.csv_file.closed:
            self.csv_file.close()
            print(f"EKF预测数据已保存到CSV文件: {self.filename}")
            print(f"总共记录了 {self.frame_count} 帧数据")
            
            # 自动导出JSON格式
            self._auto_export_json()
            
            # 如果启用，自动导出ROS bag格式
            if self.export_rosbag:
                self._auto_export_rosbag()
    
    def get_filepath(self):
        """获取数据文件路径"""
        return str(self.filename)
    
    def export_to_json(self, output_path=None):
        """将数据导出为JSON格式"""
        import json
        
        if output_path is None:
            output_path = self.filename.with_suffix('.json')
        
        # 检查CSV文件是否存在且不为空
        if not self.filename.exists():
            print(f"警告: CSV文件不存在: {self.filename}")
            return None
        
        file_size = self.filename.stat().st_size
        if file_size == 0:
            print(f"警告: CSV文件为空: {self.filename}")
            return None
        
        try:
            # 重新读取CSV数据并转换为JSON
            data = []
            with open(self.filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # 转换数据类型
                    converted_row = {}
                    for key, value in row.items():
                        if value == '':
                            converted_row[key] = None
                        elif key in ['frame_number', 'track_id']:
                            converted_row[key] = int(float(value))
                        else:
                            try:
                                converted_row[key] = float(value)
                            except ValueError:
                                converted_row[key] = value
                    data.append(converted_row)
            
            with open(output_path, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"数据已导出为JSON格式: {output_path}")
            return output_path
        except Exception as e:
            print(f"导出JSON时出错: {e}")
            return None
    
    def _auto_export_json(self):
        """自动导出JSON格式（内部方法）"""
        try:
            json_path = self.export_to_json()
            if json_path:
                print(f"已自动生成JSON文件: {json_path}")
            else:
                print("未生成JSON文件（可能数据为空或出错）")
        except Exception as e:
            print(f"自动导出JSON时出错: {e}")
    
    def export_to_rosbag(self, output_path=None, topic_prefix="/ekf_tracking"):
        """
        将数据导出为ROS bag格式
        
        Args:
            output_path: 输出bag文件路径（默认使用CSV文件名，扩展名为.bag）
            topic_prefix: ROS话题前缀（默认"/ekf_tracking"）
            
        Returns:
            str: 导出的bag文件路径，如果失败则返回None
        """
        try:
            # 尝试导入ROS相关模块
            import rosbag
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Header
            import rospy
        except ImportError as e:
            print(f"错误: 无法导入ROS模块 - {e}")
            print("请确保ROS环境已正确设置，并安装了必要的Python包:")
            print("1. 确保已安装ROS (Noetic或更高版本)")
            print("2. 确保已安装python3-rosbag和python3-rospy")
            print("3. 确保在正确的ROS环境中运行")
            return None
        
        if output_path is None:
            output_path = self.filename.with_suffix('.bag')
        
        # 检查CSV文件是否存在且不为空
        if not self.filename.exists():
            print(f"警告: CSV文件不存在: {self.filename}")
            return None
        
        file_size = self.filename.stat().st_size
        if file_size == 0:
            print(f"警告: CSV文件为空: {self.filename}")
            return None
        
        try:
            # 读取CSV数据
            data = []
            with open(self.filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # 转换数据类型
                    converted_row = {}
                    for key, value in row.items():
                        if value == '':
                            converted_row[key] = None
                        elif key in ['frame_number', 'track_id']:
                            converted_row[key] = int(float(value))
                        else:
                            try:
                                converted_row[key] = float(value)
                            except ValueError:
                                converted_row[key] = value
                    data.append(converted_row)
            
            if not data:
                print("警告: 没有数据可导出")
                return None
            
            print(f"正在导出ROS bag格式: {output_path}")
            
            # 创建rosbag文件
            with rosbag.Bag(str(output_path), 'w') as bag:
                for row in data:
                    # 创建时间戳（将Unix时间戳转换为ROS时间）
                    timestamp = rospy.Time.from_sec(float(row['timestamp']))
                    
                    # 创建PoseStamped消息
                    pose_msg = PoseStamped()
                    pose_msg.header = Header()
                    pose_msg.header.stamp = timestamp
                    pose_msg.header.frame_id = "map"  # 可以根据需要修改
                    
                    # 设置位置（预测位置）
                    pose_msg.pose.position.x = float(row['pred_x'])
                    pose_msg.pose.position.y = float(row['pred_y'])
                    pose_msg.pose.position.z = 0.0  # 2D跟踪，z=0
                    
                    # 设置方向（单位四元数，无旋转）
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = 0.0
                    pose_msg.pose.orientation.w = 1.0
                    
                    # 创建话题名称，包含跟踪ID
                    track_id = int(row['track_id'])
                    topic_name = f"{topic_prefix}/target_{track_id}/pose"
                    
                    # 写入bag
                    bag.write(topic_name, pose_msg, timestamp)
                    
                    # 如果有测量数据，也写入测量位置
                    if row['meas_x'] is not None and row['meas_y'] is not None:
                        meas_pose_msg = PoseStamped()
                        meas_pose_msg.header = Header()
                        meas_pose_msg.header.stamp = timestamp
                        meas_pose_msg.header.frame_id = "map"
                        
                        meas_pose_msg.pose.position.x = float(row['meas_x'])
                        meas_pose_msg.pose.position.y = float(row['meas_y'])
                        meas_pose_msg.pose.position.z = 0.0
                        meas_pose_msg.pose.orientation.x = 0.0
                        meas_pose_msg.pose.orientation.y = 0.0
                        meas_pose_msg.pose.orientation.z = 0.0
                        meas_pose_msg.pose.orientation.w = 1.0
                        
                        meas_topic_name = f"{topic_prefix}/target_{track_id}/measurement"
                        bag.write(meas_topic_name, meas_pose_msg, timestamp)
            
            print(f"✅ 数据已成功导出为ROS bag格式: {output_path}")
            print(f"   包含 {len(data)} 条消息")
            print(f"   话题前缀: {topic_prefix}")
            print(f"   消息类型: geometry_msgs/PoseStamped")
            return str(output_path)
            
        except Exception as e:
            print(f"导出ROS bag时出错: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _auto_export_rosbag(self):
        """自动导出ROS bag格式（内部方法）"""
        try:
            rosbag_path = self.export_to_rosbag()
            if rosbag_path:
                print(f"已自动生成ROS bag文件: {rosbag_path}")
            else:
                print("未生成ROS bag文件（可能数据为空、出错或ROS环境未配置）")
        except Exception as e:
            print(f"自动导出ROS bag时出错: {e}")
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.close()


def create_ekf_datalogger(output_dir=".", filename_prefix="ekf_predictions", export_rosbag=False):
    """
    创建EKF数据记录器的便捷函数
    
    Args:
        output_dir: 输出目录路径
        filename_prefix: 文件名前缀
        export_rosbag: 是否自动导出ROS bag格式（默认False）
    
    Returns:
        EKFDatalogger实例
    """
    return EKFDatalogger(output_dir, filename_prefix, export_rosbag)


if __name__ == "__main__":
    # 测试数据记录器
    with create_ekf_datalogger() as logger:
        # 模拟一些数据
        for i in range(10):
            logger.increment_frame()
            for track_id in [1, 2, 3]:
                logger.log_prediction(
                    track_id=track_id,
                    pred_x=i * 10 + track_id,
                    pred_y=i * 5 + track_id,
                    meas_x=i * 10 + track_id + 0.5,
                    meas_y=i * 5 + track_id + 0.5
                )
    
    print("测试完成！")