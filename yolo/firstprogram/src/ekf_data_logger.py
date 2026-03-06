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
            export_rosbag: 是否自动导出ROS2 bag (rosbag2)格式（默认False）
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
            'pred_z',
            'meas_x',
            'meas_y',
            'meas_z',
            'frame_time'
        ])
        
        self.frame_count = 0
        self.start_time = time.time()
        self.export_rosbag = export_rosbag
        print(f"EKF数据记录器已初始化，数据将保存到: {self.filename}")
        if export_rosbag:
            print("ROS2 bag (rosbag2)导出功能已启用")
    
    def log_prediction(self, track_id, pred_x, pred_y, pred_z=None, meas_x=None, meas_y=None, meas_z=None):
        """
        记录一次预测数据（支持3D坐标）
        
        Args:
            track_id: 目标跟踪ID
            pred_x: 预测的x坐标
            pred_y: 预测的y坐标
            pred_z: 预测的z坐标（可选，3D跟踪时使用）
            meas_x: 实际测量的x坐标（可选）
            meas_y: 实际测量的y坐标（可选）
            meas_z: 实际测量的z坐标（可选，3D跟踪时使用）
        """
        current_time = time.time()
        frame_time = current_time - self.start_time
        
        self.writer.writerow([
            self.frame_count,
            current_time,
            track_id,
            pred_x,
            pred_y,
            pred_z if pred_z is not None else '',
            meas_x if meas_x is not None else '',
            meas_y if meas_y is not None else '',
            meas_z if meas_z is not None else '',
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
            
            # 如果启用，自动导出ROS2 bag格式
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
        将数据导出为ROS2 bag格式 (rosbag2)
        
        Args:
            output_path: 输出bag文件路径（默认使用CSV文件名，扩展名为.db3）
            topic_prefix: ROS话题前缀（默认"/ekf_tracking"）
            
        Returns:
            str: 导出的bag文件路径，如果失败则返回None
        """
        try:
            # 尝试导入ROS2相关模块
            import rclpy
            from rclpy.serialization import serialize_message
            from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Header
            from rclpy.time import Time
        except ImportError as e:
            print(f"错误: 无法导入ROS2模块 - {e}")
            print("请确保ROS2环境已正确设置，并安装了必要的Python包:")
            print("1. 确保已安装ROS2 (Humble或更高版本)")
            print("2. 确保已安装rosbag2-py和rclpy")
            print("3. 确保在正确的ROS2环境中运行")
            return None
        
        if output_path is None:
            output_path = self.filename.with_suffix('.db3')
        
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
            
            print(f"正在导出ROS2 bag格式 (rosbag2): {output_path}")
            
            # 创建rosbag2写入器
            storage_options = StorageOptions(
                uri=str(output_path),
                storage_id='sqlite3'
            )
            converter_options = ConverterOptions('', '')
            writer = SequentialWriter()
            writer.open(storage_options, converter_options)
            
            # 收集所有唯一的话题
            topics_created = set()
            
            for row in data:
                track_id = int(row['track_id'])
                pose_topic = f"{topic_prefix}/target_{track_id}/pose"
                meas_topic = f"{topic_prefix}/target_{track_id}/measurement"
                
                # 创建预测位置话题
                if pose_topic not in topics_created:
                    pose_topic_info = TopicMetadata(
                        name=pose_topic,
                        type='geometry_msgs/msg/PoseStamped',
                        serialization_format='cdr'
                    )
                    writer.create_topic(pose_topic_info)
                    topics_created.add(pose_topic)
                
                # 创建测量位置话题（如果有测量数据）
                if row['meas_x'] is not None and row['meas_y'] is not None:
                    if meas_topic not in topics_created:
                        meas_topic_info = TopicMetadata(
                            name=meas_topic,
                            type='geometry_msgs/msg/PoseStamped',
                            serialization_format='cdr'
                        )
                        writer.create_topic(meas_topic_info)
                        topics_created.add(meas_topic)
            
            # 写入数据
            for row in data:
                # 创建时间戳（纳秒）
                timestamp_ns = int(float(row['timestamp']) * 1e9)
                
                # 创建预测位置消息
                pose_msg = PoseStamped()
                pose_msg.header = Header()
                pose_msg.header.stamp.sec = int(float(row['timestamp']))
                pose_msg.header.stamp.nanosec = int((float(row['timestamp']) % 1) * 1e9)
                pose_msg.header.frame_id = "map"
                
                # 读取预测的z坐标（如果存在）
                pred_z = 0.0
                if 'pred_z' in row and row['pred_z'] is not None and row['pred_z'] != '':
                    pred_z = float(row['pred_z'])
                
                pose_msg.pose.position.x = float(row['pred_x'])
                pose_msg.pose.position.y = float(row['pred_y'])
                pose_msg.pose.position.z = pred_z
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                
                track_id = int(row['track_id'])
                pose_topic = f"{topic_prefix}/target_{track_id}/pose"
                
                writer.write(pose_topic, serialize_message(pose_msg), timestamp_ns)
                
                # 如果有测量数据，写入测量位置
                if row['meas_x'] is not None and row['meas_y'] is not None:
                    # 读取测量的z坐标（如果存在）
                    meas_z = 0.0
                    if 'meas_z' in row and row['meas_z'] is not None and row['meas_z'] != '':
                        meas_z = float(row['meas_z'])
                    
                    meas_pose_msg = PoseStamped()
                    meas_pose_msg.header = Header()
                    meas_pose_msg.header.stamp.sec = int(float(row['timestamp']))
                    meas_pose_msg.header.stamp.nanosec = int((float(row['timestamp']) % 1) * 1e9)
                    meas_pose_msg.header.frame_id = "map"
                    
                    meas_pose_msg.pose.position.x = float(row['meas_x'])
                    meas_pose_msg.pose.position.y = float(row['meas_y'])
                    meas_pose_msg.pose.position.z = meas_z
                    meas_pose_msg.pose.orientation.x = 0.0
                    meas_pose_msg.pose.orientation.y = 0.0
                    meas_pose_msg.pose.orientation.z = 0.0
                    meas_pose_msg.pose.orientation.w = 1.0
                    
                    meas_topic = f"{topic_prefix}/target_{track_id}/measurement"
                    writer.write(meas_topic, serialize_message(meas_pose_msg), timestamp_ns)
            
            print(f"✅ 数据已成功导出为ROS2 bag格式 (rosbag2): {output_path}")
            print(f"   包含 {len(data)} 条消息")
            print(f"   话题前缀: {topic_prefix}")
            print(f"   消息类型: geometry_msgs/msg/PoseStamped")
            print(f"   存储格式: SQLite3 (.db3)")
            return str(output_path)
            
        except Exception as e:
            print(f"导出ROS2 bag时出错: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _auto_export_rosbag(self):
        """自动导出ROS2 bag格式（内部方法）"""
        try:
            rosbag_path = self.export_to_rosbag()
            if rosbag_path:
                print(f"已自动生成ROS2 bag文件: {rosbag_path}")
            else:
                print("未生成ROS2 bag文件（可能数据为空、出错或ROS2环境未配置）")
        except Exception as e:
            print(f"自动导出ROS2 bag时出错: {e}")
    
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
        export_rosbag: 是否自动导出ROS2 bag格式（默认False）
    
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