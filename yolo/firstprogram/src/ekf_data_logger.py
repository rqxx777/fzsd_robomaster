import csv
import time
from datetime import datetime
from pathlib import Path


class EKFDatalogger:
    """EKF预测数据记录器，用于保存和导出预测位置数据"""
    
    def __init__(self, output_dir=".", filename_prefix="ekf_predictions"):
        """
        初始化数据记录器
        
        Args:
            output_dir: 输出目录路径
            filename_prefix: 文件名前缀
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
        print(f"EKF数据记录器已初始化，数据将保存到: {self.filename}")
    
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
        """关闭记录器，保存文件"""
        if not self.csv_file.closed:
            self.csv_file.close()
            print(f"EKF预测数据已保存到: {self.filename}")
            print(f"总共记录了 {self.frame_count} 帧数据")
    
    def get_filepath(self):
        """获取数据文件路径"""
        return str(self.filename)
    
    def export_to_json(self, output_path=None):
        """将数据导出为JSON格式（可选功能）"""
        import json
        
        if output_path is None:
            output_path = self.filename.with_suffix('.json')
        
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
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.close()


def create_ekf_datalogger(output_dir=".", filename_prefix="ekf_predictions"):
    """
    创建EKF数据记录器的便捷函数
    
    Returns:
        EKFDatalogger实例
    """
    return EKFDatalogger(output_dir, filename_prefix)


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