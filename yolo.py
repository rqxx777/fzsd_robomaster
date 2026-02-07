#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
from ultralytics import YOLO

class YoloV8PoseNode(Node):
    def __init__(self):
        super().__init__('yolov8_pose_node')

        # ===== 模型路径（同级目录）=====
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'best.pt')

        self.get_logger().info(f'Loading YOLOv8 model from: {model_path}')
        self.model = YOLO(model_path)

        # ===== 发布器 =====
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/yolo/pose_raw',
            10
        )

        # 示例：摄像头 / 视频
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame, verbose=False)
        if len(results[0].boxes) == 0:
            return

        # 取第一个目标
        box = results[0].boxes[0]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        u = (x1 + x2) * 0.5
        v = (y1 + y2) * 0.5

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # ===== 像素 → 3D（示例，假深度）=====
        msg.pose.pose.position.x = u * 0.01
        msg.pose.pose.position.y = v * 0.01
        msg.pose.pose.position.z = 5.0

        msg.pose.pose.orientation.w = 1.0

        # ===== EKF 必须要的 covariance =====
        msg.pose.covariance = [
            0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0
        ]

        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = YoloV8PoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

