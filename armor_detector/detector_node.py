#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import  SetParametersResult
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, ObjectHypothesis
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from foxglove_msgs.msg import ImageAnnotations, CircleAnnotation, Color
import ros2_numpy
import cv2
import numpy as np
import math
from typing import Dict, Tuple, Optional
from ultralytics import YOLO
from filterpy.kalman import KalmanFilter



# PnP 解算
class PnPSolver:
    """装甲板位姿解算器"""
    def __init__(self, armor_width: float, armor_height: float,
                 camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None):
        self.armor_width = armor_width
        self.armor_height = armor_height
        # 物体坐标系下的四个角点 (顺序: 左上, 右上, 右下, 左下)
        self.object_points = np.array([
            [-armor_width/2, -armor_height/2, 0],
            [ armor_width/2, -armor_height/2, 0],
            [ armor_width/2,  armor_height/2, 0],
            [-armor_width/2,  armor_height/2, 0]
        ], dtype=np.float32)
        self.camera_matrix = camera_matrix
        if dist_coeffs is None:
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        else:
            self.dist_coeffs = dist_coeffs
        self.solve_method = cv2.SOLVEPNP_IPPE_SQUARE   # 适合平面矩形

    def solve(self, image_points: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        if image_points.shape != (4, 2):
            raise ValueError(f"image_points shape must be (4,2), got {image_points.shape}")
        image_points = image_points.astype(np.float32)
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.object_points, image_points,
                self.camera_matrix, self.dist_coeffs,
                flags=self.solve_method
            )
            return success, rvec, tvec
        except cv2.error as e:
            print(f"solvePnP error: {e}")
            return False, None, None

    def get_distance(self, tvec: np.ndarray) -> float:
        return float(np.linalg.norm(tvec))

    def get_rotation_matrix(self, rvec: np.ndarray) -> np.ndarray:
        R, _ = cv2.Rodrigues(rvec)
        return R


# EKF 跟踪
class ArmorTracker:
    def __init__(self, track_id: int, dt: float = 1/30.0):
        self.id = track_id
        self.dt = dt
        # 状态: [cx, cy, w, h, vx, vy, vw, vh]
        self.kf = KalmanFilter(dim_x=8, dim_z=4)
        self.kf.F = np.array([
            [1,0,0,0, dt,0,0,0],
            [0,1,0,0,0, dt,0,0],
            [0,0,1,0,0,0, dt,0],
            [0,0,0,1,0,0,0, dt],
            [0,0,0,0,1,0,0,0],
            [0,0,0,0,0,1,0,0],
            [0,0,0,0,0,0,1,0],
            [0,0,0,0,0,0,0,1]
        ])
        self.kf.H = np.array([
            [1,0,0,0,0,0,0,0],
            [0,1,0,0,0,0,0,0],
            [0,0,1,0,0,0,0,0],
            [0,0,0,1,0,0,0,0]
        ])
        self.kf.x = np.zeros((8, 1))
        self.kf.P *= 10.0
        self.initialized = False
        self.last_measurement = None

    def init(self, measurement: np.ndarray):
        self.kf.x[:4, 0] = measurement   # cx, cy, w, h
        self.kf.x[4:, 0] = 0.0
        self.initialized = True
        self.last_measurement = measurement

    def predict(self):
        if not self.initialized:
            return None
        self.kf.predict()
        return self.kf.x[:4, 0]

    def update(self, measurement: np.ndarray):
        if not self.initialized:
            self.init(measurement)
            return self.kf.x[:4, 0]
        self.kf.update(measurement)
        self.last_measurement = measurement
        return self.kf.x[:4, 0]

    def get_state(self) -> Optional[np.ndarray]:
        return self.kf.x[:4, 0] if self.initialized else None

    def set_noise(self, q_pos: float, q_vel: float, r_pos: float):
        q = np.array([q_pos, q_pos, q_pos, q_pos, q_vel, q_vel, q_vel, q_vel])
        self.kf.Q = np.diag(q)
        self.kf.R = np.eye(4) * r_pos

    def set_dt(self, dt: float):
        self.dt = dt
        self.kf.F[0,4] = dt
        self.kf.F[1,5] = dt
        self.kf.F[2,6] = dt
        self.kf.F[3,7] = dt



class ArmorDetector(Node):
    def __init__(self):
        super().__init__('armor_detector')

        # ---------- 参数 ----------
        self.declare_parameter('model_path', '/home/ming/Desktop/big/src/src/armor_detector/model.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('ekf_dt', 1/30.0)
        self.declare_parameter('ekf_q_pos', 0.1)
        self.declare_parameter('ekf_q_vel', 0.1)
        self.declare_parameter('ekf_r_pos', 0.5)
        self.declare_parameter('iou_threshold', 0.3)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        self.ekf_dt = self.get_parameter('ekf_dt').value
        self.q_pos = self.get_parameter('ekf_q_pos').value
        self.q_vel = self.get_parameter('ekf_q_vel').value
        self.r_pos = self.get_parameter('ekf_r_pos').value
        self.iou_thresh = self.get_parameter('iou_threshold').value

        # YOLO
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f'YOLO loaded from {model_path}, device: {device}')

        # 跟踪器字典
        self.trackers: Dict[int, ArmorTracker] = {}
        self.next_track_id = 0
        self.last_timestamp = None   # 用于动态计算 dt

        # 相机内参（我填入了仿真环境的数值）
        self.camera_matrix = np.array([
            [1303.675283386667, 0, 720],
            [0, 1303.675283386667, 540],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        # 装甲板物理尺寸（米）
        armor_width = 0.235
        armor_height = 0.127
        self.pnp_solver = PnPSolver(armor_width, armor_height,
                                    self.camera_matrix, self.dist_coeffs)

        # 弹道参数
        self.bullet_speed = 18.0   # m/s
        self.gravity = 9.8         # m/s²

        # 订阅图像
        self.subscription = self.create_subscription(Image, '/image_raw',
                                                     self.image_callback, 10)

        # 发布
        self.annotated_pub = self.create_publisher(Image, '/img_detection', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.annotations_pub = self.create_publisher(ImageAnnotations, '/detections_annotations', 10)
        self.aim_marker_pub = self.create_publisher(Marker, '/aim_point_marker', 10)

        # 参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('ArmorDetector with PnP and Ballistics started.')

    # 参数动态更新

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'ekf_q_pos':
                self.q_pos = param.value
            elif param.name == 'ekf_q_vel':
                self.q_vel = param.value
            elif param.name == 'ekf_r_pos':
                self.r_pos = param.value
            elif param.name == 'ekf_dt':
                self.ekf_dt = param.value
            elif param.name == 'iou_threshold':
                self.iou_thresh = param.value
        for tracker in self.trackers.values():
            tracker.set_noise(self.q_pos, self.q_vel, self.r_pos)
        return SetParametersResult(successful=True)

    # IoU 计算与匹配

    def compute_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0

    def associate_detections(self, detections, iou_thresh):
        """返回 {tracker_id: detection_index}"""
        if not self.trackers:
            return {}
        pred_boxes = []
        tracker_ids = []
        for tid, tracker in self.trackers.items():
            state = tracker.get_state()
            if state is not None:
                cx, cy, w, h = state
                x1 = cx - w/2
                y1 = cy - h/2
                x2 = cx + w/2
                y2 = cy + h/2
                pred_boxes.append((x1, y1, x2, y2))
                tracker_ids.append(tid)
        if not pred_boxes:
            return {}
        iou_matrix = np.zeros((len(pred_boxes), len(detections)))
        for i, pbox in enumerate(pred_boxes):
            for j, (_, dbox, _) in enumerate(detections):
                iou_matrix[i, j] = self.compute_iou(pbox, dbox)
        matched = {}
        used_det = set()
        for _ in range(min(len(pred_boxes), len(detections))):
            i, j = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
            if iou_matrix[i, j] < iou_thresh:
                break
            matched[tracker_ids[i]] = j
            used_det.add(j)
            iou_matrix[i, :] = -1
            iou_matrix[:, j] = -1
        return matched

    # 弹道解算与投影

    def solve_shooting_angle(self, s: float, h: float) -> Optional[float]:
        """返回 pitch 角（弧度），若无解返回 None"""
        if s <= 0:
            return None
        v = self.bullet_speed
        g = self.gravity
        A = g * s * s / (2.0 * v * v)
        a = A
        b = -s
        c = A + h
        delta = b*b - 4*a*c
        if delta < 0:
            return None
        sqrt_delta = math.sqrt(delta)
        u1 = (-b + sqrt_delta) / (2*a)
        u2 = (-b - sqrt_delta) / (2*a)
        u = u1 if abs(u1) < abs(u2) else u2
        return math.atan(u)

    def aim_point_in_camera(self, x_c: float, z_c: float, pitch_rad: float):
        """返回瞄准点在相机坐标系下的坐标 (x, y, z)"""
        aim_z = z_c
        aim_x = x_c
        aim_y = -z_c  * math.tan(pitch_rad)   # Y向下，向上瞄准为负
        return aim_x, aim_y, aim_z

    def project_point_to_image(self, point_camera):
        x, y, z = point_camera
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        if z <= 0:
            return None
        u = int(x / z * fx + cx)
        v = int(y / z * fy + cy)
        return (u, v)

    def image_callback(self, msg: Image):
 
        try:
            cv_image = ros2_numpy.numpify(msg)
            if cv_image.dtype != np.uint8:
                cv_image = cv_image.astype(np.uint8)
        except Exception as e:
            self.get_logger().error(f'ros2_numpy error: {e}')
            return

        #  动态时间步长（EKF）
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            if dt > 0:
                for tracker in self.trackers.values():
                    tracker.set_dt(dt)
        self.last_timestamp = current_time

        # YOLO 检测
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        # 存储当前帧检测信息
        detections_xyxy = []   # (x1,y1,x2,y2)
        detection_confs = []   # (conf, cls_id)
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                detections_xyxy.append((x1, y1, x2, y2))
                detection_confs.append((conf, cls_id))

        # 构建 current_dets: ((x1,y1,x2,y2), (cx,cy,w,h), conf, cls_id)
        current_dets = []
        for (x1,y1,x2,y2), (conf, cls_id) in zip(detections_xyxy, detection_confs):
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1
            current_dets.append(((x1,y1,x2,y2), (cx,cy,w,h), conf, cls_id))

        #  EKF 预测与关联
        for tracker in self.trackers.values():
            tracker.predict()
        match_dets = [(-1, (x1,y1,x2,y2), conf) for ((x1,y1,x2,y2), _, conf, _) in current_dets]
        matches = self.associate_detections(match_dets, self.iou_thresh)

        updated_tracker_states = {}
        used_det_indices = set(matches.values())

        # 更新匹配的跟踪器
        for tid, det_idx in matches.items():
            _, (cx,cy,w,h), conf, cls_id = current_dets[det_idx]
            meas = np.array([cx, cy, w, h])
            state = self.trackers[tid].update(meas)
            updated_tracker_states[tid] = state

        # 新建未匹配的跟踪器
        for i, (_, (cx,cy,w,h), conf, cls_id) in enumerate(current_dets):
            if i not in used_det_indices:
                new_id = self.next_track_id
                self.next_track_id += 1
                tracker = ArmorTracker(new_id, self.ekf_dt)
                tracker.set_noise(self.q_pos, self.q_vel, self.r_pos)
                meas = np.array([cx, cy, w, h])
                state = tracker.update(meas)
                self.trackers[new_id] = tracker
                updated_tracker_states[new_id] = state

        annotated = cv_image.copy()   # BGR

        # 对每个检测进行 PnP 解算 + 弹道瞄准点绘制
        # 这里只对第一个检测做 PnP 和落点（
        if current_dets:
            # 取第一个检测框的角点
            (x1, y1, x2, y2), (cx, cy, w, h), conf, cls_id = current_dets[0]
            # 四个角点顺序：左上、右上、右下、左下
            image_points = np.array([
                [x1, y1], [x2, y1], [x2, y2], [x1, y2]
            ], dtype=np.float32)

            success, rvec, tvec = self.pnp_solver.solve(image_points)
            if success:
                # tvec: 目标在相机坐标系下的位置 (X右, Y下, Z前)
                x_c, y_c, z_c = tvec[0,0], tvec[1,0], tvec[2,0]
                s = math.sqrt(x_c*x_c + z_c*z_c)   # 水平距离
                h = -y_c                           # 高度差（上正下负）
                pitch_rad = self.solve_shooting_angle(s, h)
                if pitch_rad is not None:
                    aim_x, aim_y, aim_z = self.aim_point_in_camera(x_c,z_c, pitch_rad)
                    aim_pixel = self.project_point_to_image((aim_x, aim_y, aim_z))
                    if aim_pixel:
                        u_aim, v_aim = aim_pixel
                        # 在图像上画红色瞄准点
                        cv2.circle(annotated, (u_aim, v_aim), 6, (255, 0, 0), -1)
                        cv2.putText(annotated, "Aim", (u_aim+8, v_aim),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

                        # 发布 3D Marker（可选）
                        marker = Marker()
                        marker.header.frame_id = "camera_link"   ##
                        marker.header.stamp = msg.header.stamp
                        marker.ns = "aim_point"
                        marker.id = 0
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = aim_x
                        marker.pose.position.y = aim_y
                        marker.pose.position.z = aim_z
                        marker.scale.x = 0.05
                        marker.scale.y = 0.05
                        marker.scale.z = 0.05
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        self.aim_marker_pub.publish(marker)
                else:
                    self.get_logger().debug("No ballistic solution")
            else:
                self.get_logger().debug("PnP solve failed")

        #  绘制 YOLO 检测框（绿色）
        for (x1,y1,x2,y2), (conf, cls_id) in zip(detections_xyxy, detection_confs):
            class_name = self.model.names[cls_id]
            label = f'{class_name}: {conf:.2f}'
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        #  发布 /detections 消息
        det_array = Detection2DArray()
        det_array.header = msg.header
        for (x1,y1,x2,y2), (conf, cls_id) in zip(detections_xyxy, detection_confs):
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = conf
            detection.results.append(hyp)
            det_array.detections.append(detection)

        # 发布 EKF 预测标注（绿色圆圈）
        annot_msg = ImageAnnotations()
        for state in updated_tracker_states.values():
            if state is not None:
                cx, cy, w, h = state
                if 0 <= cx < cv_image.shape[1] and 0 <= cy < cv_image.shape[0]:
                    circle = CircleAnnotation()
                    circle.position.x = float(cx)
                    circle.position.y = float(cy)
                    circle.diameter = float(max(w, h)) / 2.0
                    circle.thickness = 2.0
                    circle.outline_color = Color(r=0.0, g=1.0, b=0.0, a=1.0)
                    circle.fill_color = Color(r=0.0, g=0.0, b=0.0, a=0.0)
                    annot_msg.circles.append(circle)

        # 发布标注图像
      
        annotated_img = ros2_numpy.msgify(Image, annotated, encoding='rgb8')
        annotated_img.header = msg.header
        self.annotated_pub.publish(annotated_img)
        self.detections_pub.publish(det_array)
        self.annotations_pub.publish(annot_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

