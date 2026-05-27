# FZSD RoboMaster Sentinel AutoAim System

浮舟湿地战队 RoboMaster 哨兵机器人视觉自瞄系统，基于 ROS2 Humble + colcon 构建。

## 整体架构

```
相机 (HikVision MVS)
    │ sensor_msgs::Image
    ▼
┌─────────────────────────────────────────────┐
│  armor_detector (装甲板检测)                  │
│  - 传统CV: 二值化 → 轮廓提取 → 灯条匹配       │
│  - YOLO+OpenVINO 深度学习检测 (38类)          │
│  - PnP 位姿解算 (IPPE)                       │
│  输出: Armors (装甲板列表, 3D坐标+分类结果)    │
└──────────────┬──────────────────────────────┘
               │ auto_aim_interfaces::msg::Armors
               ▼
┌─────────────────────────────────────────────┐
│  armor_tracker (目标跟踪)                     │
│  - 9维扩展卡尔曼滤波 (EKF)                    │
│  - 状态机: LOST→DETECTING→TRACKING→TEMP_LOST │
│  - 多目标数据关联 + 目标切换                   │
│  输出: Target (目标状态估计 + 预测)            │
└──────────────┬──────────────────────────────┘
               │ auto_aim_interfaces::msg::Target
               ▼
┌─────────────────────────────────────────────┐
│  vision_attacker (弹道解算 + 运动规划)         │
│  - 弹道模型: 一阶空气阻力 + 重力 (g=9.78)      │
│  - TinyMPC ADMM 双轴独立 MPC (100步, DT=10ms) │
│  - 自适应延迟补偿 (高低速不同延时)              │
│  输出: AutoAim (瞄准角度 + 开火指令)           │
└──────────────┬──────────────────────────────┘
               │ vision_interfaces::msg::AutoAim
               ▼
┌─────────────────────────────────────────────┐
│  vision_serial_driver (串口通信)              │
│  - 下行: 瞄准指令 → MCU (500Hz)               │
│  - 上行: 机器人状态 ← MCU (枪口角/弹速/比赛信息)│
│  - 弹速滑动平均滤波                            │
│  - TF广播: odom → gimbal_link                 │
└──────────────┬──────────────────────────────┘
               │ 串口 (/dev/ttyACM0)
               ▼
          MCU (下位机)
```

## 目录结构

```
fzsd_autoaim2026-info/
├── README.md
├── .gitignore
└── src/
    ├── rm_auto_aim/
    │   ├── armor_detector/       # 装甲板检测模块
    │   │   ├── include/armor_detector/
    │   │   │   ├── armor.hpp          # Light/Armor 数据结构
    │   │   │   ├── detector.hpp       # 传统CV检测器
    │   │   │   ├── detector_node.hpp  # ROS2 检测节点
    │   │   │   ├── yolo_detector.hpp  # YOLO+OpenVINO检测器
    │   │   │   ├── number_classifier.hpp # MLP数字分类器
    │   │   │   └── pnp_solver.hpp     # PnP位姿解算
    │   │   ├── src/                   # 对应cpp实现
    │   │   ├── model/                 # 模型文件 (yolo11.xml/bin, mlp.onnx)
    │   │   ├── docs/                  # 检测效果图
    │   │   └── test/                  # 单元测试
    │   ├── armor_tracker/         # 目标跟踪模块
    │   │   ├── include/armor_tracker/
    │   │   │   ├── tracker.hpp             # 多目标EKF跟踪器
    │   │   │   ├── extended_kalman_filter.hpp # 通用EKF模板
    │   │   │   └── tracker_node.hpp        # ROS2 跟踪节点
    │   │   ├── src/                   # 对应cpp实现
    │   │   └── docs/
    │   ├── auto_aim_interfaces/    # 自定义ROS消息接口
    │   │   └── msg/
    │   │       ├── Armor.msg / Armors.msg      # 装甲板检测结果
    │   │       ├── Target.msg                  # 目标跟踪输出
    │   │       ├── DebugLight.msg / DebugLights.msg  # 调试:灯条信息
    │   │       ├── DebugArmor.msg / DebugArmors.msg  # 调试:装甲板信息
    │   │       ├── TrackerInfo.msg             # 跟踪器状态
    │   │       └── TimeInfo.msg                # 时间戳信息
    │   └── rm_auto_aim/            # 元包 (metapackage)
    │
    ├── vision_attacker/          # 弹道解算 + MPC 运动规划
    │   ├── include/vision_attacker/
    │   │   ├── vision_attacker_node.hpp  # ROS2 射手节点
    │   │   ├── planner.hpp              # TinyMPC 双轴 MPC 规划器
    │   │   ├── trajectory.hpp           # 一阶空气阻力弹道模型
    │   │   ├── target.hpp               # 11维目标状态预测
    │   │   ├── math_tools.hpp           # 坐标变换/雅可比
    │   │   └── outpost.hpp              # 前哨站装甲板模型
    │   ├── src/
    │   │   ├── vision_attacker_node.cpp
    │   │   ├── planner.cpp / trajectory.cpp / target.cpp / math_tools.cpp
    │   │   └── tinympc/             # TinyMPC ADMM 求解器
    │   │       ├── admm.cpp/hpp     # ADMM 迭代求解
    │   │       ├── codegen.cpp/hpp  # 预计算矩阵生成
    │   │       ├── tiny_api.cpp/hpp # C API 接口
    │   │       └── types.hpp        # 类型定义
    │   └── test/
    │
    ├── vision_serial_driver/     # 串口通信驱动
    │   ├── include/vision_serial_driver/
    │   │   ├── vision_serial_driver_node.hpp  # 串口ROS2节点
    │   │   ├── packet.h                       # 自定义二进制通信协议
    │   │   └── avgFilter.hpp                  # 滑动平均滤波器
    │   └── src/
    │
    ├── vision_interfaces/        # 上下行消息接口
    │   └── msg/
    │       ├── AutoAim.msg       # 瞄准指令 (yaw/pitch/开火)
    │       ├── Robot.msg         # 机器人状态 (云台角/弹速/模式)
    │       └── GameState.msg     # 比赛状态 (血量/占点/剩余时间)
    │
    ├── ros2-hik-camera/          # HikVision MVS 相机驱动
    │   ├── hikSDK/               # 海康SDK (libMvCameraControl.so)
    │   ├── config/               # 相机参数
    │   ├── launch/               # 相机启动文件
    │   └── src/hik_camera_node.cpp
    │
    ├── rm_vision/                # 启动配置
    │   └── rm_vision_bringup/
    │       ├── launch/
    │       │   ├── vision_bringup.launch.py   # 完整自瞄启动
    │       │   ├── autoaim.launch.py          # 自瞄节点组
    │       │   ├── no_hardware.launch.py      # 无硬件调试模式
    │       │   └── common.py                  # 通用launch参数
    │       └── config/
    │           ├── node_params.yaml   # 各节点参数
    │           ├── launch_params.yaml # 启动参数
    │           └── camera_info.yaml   # 相机内参/畸变
    │
    └── rm_gimbal_description/    # URDF 模型
        └── urdf/rm_gimbal.urdf.xacro
```

## 各模块详解

### 1. armor_detector — 装甲板检测

两种检测模式可切换：

**传统CV模式** (`use_yolo_=false`):
- 灰度化 → 固定阈值二值化 → `findContours` 轮廓提取
- 对每个轮廓: `minAreaRect` 拟合灯条 → `fitLine` 计算倾斜角 → 宽高比/角度筛选 → 颜色判定 (红/蓝)
- 灯条两两配对: 长度比/中心距离/连线角度验证 → 区分大小装甲板 (SMALL: 132×57mm, LARGE: 223×57mm)
- MLP (OpenCV DNN) 数字识别: 透视变换提取ROI → 20×28归一化 → ONNX推理

**YOLO+OpenVINO模式** (`use_yolo_=true`):
- YOLO11 38类别目标检测 (包含所有装甲板数字+哨兵+前哨站+基地)
- OpenVINO 推理加速 (x86/ARM 均支持)

**PnP位姿解算**: IPPE算法, 需要相机内参和畸变系数, 输出装甲板3D坐标

### 2. armor_tracker — 目标跟踪

**9维EKF状态向量**: `[xc, vx, yc, vy, za, vza, yaw, v_yaw, r]`
- xc/yc: 惯性系下的目标位置
- za: 目标z轴坐标
- yaw: 目标朝向角 (弧度), 由装甲板法向量反推
- r: 装甲板实际物理半径的一半 (区分大小装甲板)

**4维观测向量**: `[xa, ya, za, yaw]` — 相机系下的测量值

**状态机转换**:
```
LOST → DETECTING (连续检测到) → TRACKING (稳定跟踪)
  ↑         ↑                       ↓ (丢失)
  └── 长期丢失 ←── TEMP_LOST ←── 短暂丢失
                    ↑ (重叠目标)    ↑ (恢复检测)
              CHANGE_TARGET ←── 目标切换
```

**数据关联**: 基于马氏距离的最近邻匹配, 支持目标切换 (handleArmorJump)

### 3. vision_attacker — 弹道解算与运动规划

**弹道模型** (trajectory.hpp/cpp): 一阶空气阻力 `a_drag = -k * |v| * v`, 重力加速度 `g = 9.78 m/s²`
- 迭代法求解: 二分调整发射仰角, 直至落点与目标位置匹配
- 输出: 瞄准 yaw/pitch 角度

**TinyMPC 双轴独立规划** (planner.hpp/cpp):
- 每轴4维状态 `[角度, 角速度]`, 2维 `[位置, 速度]` 增广 → 4维
- 预测时域: 100步 × DT=0.01s = 1秒
- ADMM迭代求解, 加速度约束 (max_yaw_acc / max_pitch_acc)
- 自适应延迟补偿: 低速 (弹速<decision_speed) 和高速 (弹速≥decision_speed) 分别配置延迟时间

**fire_thresh**: MPC跟踪误差低于此阈值时触发开火

### 4. vision_serial_driver — 串口通信

**下行帧 (vision → MCU)**: 0xA5帧头, 19字节
```
fire | tracking | vel_control | aimYaw | aimPitch | aimPVel | aimYVel | vx | vy | wz | gyroscope
```

**上行帧 (MCU → vision)**: 0xA5帧头, ~42字节
```
mode | foeColor | robotYaw | robotPitch | muzzleSpeed | bigYaw | robot_quantity | blood_warn | game_process | left_time | our_preempt | enemy_preempt
```

**关键功能**:
- 500Hz 串口写入定时器 (每2ms发送瞄准数据)
- 独立读取线程 (循环接收MCU状态)
- 弹速滑动平均滤波 (avgFilter, 窗口可配置)
- 1Hz 串口重连检测 (断开自动重连)
- TF广播 odom → gimbal_link (发布云台位姿)

### 5. ros2-hik-camera — 海康相机驱动

- ROS2 ComposableNode 节点
- 使用 HikVision MVS SDK (`libMvCameraControl.so`)
- 支持 x86/ARM64 双平台
- 可配置曝光/增益/帧率/分辨率
- 发布 `sensor_msgs::Image` + `sensor_msgs::CameraInfo`

### 6. rm_vision_bringup — 启动配置

| 启动文件 | 用途 |
|---------|------|
| `vision_bringup.launch.py` | 完整系统启动 (相机+检测+跟踪+射手+串口) |
| `autoaim.launch.py` | 纯自瞄节点 (不含相机驱动) |
| `no_hardware.launch.py` | 无硬件调试模式 (offline测试) |
| `common.py` | 公共launch参数加载 |

## 编译

```bash
# 安装 ROS2 Humble + OpenCV + Eigen3 + OpenVINO
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

```bash
# 完整系统
ros2 launch rm_vision_bringup vision_bringup.launch.py

# 仅自瞄 (相机已运行)
ros2 launch rm_vision_bringup autoaim.launch.py

# 无硬件调试 (使用rosbag/图片回放)
ros2 launch rm_vision_bringup no_hardware.launch.py
```

## 调试

```bash
# 启用调试发布 (二值图+数字图+结果标注图+灯条/装甲板数据)
ros2 param set /armor_detector debug true

# 查看检测结果标注
ros2 run rqt_image_view rqt_image_view /armor_detector/result_img

# 查看串口通信状态
ros2 topic echo /robot
ros2 topic echo /game_state
```

## 关键参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `binary_thres` | int | 160 | 二值化阈值 (0-255) |
| `detect_color` | int | 0 | 目标颜色 (0=RED, 1=BLUE) |
| `use_yolo_` | bool | true | 使用YOLO检测模式 |
| `tracking_thres` | int | 10 | 确认跟踪帧数阈值 |
| `lost_thres` | int | 30 | 临时丢失超时帧数 |
| `max_match_distance` | double | 0.15 | 最大匹配距离 (m) |
| `fire_thresh` | double | 0.005 | 开火角度误差 (rad) |
| `max_yaw_acc` | double | 40.0 | Yaw轴最大加速度 (rad/s²) |
| `max_pitch_acc` | double | 30.0 | Pitch轴最大加速度 (rad/s²) |
| `air_k` | double | 0.001 | 空气阻力系数 |
| `lag_time` | double | 0.1 | 延迟补偿时间 (s) |

## License

本项目基于 MIT License 开源, 原作者:
- Copyright (C) 2022 ChenJun
- Copyright (C) 2024 Zheng Yu
