# ROS 2 Mobile Robot 控制包

## 🚀 概述
一个简单的ROS 2移动机器人控制包，包含键盘控制和车轮模拟器。

## ✨ 功能特性
- **键盘控制节点**：通过键盘控制机器人移动
- **车轮模拟器节点**：模拟车轮运动和里程计
- **URDF模型**：完整的机器人URDF描述
- **RViz配置**：预配置的可视化界面

## 📦 安装依赖
确保已安装ROS 2 Humble：
```bash
# 安装ROS 2核心包
sudo apt update
sudo apt install ros-humble-desktop

# 安装额外依赖
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-xacro
