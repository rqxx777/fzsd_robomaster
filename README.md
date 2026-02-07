# fzsd_robomaster
<<<<<<< Updated upstream
## 1.作业概况

通过yolov8n.pt模型和自制数据集训练一个能够识别红蓝两方装甲板的模型，通过 ONNX 格式导出后，在 C++ 环境中完成推理与结果可视化。
***
## 2.数据集构建

下载网络中找到的第一人称实战视频，上传到roboflow并使用roboflow的制作数据集功能。
***
## 3.实现的功能

调用电脑的默认摄像头实现对红蓝两方的装甲板的识别。
***
## 4.main文件的主要流程

-   **初始化 ONNX Runtime 环境**
    
    -   创建推理环境（Env）
        
    -   配置 SessionOptions（启用图优化、CUDA 推理等）
        
-   **加载模型**
    
    -   读取 ONNX 模型文件
        
    -   获取模型输入 / 输出信息
        
-   **图像采集与预处理**
    
    -   通过 OpenCV 获取摄像头画面
        
    -   使用 Letterbox 保持宽高比缩放
        
    -   归一化并转换为 NCHW 格式的 Tensor
        
-   **模型推理**
    
    -   构造输入 Tensor
        
    -   调用 ONNX Runtime 进行前向推理
        
-   **后处理**
    
    -   解析模型输出
        
    -   按置信度阈值筛选候选框
        
    -   使用 NMS 去除冗余检测框
        
-   **结果可视化**
    
    -   将检测框映射回原图坐标
        
    -   绘制类别与置信度信息
 ***
 
## 5.使用的环境
=======

## 1.作业概况
在上一次的基础上引入EKF预测
***
## 2.做了什么
学习EKF原理，做成代码，将代码改成ros2节点方便接入foxglove
***
## 3.实现的功能

调用电脑的默认摄像头实现对红蓝两方的装甲板的识别，并预测位置
***
## 4.各节点的功能

**camera_node**
顾名思义，调用摄像头并发布摄像头的内容
**detect_ekf_node**
接收camera_node的Imagemsg转换成mat并使用onnxruntime调用装甲板识别模型识别装甲板,输出为Detection
```cpp
struct  Detection

{

cv::Rect  box;

int  class_id;

float  conf;

};
```
利用Detection.box获取装甲板的信息，结合相机的参数测算装甲板大致的距离，得到装甲板的坐标。并将这个坐标带入ekf.cpp的函数中算出预测位置。构造marker和posestamped，分别传入装甲板和预测位置的信息，并将其发布出去


 ***
 ## 5.需要自己更改的参数
 ***
 onnxruntime的路径
 相机的参数
 模型的路径
 装甲板的大小

## 6.使用的环境
>>>>>>> Stashed changes
**系统**：Ubuntu 24.04 LTS
**编译器**：g++ 13.3.0
**CMake**:3.28.3
**OpenCV** 4.12.0
**ONNX Runtime**: 1.20.0 (CUDA)
CUDA Toolkit 12.8
GPU: NVIDIA RTX 5060

<<<<<<< Updated upstream

浮舟湿地战队算法组预备队员仓库
=======
>>>>>>> Stashed changes
