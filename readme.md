此次为浮洲湿地Robomaster算法组2025级大作业。本人是2025视觉组的小登朱彦。
此次作业分为部署仿真项目，计算最佳打击点，动态调参，以及引入轨迹规划器。

开门见山且很遗憾的告诉你，本人到目前为止依旧没有引入济瞄的轨迹规划。

但其他任务本人基本上马马虎虎地做完了（那个神经网络还在训练，前两周b事太多了，一万五千张图片经过上交的粗标后还要我进一步手标。别杀我，严哲涵学长，我马上滚去标）


感谢Actor&Thinker 战队成员开发的仿真环境
以及感谢华南师范大学成员提供的弹道解析的思路。虽然我本来尝试着去用，但最终还是自己写了一个简单一些的版本


## 环境配置：
本项目基于python3.10开发
请使用conda配置虚拟环境

创建后，请通过以下命令安装ros2核心包
conda config --env --add channels conda-forge
conda config --env --add channels robostack-humble
conda config --env --set channel_priority strict

以及为了与foxglove联动，也请使用下面的命令
conda install ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-foxglove-msgs -y

不知道是什么原因，我在程序运行过程中发现numpy用不了最新的，它提示我需要降到2.0以下：
//固定 numpy 版本（避免 NumPy 2.x 兼容问题）
conda install numpy=1.26.* -y

//安装 OpenCV（用于图像处理与 PnP）
pip install opencv-python

// 安装 YOLO
pip install ultralytics

//安装 EKF 支持
pip install filterpy

//安装 ROS 2 图像转换辅助库
pip install ros2-numpy

## 功能介绍：
首先，模型为model.pt，基于yolo26n训练得到。在检测装甲板时，置信度可以达到0.90左右。但我发现在很靠近时会出现检测不到的问题。

通过订阅仿真环境内的image_raw节点，我通过程序获取图像并用yolo进行检测。在foxglove里可用/img_detection节点看见绿色的框框

然后，通过EKF可以进行预测。所有的数据都可以在137至141行进行动态调节。
并以annotion的形式可以选择在图像中标出来，节点名称应该是/annotated_img
效果是可以在装甲板中心位置处以一个绿色的圈圈进行跟踪预测。

至于但弹道结算。本人通过pnp结算（整个装甲板，但你知道的，相较于四点，整块的误差相对更大一些）算出距离后，通过弹速以及重力加速度结算出弹道落点,在图像中，以红色的aim点标出。

当然，你也可以通过3D模板里选择marker，去看看红色的击打点。

## TODO
首先，在调试过程中。发现帧率十分感人，个人在img_compressed的使用中犹豫，发现用了反而在仿真环境中更慢了。
其次，只是计算出了落点以及pitch轴角度，不知到实时环境中效果怎样，有待测试。
最后，济瞄的学习该提上来了！！！

哦，我要去标数据集了，滚了。

