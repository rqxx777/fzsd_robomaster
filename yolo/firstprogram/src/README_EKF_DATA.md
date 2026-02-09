# EKF预测数据记录系统

## 概述

本系统扩展了原有的目标跟踪程序，增加了EKF（扩展卡尔曼滤波器）预测数据的记录功能。系统会自动将EKF预测的位置数据保存到CSV文件中，方便后续分析和可视化，特别适用于Foxglove数据可视化平台。

## 文件结构

```
firstprogram/src/
├── track.py                    # 主跟踪程序（已集成数据记录功能）
├── ekf_data_logger.py          # EKF数据记录器模块
├── README_EKF_DATA.md          # 本说明文档
├── armor_camera_detection.py   # 装甲板相机检测
├── cv.py                       # 计算机视觉工具
├── export_to_onnx.py           # ONNX模型导出
├── train.py                    # 训练脚本
├── best.pt                     # YOLO模型权重
└── demo_vedio.mp4              # 示例视频
```

## 使用方法

### 1. 运行跟踪程序

**重要：必须从yolo目录运行脚本**

```bash
# 切换到yolo目录
cd /home/xdc/fzsd_robomaster/yolo

# 运行跟踪程序
python3 firstprogram/src/track.py
```

**注意：不能从其他目录运行，否则会出现文件找不到的错误**

### 2. 验证安装

运行前可以验证文件路径是否正确：

```bash
cd /home/xdc/fzsd_robomaster/yolo
python3 -c "
import os
print('模型文件存在:', os.path.exists('firstprogram/best.pt'))
print('视频文件存在:', os.path.exists('firstprogram/demo_vedio.mp4'))
print('脚本文件存在:', os.path.exists('firstprogram/src/track.py'))
print('数据记录器存在:', os.path.exists('firstprogram/src/ekf_data_logger.py'))
"
```

### 3. 程序输出

程序运行时会显示以下信息：
- "当前目录: [路径]" (调试信息)
- "加载模型: firstprogram/best.pt"
- "输入视频: firstprogram/demo_vedio.mp4"
- "输出视频: firstprogram/src/detected_demo.mp4"
- "注意：请从/home/xdc/fzsd_robomaster/yolo目录运行此脚本"
- "开始EKF预测跟踪..."
- "EKF预测数据将自动保存到CSV和JSON文件中，可用于foxglove可视化"
- 实时显示检测和跟踪画面
- 按'q'键退出程序

程序结束后会显示：
- "EKF预测跟踪完成。结果保存到: [视频文件路径]"
- "EKF预测数据已保存到CSV和JSON文件，可用于foxglove可视化"
- "EKF数据记录器已初始化，数据将保存到: [CSV文件路径]" (程序开始时)
- "EKF预测数据已保存到CSV文件: [CSV文件路径]" (程序结束时)
- "已自动生成JSON文件: [JSON文件路径]" (程序结束时)

## 数据文件

### 生成的文件

程序运行后会生成两个数据文件，位于 `firstprogram/src/` 目录下：

1. **CSV文件**：`ekf_predictions_YYYYMMDD_HHMMSS.csv`
   - 原始数据格式，易于查看和导入
   - 包含完整的预测和测量数据

2. **JSON文件**：`ekf_predictions_YYYYMMDD_HHMMSS.json`
   - 自动从CSV转换生成
   - 结构化数据格式，易于程序处理
   - 包含类型转换后的数据

### CSV文件格式

| 列名 | 类型 | 描述 |
|------|------|------|
| frame_number | 整数 | 帧编号（从0开始） |
| timestamp | 浮点数 | 系统时间戳（秒） |
| track_id | 整数 | 目标跟踪ID |
| pred_x | 浮点数 | 预测的x坐标（像素） |
| pred_y | 浮点数 | 预测的y坐标（像素） |
| meas_x | 浮点数 | 实际测量的x坐标（像素，可选） |
| meas_y | 浮点数 | 实际测量的y坐标（像素，可选） |
| frame_time | 浮点数 | 相对于程序开始的时间（秒） |

### 示例数据

```
frame_number,timestamp,track_id,pred_x,pred_y,meas_x,meas_y,frame_time
0,1676543210.123,1,320.5,240.2,320.0,240.0,0.0
0,1676543210.123,2,100.3,150.7,100.0,150.0,0.0
1,1676543210.234,1,322.1,241.3,321.5,241.0,0.111
1,1676543210.234,2,101.8,152.4,101.2,152.1,0.111
```

## Foxglove可视化

### 方法1：导入CSV文件

1. 打开Foxglove Studio
2. 点击"Add Panel" → "Table"
3. 在Table面板中，点击"Import CSV"
4. 选择生成的CSV文件 (`ekf_predictions_YYYYMMDD_HHMMSS.csv`)
5. 数据将显示在表格中，可以进一步创建图表

### 方法2：导入JSON文件

1. 打开Foxglove Studio
2. 点击"Add Panel" → "Table"
3. 在Table面板中，点击"Import JSON"
4. 选择生成的JSON文件 (`ekf_predictions_YYYYMMDD_HHMMSS.json`)
5. JSON数据将自动解析并显示

### 方法3：使用Plot面板

1. 添加"Plot"面板
2. 配置数据源为CSV或JSON文件
3. 设置X轴为`frame_time`或`timestamp`
4. 设置Y轴为`pred_x`或`pred_y`
5. 按`track_id`分组显示不同目标的轨迹

### 方法4：创建2D可视化

1. 添加"Image"面板（如果需要显示视频）
2. 添加"Scatter Plot"面板显示2D位置
3. 配置X轴为`pred_x`，Y轴为`pred_y`
4. 使用`track_id`作为颜色编码

**推荐使用JSON格式**，因为：
- 数据类型已正确转换（数字、字符串、空值）
- 结构化格式更易于Foxglove解析
- 自动生成，无需手动转换

## 高级用法

### 独立使用数据记录器

`ekf_data_logger.py`可以独立使用，在其他项目中记录EKF预测数据：

```python
from ekf_data_logger import create_ekf_datalogger

# 创建数据记录器
with create_ekf_datalogger(output_dir="./data", filename_prefix="my_predictions") as logger:
    # 记录数据
    logger.increment_frame()
    logger.log_prediction(
        track_id=1,
        pred_x=100.5,
        pred_y=200.3,
        meas_x=100.0,
        meas_y=200.0
    )
```



### 数据转换脚本示例

创建 `convert_for_foxglove.py`：

```python
import pandas as pd
import json

# 读取CSV数据
df = pd.read_csv("ekf_predictions_20260204_143702.csv")

# 转换为Foxglove友好的格式
foxglove_data = []
for _, row in df.iterrows():
    foxglove_data.append({
        "timestamp": row["timestamp"],
        "track_id": int(row["track_id"]),
        "position": {
            "x": float(row["pred_x"]),
            "y": float(row["pred_y"]),
            "z": 0.0
        },
        "measurement": {
            "x": float(row["meas_x"]) if pd.notna(row["meas_x"]) else None,
            "y": float(row["meas_y"]) if pd.notna(row["meas_y"]) else None,
            "z": 0.0
        }
    })

# 保存为JSON
with open("ekf_data_foxglove.json", "w") as f:
    json.dump(foxglove_data, f, indent=2)

print("转换完成！")
```

## 故障排除

### 常见问题

1. **CSV文件未生成**
   - 检查输出目录权限
   - 确保程序正常退出（不是强制终止）
   - 查看控制台是否有错误信息

2. **数据不完整**
   - 确保程序运行到正常结束
   - 检查视频文件是否能正常打开

3. **Foxglove导入失败**
   - 确保CSV文件格式正确
   - 检查列名是否匹配
   - 尝试用文本编辑器打开CSV文件检查格式

### 日志信息

程序运行时会显示数据记录器的状态信息：
- "EKF数据记录器已初始化，数据将保存到: [文件路径]"
- "EKF预测数据已保存到: [文件路径]"
- "总共记录了 [数量] 帧数据"

## 扩展开发

### 添加新字段

要添加新的数据字段，修改以下文件：

1. `ekf_data_logger.py`:
   - 在`__init__`方法中更新表头
   - 在`log_prediction`方法中添加参数和写入逻辑

2. `track.py`:
   - 更新`log_prediction`调用，传入新字段的值

### 支持其他格式

数据记录器已支持JSON和ROS2 bag (rosbag2)导出，可以轻松扩展支持其他格式：
- JSON（默认自动导出，用于Foxglove可视化）
- ROS2 bag (rosbag2)（用于机器人系统，需要ROS2环境）
- Parquet（用于大数据集）
- SQLite（用于结构化查询）

## 联系与支持

如有问题或建议，请参考代码注释或联系开发团队。

---