// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
// YOLO Detector for ROS2 - Adapted from sp_vision_25

#include "armor_detector/yolo_detector.hpp"

#include <algorithm>

namespace rm_auto_aim
{

// 38类装甲板属性映射表
// 顺序与训练模型的类别顺序一致
// {color, number, type}: color: 0=blue, 1=red, 2=extinguish, 3=purple
const std::vector<std::tuple<int, std::string, ArmorType>> YoloDetector::armor_classes_ = {
  {0, "sentry", ArmorType::SMALL},     // 0: blue sentry
  {1, "sentry", ArmorType::SMALL},     // 1: red sentry
  {2, "sentry", ArmorType::SMALL},     // 2: extinguish sentry
  {0, "1", ArmorType::LARGE},          // 3: blue 1 (hero, always large)
  {1, "1", ArmorType::LARGE},          // 4: red 1
  {2, "1", ArmorType::LARGE},          // 5: extinguish 1
  {0, "2", ArmorType::SMALL},          // 6: blue 2 (engineer, always small)
  {1, "2", ArmorType::SMALL},          // 7: red 2
  {2, "2", ArmorType::SMALL},          // 8: extinguish 2
  {0, "3", ArmorType::SMALL},          // 9: blue 3
  {1, "3", ArmorType::SMALL},          // 10: red 3
  {2, "3", ArmorType::SMALL},          // 11: extinguish 3
  {0, "4", ArmorType::SMALL},          // 12: blue 4
  {1, "4", ArmorType::SMALL},          // 13: red 4
  {2, "4", ArmorType::SMALL},          // 14: extinguish 4
  {0, "5", ArmorType::SMALL},          // 15: blue 5
  {1, "5", ArmorType::SMALL},          // 16: red 5
  {2, "5", ArmorType::SMALL},          // 17: extinguish 5
  {0, "outpost", ArmorType::SMALL},    // 18: blue outpost
  {1, "outpost", ArmorType::SMALL},    // 19: red outpost
  {2, "outpost", ArmorType::SMALL},    // 20: extinguish outpost
  {0, "base", ArmorType::LARGE},       // 21: blue base big
  {1, "base", ArmorType::LARGE},       // 22: red base big
  {2, "base", ArmorType::LARGE},       // 23: extinguish base big
  {3, "base", ArmorType::LARGE},       // 24: purple base big
  {0, "base", ArmorType::SMALL},       // 25: blue base small
  {1, "base", ArmorType::SMALL},       // 26: red base small
  {2, "base", ArmorType::SMALL},       // 27: extinguish base small
  {3, "base", ArmorType::SMALL},       // 28: purple base small
  {0, "3", ArmorType::LARGE},          // 29: blue 3 big
  {1, "3", ArmorType::LARGE},          // 30: red 3 big
  {2, "3", ArmorType::LARGE},          // 31: extinguish 3 big
  {0, "4", ArmorType::LARGE},          // 32: blue 4 big
  {1, "4", ArmorType::LARGE},          // 33: red 4 big
  {2, "4", ArmorType::LARGE},          // 34: extinguish 4 big
  {0, "5", ArmorType::LARGE},          // 35: blue 5 big
  {1, "5", ArmorType::LARGE},          // 36: red 5 big
  {2, "5", ArmorType::LARGE},          // 37: extinguish 5 big
};

  // OpenVINO 模型加载与预处理配置 (NHWC → NCHW, BGR → RGB, 归一化 1/255)
YoloDetector::YoloDetector(const YoloParams & params) : params_(params)
{
  // 初始化ROI偏移
  if (params_.use_roi) {
    offset_ = cv::Point2f(params_.roi.x, params_.roi.y);
  } else {
    offset_ = cv::Point2f(0, 0);
  }

  // 加载模型
  auto model = core_.read_model(params_.model_path);
  
  // 配置预处理
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, params_.input_size, params_.input_size, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, params_.device, 
    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::vector<Armor> YoloDetector::detect(const cv::Mat & img)
  // 主检测流程: 预处理 → 推理 → 后处理 → 有效性检查 → 格式转换
{
  if (img.empty()) {
    return std::vector<Armor>();
  }

  // 预处理
  double scale;
  cv::Mat input_img = preprocess(img, scale);
  
  // 创建输入tensor
  size_t input_sz = static_cast<size_t>(params_.input_size);
  ov::Tensor input_tensor(
    ov::element::u8, 
    {1, input_sz, input_sz, 3}, 
    input_img.data);

  // 推理
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // 后处理
  auto output_tensor = infer_request.get_output_tensor();
  std::vector<YoloArmor> yolo_armors = postprocess(scale, output_tensor);

  // 转换为Armor格式
  std::vector<Armor> armors;
  for (const auto & yolo_armor : yolo_armors) {
    if (checkArmor(yolo_armor)) {
      armors.push_back(convertToArmor(yolo_armor));
    }
  }

  return armors;
}

cv::Mat YoloDetector::preprocess(const cv::Mat & img, double & scale)
  // Letterbox 缩放: 保持宽高比填充到 input_size × input_size
{
  cv::Mat bgr_img;
  
  // 应用ROI
  if (params_.use_roi) {
    cv::Rect roi = params_.roi;
    if (roi.width == -1) roi.width = img.cols;
    if (roi.height == -1) roi.height = img.rows;
    
    // 确保ROI在图像范围内
    roi.x = std::max(0, std::min(roi.x, img.cols - 1));
    roi.y = std::max(0, std::min(roi.y, img.rows - 1));
    roi.width = std::min(roi.width, img.cols - roi.x);
    roi.height = std::min(roi.height, img.rows - roi.y);
    
    bgr_img = img(roi);
    offset_ = cv::Point2f(roi.x, roi.y);
  } else {
    bgr_img = img;
    offset_ = cv::Point2f(0, 0);
  }

  // 计算缩放比例
  auto x_scale = static_cast<double>(params_.input_size) / bgr_img.rows;
  auto y_scale = static_cast<double>(params_.input_size) / bgr_img.cols;
  scale = std::min(x_scale, y_scale);
  
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // 创建输入图像（letterbox方式）
  cv::Mat input = cv::Mat(params_.input_size, params_.input_size, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Rect roi_rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi_rect), {w, h});

  return input;
}

  // 解析 YOLO 输出: 边界框解码 + NMS 去重 + ROI 偏移补偿
std::vector<YoloArmor> YoloDetector::postprocess(double scale, const ov::Tensor & output_tensor)
{
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data<float>());
  
  // YOLO输出格式: [batch, features, num_detections] -> 需要转置
  cv::transpose(output, output);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> all_keypoints;

  for (int r = 0; r < output.rows; r++) {
    auto xywh = output.row(r).colRange(0, 4);
    auto scores = output.row(r).colRange(4, 4 + params_.class_num);
    auto keypoints_data = output.row(r).colRange(4 + params_.class_num, 4 + params_.class_num + 8);

    // 找到最大分数的类别
    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

    if (score < params_.score_threshold) continue;

    // 解析边界框
    auto x = xywh.at<float>(0);
    auto y = xywh.at<float>(1);
    auto w = xywh.at<float>(2);
    auto h = xywh.at<float>(3);
    auto left = static_cast<int>((x - 0.5 * w) / scale);
    auto top = static_cast<int>((y - 0.5 * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    // 解析关键点
    std::vector<cv::Point2f> keypoints;
    for (int i = 0; i < 4; i++) {
      float kp_x = keypoints_data.at<float>(0, i * 2 + 0) / scale;
      float kp_y = keypoints_data.at<float>(0, i * 2 + 1) / scale;
      keypoints.emplace_back(kp_x, kp_y);
    }

    ids.push_back(max_point.x);
    confidences.push_back(static_cast<float>(score));
    boxes.emplace_back(left, top, width, height);
    all_keypoints.push_back(keypoints);
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, params_.score_threshold, params_.nms_threshold, indices);

  // 构建结果
  std::vector<YoloArmor> results;
  for (const auto & i : indices) {
    YoloArmor armor;
    armor.class_id = ids[i];
    armor.confidence = confidences[i];
    armor.box = boxes[i];
    armor.keypoints = all_keypoints[i];
    
    // 排序关键点
    sortKeypoints(armor.keypoints);
    
    // 应用ROI偏移
    if (params_.use_roi) {
      armor.box.x += static_cast<int>(offset_.x);
      armor.box.y += static_cast<int>(offset_.y);
      for (auto & kp : armor.keypoints) {
        kp += offset_;
      }
    }
    
    // 解析类别
    parseClassId(armor.class_id, armor.color, armor.number, armor.type);
    
    results.push_back(armor);
  }

  return results;
}
  // 角点排序: 按 y 分为上下两组，每组按 x 排列 → 左上、右上、右下、左下

void YoloDetector::sortKeypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) return;

  // 按y坐标排序，分成上下两组
  std::sort(keypoints.begin(), keypoints.end(), 
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  // 上面两点按x排序
  std::sort(top_points.begin(), top_points.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });
  
  // 下面两点按x排序
  std::sort(bottom_points.begin(), bottom_points.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

  // 重新排列: 左上、右上、右下、左下
  keypoints[0] = top_points[0];     // top-left
  keypoints[1] = top_points[1];     // top-right
  keypoints[2] = bottom_points[1];  // bottom-right
  keypoints[3] = bottom_points[0];  // bottom-left
  // 从 class_id 查表获取: 颜色、数字标签、装甲板大小类型
}

void YoloDetector::parseClassId(int class_id, int & color, std::string & number, ArmorType & type)
{
  if (class_id >= 0 && class_id < static_cast<int>(armor_classes_.size())) {
    color = std::get<0>(armor_classes_[class_id]);
    number = std::get<1>(armor_classes_[class_id]);
    type = std::get<2>(armor_classes_[class_id]);
  } else {
    // 默认值
    color = 0;
    number = "";
    type = ArmorType::SMALL;
  // 有效性验证: 置信度阈值 + 数字非空检查
  }
}

bool YoloDetector::checkArmor(const YoloArmor & armor)
{
  // 检查置信度
  if (armor.confidence < params_.min_confidence) {
    return false;
  }
  
  // 检查数字是否有效
  if (armor.number.empty()) {
    return false;
  }
  
  // 过滤熄灭的灯条 (color == 2 是 extinguish)
  // 如果需要检测熄灭的，可以注释掉这行
  // if (armor.color == 2) return false;
  // YoloArmor → Armor 转换: 将YOLO关键点映射到系统的左右灯条结构
  
  return true;
}

Armor YoloDetector::convertToArmor(const YoloArmor & yolo_armor)
{
  Armor armor;
  
  // 从关键点构造Light
  // keypoints顺序: 左上(0)、右上(1)、右下(2)、左下(3)
  // Light需要: top, bottom
  armor.left_light.top = yolo_armor.keypoints[0];     // 左上
  armor.left_light.bottom = yolo_armor.keypoints[3];  // 左下
  armor.left_light.center = (armor.left_light.top + armor.left_light.bottom) / 2;
  armor.left_light.length = cv::norm(armor.left_light.top - armor.left_light.bottom);
  
  armor.right_light.top = yolo_armor.keypoints[1];    // 右上
  armor.right_light.bottom = yolo_armor.keypoints[2]; // 右下
  armor.right_light.center = (armor.right_light.top + armor.right_light.bottom) / 2;
  armor.right_light.length = cv::norm(armor.right_light.top - armor.right_light.bottom);
  
  // 设置颜色
  // YOLO的color: 0=blue, 1=red, 2=extinguish, 3=purple
  // 原系统的color: RED=0, BLUE=1
  if (yolo_armor.color == 0) {
    armor.left_light.color = BLUE;
    armor.right_light.color = BLUE;
  } else if (yolo_armor.color == 1) {
    armor.left_light.color = RED;
    armor.right_light.color = RED;
  } else {
    // 熄灭/紫色装甲板，设为-1，颜色过滤时一定会被排除
    armor.left_light.color = -1;
    armor.right_light.color = -1;
  }
  
  // 计算中心点
  armor.center = (armor.left_light.center + armor.right_light.center) / 2;
  
  // 设置类型和编号
  armor.type = yolo_armor.type;
  armor.number = yolo_armor.number;
  armor.confidence = yolo_armor.confidence;
  armor.classfication_result = yolo_armor.number;
  
  return armor;
}

void YoloDetector::drawResults(cv::Mat & img, const std::vector<Armor> & armors)
{
  for (const auto & armor : armors) {
    // 画关键点连线
    std::vector<cv::Point> pts = {
      armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom, armor.left_light.bottom
    };
    cv::polylines(img, pts, true, cv::Scalar(0, 255, 0), 2);
    
    // 画对角线
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);
    
    // 显示信息
    std::string info = armor.number + " " + 
                       std::to_string(armor.confidence).substr(0, 4) + " " +
                       ARMOR_TYPE_STR[static_cast<int>(armor.type)];
    cv::putText(img, info, armor.left_light.top, 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace rm_auto_aim
