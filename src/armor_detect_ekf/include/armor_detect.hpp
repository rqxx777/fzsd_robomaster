#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <algorithm>
struct Detection
{
    cv::Rect box;
    int class_id;
    float conf;
};

class ArmorDetector
{
public:
    // model_path: best.onnx 的路径
    explicit ArmorDetector(const std::string& model_path, bool use_cuda = true);

    // 对一帧图像进行检测
    void detect(const cv::Mat& frame, std::vector<Detection>& detections);

private:
    // ========= 工具函数 =========
    cv::Mat letterbox(const cv::Mat& img,
                      float& scale,
                      int& pad_w,
                      int& pad_h);

    std::vector<float> preprocess(const cv::Mat& img);

    std::vector<int> nms(const std::vector<Detection>& dets,
                         float iou_threshold);

private:
    // ========= ONNX Runtime =========
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::AllocatorWithDefaultOptions allocator_;

    std::string input_name_;
    std::string output_name_;

    int input_w_;
    int input_h_;

    std::vector<std::string> class_names_;
};
