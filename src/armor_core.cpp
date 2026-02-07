#include "armor_detect.hpp"
#include <algorithm>
#include <numeric>
#include <iostream>

// ================= 构造函数 =================
ArmorDetector::ArmorDetector(const std::string& model_path, bool use_cuda)
    : env_(ORT_LOGGING_LEVEL_WARNING, "yolov8")
{
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (use_cuda)
    {
        OrtCUDAProviderOptions cuda_options;
        cuda_options.device_id = 0;
        session_options_.AppendExecutionProvider_CUDA(cuda_options);
    }

    session_ = Ort::Session(env_, model_path.c_str(), session_options_);

    // 输入 / 输出名
    input_name_ =
        session_.GetInputNameAllocated(0, allocator_).get();
    output_name_ =
        session_.GetOutputNameAllocated(0, allocator_).get();

    auto input_shape = session_.GetInputTypeInfo(0)
                           .GetTensorTypeAndShapeInfo()
                           .GetShape();

    input_h_ = static_cast<int>(input_shape[2]);
    input_w_ = static_cast<int>(input_shape[3]);

    class_names_ = {"blue", "red"};

    std::cout << "[ArmorDetector] Model loaded. Input: "
              << input_w_ << "x" << input_h_ << std::endl;
}

// ================= detect =================
void ArmorDetector::detect(const cv::Mat& frame,
                           std::vector<Detection>& detections)
{
    detections.clear();

    float scale;
    int pad_w, pad_h;

    cv::Mat input_img =
        letterbox(frame, scale, pad_w, pad_h);

    std::vector<float> input_tensor =
        preprocess(input_img);

    Ort::MemoryInfo mem_info =
        Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);

    std::vector<int64_t> dims = {1, 3, input_h_, input_w_};

    Ort::Value input_ort =
        Ort::Value::CreateTensor<float>(
            mem_info,
            input_tensor.data(),
            input_tensor.size(),
            dims.data(),
            dims.size());

    const char* input_names[] = {input_name_.c_str()};
    const char* output_names[] = {output_name_.c_str()};

    auto outputs = session_.Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_ort,
        1,
        output_names,
        1);

    float* out_data =
        outputs[0].GetTensorMutableData<float>();

    auto out_shape =
        outputs[0].GetTensorTypeAndShapeInfo().GetShape();

    int64_t N = out_shape[2];

    float conf_thresh = 0.35f;

    for (int64_t i = 0; i < N; i++)
    {
        float x = out_data[i];
        float y = out_data[N + i];
        float w = out_data[2 * N + i];
        float h = out_data[3 * N + i];
        float conf = out_data[4 * N + i];
        float cls = out_data[5 * N + i];

        if (conf < conf_thresh)
            continue;

        float cx = (x - pad_w) / scale;
        float cy = (y - pad_h) / scale;
        float bw = w / scale;
        float bh = h / scale;

        Detection det;
        det.box = cv::Rect(
            int(cx - bw / 2),
            int(cy - bh / 2),
            int(bw),
            int(bh));

        det.conf = conf;
        det.class_id = static_cast<int>(cls);

        detections.push_back(det);
    }

    auto keep = nms(detections, 0.45f);

    std::vector<Detection> filtered;
    for (int idx : keep)
        filtered.push_back(detections[idx]);

    detections.swap(filtered);
}

// ================= letterbox =================
cv::Mat ArmorDetector::letterbox(const cv::Mat& img,
                                 float& scale,
                                 int& pad_w,
                                 int& pad_h)
{
    int w = img.cols;
    int h = img.rows;

    scale = std::min(
        static_cast<float>(input_w_) / w,
        static_cast<float>(input_h_) / h);

    int new_w = int(w * scale);
    int new_h = int(h * scale);

    pad_w = (input_w_ - new_w) / 2;
    pad_h = (input_h_ - new_h) / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    cv::Mat out(input_h_, input_w_, CV_8UC3,
                cv::Scalar(114, 114, 114));

    resized.copyTo(
        out(cv::Rect(pad_w, pad_h, new_w, new_h)));

    return out;
}

// ================= preprocess =================
std::vector<float> ArmorDetector::preprocess(const cv::Mat& img)
{
    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    std::vector<float> data(3 * rgb.rows * rgb.cols);
    int idx = 0;

    for (int c = 0; c < 3; c++)
    {
        for (int i = 0; i < rgb.rows; i++)
        {
            for (int j = 0; j < rgb.cols; j++)
            {
                data[idx++] =
                    rgb.at<cv::Vec3f>(i, j)[c];
            }
        }
    }
    return data;
}

// ================= nms =================
std::vector<int> ArmorDetector::nms(
    const std::vector<Detection>& dets,
    float iou_threshold)
{
    std::vector<int> idxs(dets.size());
    std::iota(idxs.begin(), idxs.end(), 0);

    std::sort(idxs.begin(), idxs.end(),
              [&](int a, int b)
              {
                  return dets[a].conf > dets[b].conf;
              });

    std::vector<int> keep;

    while (!idxs.empty())
    {
        int cur = idxs[0];
        keep.push_back(cur);

        std::vector<int> remain;

        for (size_t i = 1; i < idxs.size(); i++)
        {
            int other = idxs[i];

            float inter_x1 =
                std::max(dets[cur].box.x,
                         dets[other].box.x);
            float inter_y1 =
                std::max(dets[cur].box.y,
                         dets[other].box.y);
            float inter_x2 =
                std::min(dets[cur].box.x + dets[cur].box.width,
                         dets[other].box.x + dets[other].box.width);
            float inter_y2 =
                std::min(dets[cur].box.y + dets[cur].box.height,
                         dets[other].box.y + dets[other].box.height);

            float inter_w =
                std::max(0.0f, inter_x2 - inter_x1);
            float inter_h =
                std::max(0.0f, inter_y2 - inter_y1);

            float inter_area = inter_w * inter_h;
            float area1 = dets[cur].box.area();
            float area2 = dets[other].box.area();

            float iou =
                inter_area / (area1 + area2 - inter_area);

            if (iou <= iou_threshold)
                remain.push_back(other);
        }

        idxs.swap(remain);
    }

    return keep;
}
