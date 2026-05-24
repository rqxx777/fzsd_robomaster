#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <numeric>   

struct Detection 
{
    cv::Rect box;
    int class_id;
    float conf;
};

// Letterbox
static cv::Mat letterbox(const cv::Mat& img, int target_w, int target_h, float& scale, int& pad_w, int& pad_h) 
{
    int w = img.cols, h = img.rows;
    scale = std::min((float)target_w / w, (float)target_h / h);
    int new_w = (int)(w * scale);
    int new_h = (int)(h * scale);

    pad_w = (target_w - new_w) / 2;
    pad_h = (target_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    cv::Mat out(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(pad_w, pad_h, new_w, new_h)));
    return out;
}

// Preprocess
static std::vector<float> preprocess(const cv::Mat& img) 
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
                data[idx++] = rgb.at<cv::Vec3f>(i, j)[c];
            }
        }
    }
    return data;
}

// NMS
static std::vector<int> nms(const std::vector<Detection>& dets, float iou_threshold) 
{
    std::vector<int> idxs(dets.size());
    std::iota(idxs.begin(), idxs.end(), 0);

    std::sort(idxs.begin(), idxs.end(), [&](int a, int b) {
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
            float inter_x1 = std::max(dets[cur].box.x, dets[other].box.x);
            float inter_y1 = std::max(dets[cur].box.y, dets[other].box.y);
            float inter_x2 = std::min(dets[cur].box.x + dets[cur].box.width, dets[other].box.x + dets[other].box.width);
            float inter_y2 = std::min(dets[cur].box.y + dets[cur].box.height, dets[other].box.y + dets[other].box.height);

            float inter_w = std::max(0.0f, inter_x2 - inter_x1);
            float inter_h = std::max(0.0f, inter_y2 - inter_y1);
            float inter_area = inter_w * inter_h;

            float area1 = dets[cur].box.area();
            float area2 = dets[other].box.area();
            float iou = inter_area / (area1 + area2 - inter_area);

            if (iou <= iou_threshold) 
            {
                remain.push_back(other);
            }
        }
        idxs = remain;
    }
    return keep;
}

int main() 
{
    // ONNX Runtime 初始化
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolov8");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // 类别名称
    std::vector<std::string> class_names = {"blue", "red"};

    const char* model_path = "/home/lby/opencv/src/best.onnx";
    // 启用 CUDA（GPU）推理
    OrtCUDAProviderOptions cuda_options;
    cuda_options.device_id = 0;
    session_options.AppendExecutionProvider_CUDA(cuda_options);

    Ort::Session session(env, model_path, session_options);

    // 获取输入输出名称（正确方式）
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name_alloc = session.GetInputNameAllocated(0, allocator);
    auto output_name_alloc = session.GetOutputNameAllocated(0, allocator);

    std::string input_name = input_name_alloc.get();
    std::string output_name = output_name_alloc.get();

    // 输入形状
    auto input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    int input_h = (int)input_shape[2];
    int input_w = (int)input_shape[3];

    std::cout << "Input shape: " << input_shape[0] << " "
              << input_shape[1] << " " << input_shape[2] << " " << input_shape[3] << std::endl;

    // 摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) 
    {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) 
    {
        cap >> frame;
        if (frame.empty()) break;

        float scale;
        int pad_w, pad_h;
        cv::Mat input_img = letterbox(frame, input_w, input_h, scale, pad_w, pad_h);
        std::vector<float> input_tensor = preprocess(input_img);

        // 创建输入 tensor
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> dims = {1, 3, input_h, input_w};
        Ort::Value input_ort = Ort::Value::CreateTensor<float>(mem_info, input_tensor.data(), input_tensor.size(), dims.data(), dims.size());

        // 推理
        const char* input_names[] = {input_name.c_str()};
        const char* output_names[] = {output_name.c_str()};

        auto output_tensors = session.Run(Ort::RunOptions{nullptr},
                                          input_names, &input_ort, 1,
                                          output_names, 1);

        float* out_data = output_tensors[0].GetTensorMutableData<float>();
        auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

        int64_t N = out_shape[2];

        std::vector<Detection> dets;
        float conf_thresh = 0.35f;

        for (int64_t i = 0; i < N; i++) 
        {
            float x = out_data[i];
            float y = out_data[N + i];
            float w = out_data[2 * N + i];
            float h = out_data[3 * N + i];
            float conf = out_data[4 * N + i];
            float cls = out_data[5 * N + i];

            if (conf < conf_thresh) continue;

            float cx = (x - pad_w) / scale;
            float cy = (y - pad_h) / scale;
            float bw = w / scale;
            float bh = h / scale;

            Detection det;
            det.box = cv::Rect(int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh));
            det.conf = conf;
            det.class_id = (int)cls;
            dets.push_back(det);
        }

        auto keep = nms(dets, 0.45f);

        for (auto idx : keep) 
        {
            auto& d = dets[idx];
            cv::rectangle(frame, d.box, cv::Scalar(0, 255, 0), 2);

            // 显示类别名称
            std::string label = class_names[d.class_id] + ":" + cv::format("%.2f", d.conf);
            cv::putText(frame, label, cv::Point(d.box.x, d.box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("YOLOv8 ONNX", frame);
        if (cv::waitKey(1) == 27) break;
    }

    allocator.Free(input_name_alloc.release());
    allocator.Free(output_name_alloc.release());

    return 0;
}
