#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// 图像预处理，包括缩放、减均值、归一化
cv::Mat preprocess(const cv::Mat& frame, const cv::Size& size) {
    cv::Mat resized, img_float;
    cv::resize(frame, resized, size);
    resized.convertTo(img_float, CV_32F, 1.0 / 255);
    return img_float;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_onnx_model> <path_to_image>\n";
        return -1;
    }

    std::string model_path = argv[1];
    std::string image_path = argv[2];

    // 读取图像
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Error: Cannot read image from path: " << image_path << std::endl;
        return -1;
    }
    
    // 图像预处理
    cv::Size input_size(416, 416);
    cv::Mat preprocessed_image = preprocess(image, input_size);

    // 创建ONNX Runtime会话
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolov4_tiny_onnx_runtime");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env, model_path.c_str(), session_options);

    // 准备输入
    std::vector<int64_t> input_dims = {1, 3, input_size.height, input_size.width};
    std::vector<float> input_tensor_values(preprocessed_image.begin<float>(), preprocessed_image.end<float>());
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_dims.data(), input_dims.size());

    // 运行模型
    const char* input_names[] = {"input"};
    const char* output_names[] = {"output"};
    std::vector<Ort::Value> output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    // 获取输出张量
    Ort::Value& output_tensor = output_tensors[0];
    const int64_t* output_dims = output_tensor.GetTensorTypeAndShapeInfo().GetShape().data();
    const float* output_data = output_tensor.GetTensorMutableData<float>();

    // 处理输出结果，进行后处理并显示检测结果
    // ...

    return 0;
}