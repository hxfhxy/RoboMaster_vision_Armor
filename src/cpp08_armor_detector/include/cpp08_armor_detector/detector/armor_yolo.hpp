#ifndef ARMOR_DETECTOR_YOLO_HPP
#define ARMOR_DETECTOR_YOLO_HPP

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "cpp08_armor_detector/detector/armor_detector_matching.hpp"

class ArmorYolo {
public:
    ArmorYolo();// 构造函数
    void loadModel(const std::string& model_path);// 加载ONNX模型
    std::vector<DetectedArmor> detect(const cv::Mat& img);// 进行装甲板检测，返回检测结果列表

    void setDetectColor(int color) { detect_color_ = color; }// 设置检测颜色（0=蓝色，1=红色）
    void setConfThreshold(float thr) { conf_threshold_ = thr; }// 设置置信度阈值
    void setNmsThreshold(float thr) { nms_threshold_ = thr; }// 设置NMS阈值

private:
    cv::dnn::Net net_;// DNN网络对象
    float conf_threshold_ = 0.65f;// 置信度阈值
    float nms_threshold_ = 0.45f;// NMS阈值
    int detect_color_ = 0;// 检测颜色（0=蓝色，1=红色）
    static constexpr int INPUT_WIDTH = 640;// 模型输入宽度
    static constexpr int INPUT_HEIGHT = 640;// 模型输入高度
};

#endif // ARMOR_DETECTOR_YOLO_HPP
