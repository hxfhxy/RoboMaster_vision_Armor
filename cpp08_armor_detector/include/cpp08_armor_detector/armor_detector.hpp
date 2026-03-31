#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include "kalman_filter.hpp"
#include "armor_detector_lightbar.hpp"
#include "armor_detector_matching.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include "cpp08_armor_detector/msg/armor_target.hpp"

// 装甲板识别核心类

class ArmorDetector {
public:
    // 检测参数常量
    static constexpr float HEIGHT_RATIO_THRESH = 0.2f;
    static constexpr float Y_OFFSET_RATIO_THRESH = 0.3f;
    static constexpr float X_DIST_RATIO_THRESH = 5.0f;
    static constexpr int MIN_LIGHTBAR_AREA = 350;
    static constexpr int COLOR_DIFF_THRESH = 15;   // 颜色阈值
    static constexpr int BRIGHTNESS_THRESH = 30;   // 亮度阈值

    // 相机参数
    cv::Mat cameraMatrix, distCoeffs;   
    // 装甲板物理坐标
    std::vector<cv::Point3f> objectPoints;
    // 卡尔曼滤波器
    MyKalmanFilter kf{2, 1};
    // 滤波数据缓存
    std::deque<float> rawYawList, filteredYawList;

    // 目标颜色设定：0 代表蓝色，1 代表红色
    int enemy_color = 0;   

    //find_robot_center相关成员 
    struct CenterFit {
        cv::Point3f position;   // 装甲板3D位置
        cv::Point3f normalvector; // 装甲板法向量
    };
    std::vector<CenterFit> center_fits;
    std::vector<cv::Point3f> center_3d;
    std::vector<cv::Point2f> center_2d;
    bool found = false;
    bool find_center = false;
    bool first_frame = true;
    cv::Point2f last_center_2d;

    // 构造函数：初始化参数
    ArmorDetector();

    // 图像预处理
    cv::Mat preprocess(cv::Mat img);

    // 绘制Yaw滤波图
    void drawYawPlot();

    // 核心识别逻辑
    cpp08_armor_detector::msg::ArmorTarget detect(cv::Mat img);

    // find_robot_center成员函数声明
    void find_robot_center();
};

#endif // ARMOR_DETECTOR_HPP
