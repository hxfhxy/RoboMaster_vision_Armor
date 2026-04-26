#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <opencv2/opencv.hpp>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <cmath>

// 装甲板扩展卡尔曼滤波器（EKF）类：用于平滑目标位姿，抑制抖动
class ArmorEKF {
public:
    // 状态量 X (8维): xc​,vxc​,yc​,vyc​,zc​,vzc​,yaw,vyaw​
    cv::Mat x; 
    
    // 协方差矩阵 P (8x8)：表示状态量的不确定性
    cv::Mat P;
    
    // 过程噪声 Q (8x8)：表示模型预测的误差（重点调参对象）
    cv::Mat Q;
    
    // 观测噪声 R (4x4)：表示传感器观测的误差（重点调参对象）
    cv::Mat R;
    
    // 单位矩阵 I (8x8)
    cv::Mat I;

    // 固定参数：机器人中心到装甲板的旋转半径（单位：米，根据实际尺寸调整）
    const double r; 

    // 构造函数：初始化EKF的所有矩阵
    ArmorEKF();

    // 1. 初始化 EKF (第一次看到装甲板时调用)
    void init(const cv::Mat& tvec, double yaw_rad,double init_timestamp);

    // 2. 预测步 (每帧必做，匀速运动模型)
    void predict(double current_timestamp);


    // 3. 更新步 (看到装甲板时才做)
    void update(const cv::Mat& tvec, double yaw_rad);

    // 4. 获取滤波后的装甲板预测位置 (发给电控前用)
    void getPredictedArmor(cv::Mat& out_tvec, double& out_yaw);
    // 工具函数: 归一化角度到 [-pi, pi]
    static void normalizeYaw(double& yaw);
private:
    
    cv::TickMeter tick_meter_;  // OpenCV高精度计时器
    bool is_first_predict_;     // 标记是否第一次调用predict
    const double MAX_DT = 0.1;  // 最大允许帧间隔(100ms)，防止跳变
    double prev_timestamp_;

};

#endif // ARMOR_EKF_HPP