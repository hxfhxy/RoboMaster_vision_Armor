#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <opencv2/opencv.hpp>

class MyKalmanFilter {
public:
    cv::Mat x, P, F, Q, H, R, I;

    // 状态数=2 [角度, 角速度], 测量数=1 [角度]
    MyKalmanFilter(int stateNum = 2, int measureNum = 1) {
        x = cv::Mat::zeros(stateNum, 1, CV_32F);
        P = cv::Mat::eye(stateNum, stateNum, CV_32F);
        
        // 1. 状态转移矩阵 F: x_new = x + v*dt
        // 设 dt = 0.02 
        float dt = 0.02;
        F = (cv::Mat_<float>(2, 2) << 1, dt, 
                                      0, 1);

        // 2. 过程噪声 Q: 对预测模型的信任程度 
        Q = (cv::Mat_<float>(2, 2) << 0.01, 0, 
                                      0,    0.01);

        // 3. 测量矩阵 H: 只测量位置，不测量速度
        H = (cv::Mat_<float>(1, 2) << 1, 0);

        // 4. 测量噪声 R: 对视觉识别结果的信任程度 
        R = cv::Mat::eye(measureNum, measureNum, CV_32F) * 0.5;
        
        I = cv::Mat::eye(stateNum, stateNum, CV_32F);
    }

    void predict() {
        x = F * x;
        P = F * P * F.t() + Q;
    }

    void correct(cv::Mat z) {
    // 1. 先计算预测的测量值 (Hx) 并存入临时变量
    cv::Mat prediction = H * x; 

    float diff = z.at<float>(0, 0) - prediction.at<float>(0, 0);
    
    // 3. 处理角度环跳变 
    if (diff > 180.0) diff -= 360.0;
    else if (diff < -180.0) diff += 360.0;

    // 4. 标准卡尔曼更新步骤
    cv::Mat S = H * P * H.t() + R;
    cv::Mat K = P * H.t() * S.inv();
    
    // 5. 更新状态
    cv::Mat innovation = (cv::Mat_<float>(1, 1) << diff);
    x = x + K * innovation;
    P = (I - K * H) * P;
  }
};

#endif