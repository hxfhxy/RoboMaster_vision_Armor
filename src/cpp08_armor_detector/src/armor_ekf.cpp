#include "cpp08_armor_detector/kalman_filter.hpp"

// 构造函数：初始化EKF的所有矩阵
ArmorEKF::ArmorEKF() : r(0.26) { // const成员r必须在初始化列表里赋值
    x = cv::Mat::zeros(6, 1, CV_64F); // 状态量初始化为0        
    P = cv::Mat::eye(6, 6, CV_64F) * 1.0; // 协方差矩阵初始化为单位矩阵
    
    // 过程噪声 Q：值越大越相信预测，越小越相信观测
    Q = cv::Mat::eye(6, 6, CV_64F) * 0.01;
    Q.at<double>(1, 1) = 0.05;  // vxc: 允许x方向速度有一定变化
    Q.at<double>(3, 3) = 0.05;  // vyc: 允许y方向速度有一定变化
    Q.at<double>(4, 4) = 0.01;  // yaw: 允许角度有一定变化
    
    // 观测噪声 R：值越大越不相信观测，滤波效果越强
    R = cv::Mat::eye(4, 4, CV_64F) * 0.5;  // 增大观测噪声，让滤波起作用
    R.at<double>(3, 3) = 0.1;                 // yaw 稍微信任一点（角度观测相对稳定）
    
    I = cv::Mat::eye(6, 6, CV_64F); // 初始化单位矩阵
}

// 1. 初始化 EKF (第一次看到装甲板时调用)
void ArmorEKF::init(const cv::Mat& tvec, double yaw_rad) {
    // 提取装甲板的3D位置（相机坐标系）
    double xa = tvec.at<double>(0, 0);
    double ya = tvec.at<double>(1, 0);
    double za = tvec.at<double>(2, 0);
    
    // 核心公式: 机器人中心 = 装甲板位置 + 半径 * 旋转向量
    x.at<double>(0, 0) = xa + r * cos(yaw_rad); // xc: 中心x坐标
    x.at<double>(1, 0) = 0.0;                    // vxc: 初始速度为0
    x.at<double>(2, 0) = ya + r * sin(yaw_rad); // yc: 中心y坐标
    x.at<double>(3, 0) = 0.0;                    // vyc: 初始速度为0
    x.at<double>(4, 0) = yaw_rad;                // yaw: 初始偏航角
    x.at<double>(5, 0) = za;                     // za: 初始深度
    
    P = cv::Mat::eye(6, 6, CV_64F) * 1.0; // 重置协方差矩阵
}

// 2. 预测步 (每帧必做，匀速运动模型)
void ArmorEKF::predict(double dt) {
    cv::Mat x_new = x.clone();
    
    // 状态预测：匀速模型
    // xc_new = xc + vxc * dt（x方向位置更新）
    x_new.at<double>(0, 0) = x.at<double>(0, 0) + x.at<double>(1, 0) * dt;
    // yc_new = yc + vyc * dt（y方向位置更新）
    x_new.at<double>(2, 0) = x.at<double>(2, 0) + x.at<double>(3, 0) * dt;
    // yaw_new = yaw (角度不预测，靠观测更新)
    // za_new = za (深度不预测，靠观测更新)
    
    x = x_new; // 更新状态量
    
    // 计算状态转移矩阵 F (6x6)：描述状态如何从k时刻转移到k+1时刻
    cv::Mat F = cv::Mat::eye(6, 6, CV_64F);
    F.at<double>(0, 1) = dt; // d(xc)/d(vxc) = dt（x位置对x速度的偏导）
    F.at<double>(2, 3) = dt; // d(yc)/d(vyc) = dt（y位置对y速度的偏导）
    
    // 协方差预测：P = F*P*F^T + Q
    P = F * P * F.t() + Q;
}

// 3. 更新步 (看到装甲板时才做)
void ArmorEKF::update(const cv::Mat& tvec, double yaw_rad) {
    // 提取当前预测的状态量
    double xc = x.at<double>(0, 0);
    double yc = x.at<double>(2, 0);
    double yaw = x.at<double>(4, 0);
    double za = x.at<double>(5, 0);

    // ========== 【关键】装甲板跳变检测 ==========
    // 计算观测角度与预测角度的差值
    double yaw_diff = yaw_rad - yaw;
    normalizeYaw(yaw_diff); // 归一化角度差到[-pi, pi]
    
    // 如果跳变接近 90° 或 180°，说明可能切换了装甲板，只更新 yaw，不修正中心和速度！
    if (fabs(fabs(yaw_diff) - CV_PI/2) < 0.2 || 
        fabs(fabs(yaw_diff) - CV_PI) < 0.2) {
        x.at<double>(4, 0) = yaw_rad; // 只更新 yaw
        return; // 直接返回，不做 EKF 更新
    }
    // ========== 跳变检测结束 ==========

    // 1. 组装观测向量 Z (4维): [x, y, z, yaw]（装甲板的观测位置和角度）
    cv::Mat z = (cv::Mat_<double>(4, 1) << 
        tvec.at<double>(0, 0),
        tvec.at<double>(1, 0),
        tvec.at<double>(2, 0),
        yaw_rad);

    // 2. 观测函数 h(x): 从状态量预测观测值（装甲板 = 中心 - 半径 * 旋转向量）
    double xa_ped = xc - r * cos(yaw); // 预测的装甲板x
    double ya_pred = yc - r * sin(yaw); // 预测的装甲板y
    double za_pred = za;                  // 预测的装甲板z
    double yaw_pred = yaw;                // 预测的装甲板yaw

    cv::Mat z_pred = (cv::Mat_<double>(4, 1) << 
        xa_pred, ya_pred, za_pred, yaw_pred);

    // 3. 计算残差：y = 观测值 - 预测值
    cv::Mat y = z - z_pred;
    normalizeYaw(y.at<double>(3, 0)); // 对角度残差归一化

    // 4. 计算观测雅可比矩阵 H (4x6)：h(x)对状态量x的偏导
    cv::Mat H = cv::Mat::zeros(4, 6, CV_64F);
    
    // 观测量 0: xa（装甲板x）
    H.at<double>(0, 0) = 1;           // d(xa)/d(xc) = 1
    H.at<double>(0, 4) = r * sin(yaw); // d(xa)/d(yaw) = r*sin(yaw)
    
    // 观测量 1: ya（装甲板y）
    H.at<double>(1, 2) = 1;           // d(ya)/d(yc) = 1
    H.at<double>(1, 4) = -r * cos(yaw); // d(ya)/d(yaw) = -r*cos(yaw)
    
    // 观测量 2: za（装甲板z）
    H.at<double>(2, 5) = 1;           // d(za)/d(za) = 1
    
    // 观测量 3: yaw（装甲板偏航角）
    H.at<double>(3, 4) = 1;           // d(yaw)/d(yaw) = 1

    // 5. 标准 EKF 更新公式
    cv::Mat S = H * P * H.t() + R; // 残差协方差
    cv::Mat K = P * H.t() * S.inv(); // 卡尔曼增益
    
    x = x + K * y; // 更新状态量
    P = (I - K * H) * P; // 更新协方差矩阵
    
    normalizeYaw(x.at<double>(4, 0)); // 归一化更新后的yaw角度
}

// 4. 获取滤波后的装甲板预测位置 (发给电控前用)
void ArmorEKF::getPredictedArmor(cv::Mat& out_tvec, double& out_yaw) {
    // 提取滤波后的状态量
    double xc = x.at<double>(0, 0);
    double yc = x.at<double>(2, 0);
    double yaw = x.at<double>(4, 0);
    double za = x.at<double>(5, 0);

    // 反推装甲板位置：装甲板 = 中心 - 半径 * 旋转向量
    out_tvec = (cv::Mat_<double>(3, 1) <<
        xc - r * cos(yaw),
        yc - r * sin(yaw),
        za);
    
    out_yaw = yaw; // 输出滤波后的yaw角
}

// 私有工具函数: 归一化角度到 [-pi, pi]
void ArmorEKF::normalizeYaw(double& yaw) {
    while (yaw > CV_PI)  yaw -= 2 * CV_PI;
    while (yaw < -CV_PI) yaw += 2 * CV_PI;
}
