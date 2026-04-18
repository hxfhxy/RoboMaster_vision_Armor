#include "cpp08_armor_detector/kalman_filter.hpp"

// 构造函数：初始化 8 维 EKF 的所有矩阵
ArmorEKF::ArmorEKF() : r(260) { 
    x = cv::Mat::zeros(8, 1, CV_64F); // 状态量 8 维
    P = cv::Mat::eye(8, 8, CV_64F) * 1.0; 
    
    // 过程噪声 Q：调整各维度的信任度
    Q = cv::Mat::eye(8, 8, CV_64F) * 0.01;
    Q.at<double>(0, 0) = 10; // xc
    Q.at<double>(1, 1) = 300;  // vxc: 允许一定速度变化
    Q.at<double>(2, 2) = 10; // yc
    Q.at<double>(3, 3) = 300;  // vyc
    Q.at<double>(4, 4) = 10; // zc
    Q.at<double>(5, 5) = 300;  // vzc: 允许前后速度变化
    Q.at<double>(6, 6) = 0.01; // yaw
    Q.at<double>(7, 7) = 0.8;  // vyaw: 角速度变化可能较快，给大一点

    // 观测噪声 R：4 维 [xa, ya, za, yaw]
    R = cv::Mat::eye(4, 4, CV_64F) * 0.1; 
    R.at<double>(0, 0) = 10.0; // xa
    R.at<double>(1, 1) = 10.0; // ya
    R.at<double>(2, 2) = 10.0; // za
    R.at<double>(3, 3) = 0.3;  
    
    I = cv::Mat::eye(8, 8, CV_64F); 
}

// 1. 初始化 EKF
void ArmorEKF::init(const cv::Mat& tvec, double yaw_rad) {
     //yaw_rad = CV_PI / 4.0; 
    double xa = tvec.at<double>(0, 0);
    double ya = tvec.at<double>(1, 0);
    double za = tvec.at<double>(2, 0);
    
    // 核心几何模型: 中心 = 装甲板 + 偏移量
    x.at<double>(0, 0) = xa + r * sin(yaw_rad); // xc
    x.at<double>(1, 0) = 0.0;                   // vxc
    x.at<double>(2, 0) = ya;                    // yc (高度一致)
    x.at<double>(3, 0) = 0.0;                   // vyc
    x.at<double>(4, 0) = za + r * cos(yaw_rad); // zc
    x.at<double>(5, 0) = 0.0;                   // vzc
    x.at<double>(6, 0) = yaw_rad;               // yaw
    x.at<double>(7, 0) = 0.0;                   // vyaw: 初始角速度为0
    
    P = cv::Mat::eye(8, 8, CV_64F) * 1.0; 
}

// 2. 预测步 (匀速直线 + 匀角速模型)
void ArmorEKF::predict(double dt) {
    cv::Mat x_new = x.clone();
    
    // 状态预测
    x_new.at<double>(0, 0) += x.at<double>(1, 0) * dt; // xc += vxc * dt
    x_new.at<double>(2, 0) += x.at<double>(3, 0) * dt; // yc += vyc * dt
    x_new.at<double>(4, 0) += x.at<double>(5, 0) * dt; // zc += vzc * dt
    x_new.at<double>(6, 0) += x.at<double>(7, 0) * dt; // yaw += vyaw * dt
    
    normalizeYaw(x_new.at<double>(6, 0)); // 归一化预测后的角度
    x = x_new;
    
    // 状态转移矩阵 F (8x8)
    cv::Mat F = cv::Mat::eye(8, 8, CV_64F);
    F.at<double>(0, 1) = dt; // d(xc)/d(vxc)
    F.at<double>(2, 3) = dt; // d(yc)/d(vyc)
    F.at<double>(4, 5) = dt; // d(zc)/d(vzc)
    F.at<double>(6, 7) = dt; // d(yaw)/d(vyaw)
    
    P = F * P * F.t() + Q;
}

// 3. 更新步
void ArmorEKF::update(const cv::Mat& tvec, double yaw_rad) {
     //yaw_rad = CV_PI / 4.0; 
    double xc  = x.at<double>(0, 0);
    double yc  = x.at<double>(2, 0);
    double zc  = x.at<double>(4, 0);
    double yaw = x.at<double>(6, 0);

    // // 跳变检测 (防止因为装甲板切换导致角速度暴走)
    // double yaw_diff = yaw_rad - yaw;
    // normalizeYaw(yaw_diff); 
    // if (fabs(fabs(yaw_diff) - CV_PI/2) < 0.2 || fabs(fabs(yaw_diff) - CV_PI) < 0.2) {
    //     x.at<double>(6, 0) = yaw_rad; // 只更正角度
    //     // 可以考虑在这里清零角速度 x.at<double>(7, 0) = 0; 视情况而定
    //     return; 
    // }

    cv::Mat z = (cv::Mat_<double>(4, 1) << 
        tvec.at<double>(0, 0), 
        tvec.at<double>(1, 0), 
        tvec.at<double>(2, 0), 
        yaw_rad
    );

    // 观测函数 h(x): 预测装甲板位置
    double xa_pred = xc - r * sin(yaw); 
    double ya_pred = yc; 
    double za_pred = zc - r * cos(yaw);  
    double yaw_pred = yaw;               

    cv::Mat z_pred = (cv::Mat_<double>(4, 1) << xa_pred, ya_pred, za_pred, yaw_pred);

    cv::Mat y = z - z_pred;
    normalizeYaw(y.at<double>(3, 0)); 

    // 观测雅可比矩阵 H (4x8) —— 完全匹配 h(x) 的求导
    cv::Mat H = cv::Mat::zeros(4, 8, CV_64F);
    
    H.at<double>(0, 0) = 1;                 // d(xa)/d(xc)
    H.at<double>(0, 6) = -r * cos(yaw);     // d(xa)/d(yaw) 
    
    H.at<double>(1, 2) = 1;                 // d(ya)/d(yc)
    
    H.at<double>(2, 4) = 1;                 // d(za)/d(zc)
    H.at<double>(2, 6) = r * sin(yaw);      // d(za)/d(yaw)
    
    H.at<double>(3, 6) = 1;                 // d(yaw)/d(yaw)

    // EKF 更新
    cv::Mat S = H * P * H.t() + R; 
    cv::Mat K = P * H.t() * S.inv(); 
    
    x = x + K * y; 
    P = (I - K * H) * P; 
    
    normalizeYaw(x.at<double>(6, 0)); 
}

// 4. 获取预测后的装甲板
void ArmorEKF::getPredictedArmor(cv::Mat& out_tvec, double& out_yaw) {
    double xc  = x.at<double>(0, 0);
    double yc  = x.at<double>(2, 0);
    double zc  = x.at<double>(4, 0);
    double yaw = x.at<double>(6, 0);

    // 推回装甲板位置
    out_tvec = (cv::Mat_<double>(3, 1) <<
        xc - r * sin(yaw),
        yc,
        zc - r * cos(yaw));
    
    out_yaw = yaw; 
}

// 归一化函数保持不变...

// 私有工具函数: 归一化角度到 [-pi, pi]
void ArmorEKF::normalizeYaw(double& yaw) {
    while (yaw > CV_PI)  yaw -= 2 * CV_PI;
    while (yaw < -CV_PI) yaw += 2 * CV_PI;
}       