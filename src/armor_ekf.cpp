#include "cpp08_armor_detector/kalman_filter.hpp"

constexpr double MAX_DT = 0.1; 

// 构造函数：初始化 8 维 EKF 的所有矩阵
ArmorEKF::ArmorEKF() : r(260) { 
    x = cv::Mat::zeros(9, 1, CV_64F); // 状态量 9 维[xc, vxc, yc, vyc, zc, vzc, yaw, vyaw, ayaw]
    P = cv::Mat::eye(9, 9, CV_64F) * 1.0; 
    is_first_predict_ = true;   
    
    // ==================== 过程噪声 Q ====================
    Q = cv::Mat::eye(9, 9, CV_64F);
    Q.at<double>(0, 0) = 0.05*1000;     // xc (q_x)
    Q.at<double>(1, 1) = 0.5*1000;      // vxc (q_v_x)
    Q.at<double>(2, 2) = 0.05*1000;     // yc (q_y)
    Q.at<double>(3, 3) = 0.5*1000;      // vyc (q_v_y)
    Q.at<double>(4, 4) = 0.05*1000;     // zc (q_z)
    Q.at<double>(5, 5) = 0.5*1000;      // vzc (q_v_z)
    Q.at<double>(6, 6) = 0.008;    // yaw (q_yaw)
    Q.at<double>(7, 7) = 0.05;     // vyaw (q_omega)
    Q.at<double>(8, 8) = 0.1;      // ayaw (q_a_omega)

    // ==================== 观测噪声 R ====================
    R = cv::Mat::eye(4, 4, CV_64F); 
    R.at<double>(0, 0) = 0.05*1000;     // xa (r_x)
    R.at<double>(1, 1) = 0.05*1000;     // ya (r_y)
    R.at<double>(2, 2) = 0.1*1000;      // za (r_z)
    R.at<double>(3, 3) = 0.4;      // yaw (r_yaw)
    
    I = cv::Mat::eye(9, 9, CV_64F); 
}

// 1. 初始化 EKF
void ArmorEKF::init(const cv::Mat& tvec, double yaw_rad,double init_timestamp) {
     //yaw_rad = CV_PI / 4.0; 
    double xa = tvec.at<double>(0, 0);
    double ya = tvec.at<double>(1, 0);
    double za = tvec.at<double>(2, 0);
    double init_yaw = yaw_rad ; 
    normalizeYaw(init_yaw);
    // 2. 利用车体 Yaw 和局部坐标 (dx=r, dy=0) 逆推中心
    // 展开公式: init_x = xa - (cos(init_yaw)*r - sin(init_yaw)*0)
    // 展开公式: init_y = ya - (sin(init_yaw)*r + cos(init_yaw)*0)
    double cos_yaw = std::cos(init_yaw);
    double sin_yaw = std::sin(init_yaw);
    
    // 核心几何模型: 中心 = 装甲板 -+ 偏移量
    x.at<double>(0, 0) = xa - cos_yaw * r; // xc
    x.at<double>(1, 0) = 0.0;              // vxc
    x.at<double>(2, 0) = ya - sin_yaw * r; // yc 
    x.at<double>(3, 0) = 0.0;              // vyc
    x.at<double>(4, 0) = za;               // zc
    x.at<double>(5, 0) = 0.0;              // vzc
    x.at<double>(6, 0) = init_yaw;         // 存入的是车体 Yaw！
    x.at<double>(7, 0) = 0.0;              // vyaw
    x.at<double>(8, 0) = 0.0;              // ayaw
    
    P = cv::Mat::eye(9, 9, CV_64F) * 1.0; 

    is_first_predict_ = true;
    prev_timestamp_ = init_timestamp; // 初始化时间戳       
}

// 2. 预测步 (匀速直线 + 匀角加速度模型)
void ArmorEKF::predict(double current_timestamp) {
   double dt;

    // 计算真实帧间隔（完全基于图像时间戳差值）
    if (is_first_predict_) {
        // 第一次预测：用初始时间戳与当前时间戳的差值
        dt = current_timestamp - prev_timestamp_;
        // 首次预测防异常：若差值无效则用30fps默认值
        if (dt <= 0 || dt > MAX_DT) {
            dt = 0.033;
        }
        is_first_predict_ = false;
    } else {
        // 非首次预测：直接计算时间戳差值（秒）
        dt = current_timestamp - prev_timestamp_;
        
        // 防跳变：超过100ms/负数（时间戳回退）就用默认值，防止卡顿时预测爆炸
        if (dt > MAX_DT || dt < 0) {
            dt = 0.033;
        }
    }
    // 更新上一帧时间戳为当前帧（统一放在此处，避免重复代码）
    prev_timestamp_ = current_timestamp;


    cv::Mat x_new = x.clone();
    
    // 状态预测
    x_new.at<double>(0, 0) += x.at<double>(1, 0) * dt; // xc += vxc * dt
    x_new.at<double>(2, 0) += x.at<double>(3, 0) * dt; // yc += vyc * dt
    x_new.at<double>(4, 0) += x.at<double>(5, 0) * dt; // zc += vzc * dt
    // yaw = yaw + vyaw * dt + 0.5 * ayaw * dt^2
    x_new.at<double>(6, 0) += x.at<double>(7, 0) * dt + 0.5 * x.at<double>(8, 0) * dt * dt; 
    // vyaw = vyaw + ayaw * dt
    x_new.at<double>(7, 0) += x.at<double>(8, 0) * dt; 
    // ayaw 保持不变 (匀角加速度假设)
    
    normalizeYaw(x_new.at<double>(6, 0)); // 归一化预测后的角度
    x = x_new;
    
    // 状态转移矩阵 F (9x9)
    cv::Mat F = cv::Mat::eye(9, 9, CV_64F);
    F.at<double>(0, 1) = dt; // d(xc)/d(vxc)
    F.at<double>(2, 3) = dt; // d(yc)/d(vyc)
    F.at<double>(4, 5) = dt; // d(zc)/d(vzc)
    F.at<double>(6, 7) = dt;               // d(yaw_new) / d(vyaw)
    F.at<double>(6, 8) = 0.5 * dt * dt;    // d(yaw_new) / d(ayaw)
    F.at<double>(7, 8) = dt;               // d(vyaw_new) / d(ayaw)
    
    P = F * P * F.t() + Q;
}

// 3. 更新步
void ArmorEKF::update(const cv::Mat& tvec, double yaw_rad) {
     //yaw_rad = CV_PI / 4.0; 
    double xc  = x.at<double>(0, 0);
    double  yc  = x.at<double>(2, 0);
    double zc  = x.at<double>(4, 0);
    double yaw = x.at<double>(6, 0);

    // 跳变检测 (防止因为装甲板切换导致角速度暴走)
    double measured_yaw_car = yaw_rad ;
    normalizeYaw(measured_yaw_car);

    // 用【测量的车体 Yaw】减去【预测的车体 Yaw】
    double yaw_diff = measured_yaw_car - yaw;
    normalizeYaw(yaw_diff); 

    // 跳变检测 (检测是否打陀螺切换了装甲板，比如切换了 90 度或 180 度)
    if (fabs(fabs(yaw_diff) - CV_PI/2) < 0.3 || fabs(fabs(yaw_diff) - CV_PI) < 0.3) {
        x.at<double>(6, 0) = measured_yaw_car; // 发生切换，直接更新为当前车体角度
        x.at<double>(7, 0) = 0.0;              // 角速度置零，防止爆炸
        x.at<double>(8, 0) = 0.0;              // 角加速度置零
        return; 
    }

    cv::Mat z = (cv::Mat_<double>(4, 1) << 
        tvec.at<double>(0, 0), 
        tvec.at<double>(1, 0), 
        tvec.at<double>(2, 0),      
        yaw_rad
    );

    // 观测函数 h(x): 预测装甲板位置
    double xa_pred = xc + r * cos(yaw); 
    double ya_pred = yc + r * sin(yaw); 
    double za_pred = zc;
    double yaw_pred = yaw;               

    cv::Mat z_pred = (cv::Mat_<double>(4, 1) << xa_pred, ya_pred, za_pred, yaw_pred);

    cv::Mat y = z - z_pred;
    normalizeYaw(y.at<double>(3, 0)); 
    if (std::abs(y.at<double>(3, 0)) > 0.6 && !is_first_predict_) {
        y.at<double>(3, 0) = 0.0; // 忽略 PnP 的错误观测，完全相信 EKF 预测
    }

    // 观测雅可比矩阵 H (4x9) —— 完全匹配 h(x) 的求导
    cv::Mat H = cv::Mat::zeros(4, 9, CV_64F);
    
    H.at<double>(0, 0) = 1;                 // d(xa)/d(xc)
    H.at<double>(0, 6) = -r * sin(yaw);     // d(xa)/d(yaw) 
    
    H.at<double>(1, 2) = 1;                 // d(ya)/d(yc)
    H.at<double>(1, 6) = r * cos(yaw);      // d(ya)/d(yaw)

    H.at<double>(2, 4) = 1;                 // d(za)/d(zc)
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
            xc + r * cos(yaw),
            yc + r * sin(yaw),
            zc);
        
    out_yaw = yaw ; 
    normalizeYaw(out_yaw);
}

// 私有工具函数: 归一化角度到 [-pi, pi]
void ArmorEKF::normalizeYaw(double& yaw) {
    while (yaw > CV_PI)  yaw -= 2 * CV_PI;
    while (yaw < -CV_PI) yaw += 2 * CV_PI;
}       