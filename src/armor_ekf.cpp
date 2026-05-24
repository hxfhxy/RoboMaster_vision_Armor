#include "cpp08_armor_detector/kalman_filter.hpp"
#include <algorithm> // 必须包含，用于 std::clamp

constexpr double MAX_DT = 0.1; 

// 1. 初始化 11 维 EKF
ArmorEKF::ArmorEKF() { 
    // 状态量 11 维: [xc, vxc, yc, vyc, zc, vzc, yaw, vyaw, r1(前后半径), r2(左右半径), dz(左右高度差)]
    x = cv::Mat::zeros(11, 1, CV_64F); 
    P = cv::Mat::eye(11, 11, CV_64F) * 1.0; 
    is_first_predict_ = true;   
    
    // ==================== 过程噪声 Q ====================
    Q = cv::Mat::eye(11, 11, CV_64F);
    Q.at<double>(0, 0) = 10;       // xc
    Q.at<double>(1, 1) = 1000;       // vxc
    Q.at<double>(2, 2) = 10;       // yc
    Q.at<double>(3, 3) = 1000;       // vyc
    Q.at<double>(4, 4) = 0.0001;       // zc
    Q.at<double>(5, 5) = 1;       // vzc
    Q.at<double>(6, 6) = 0.5;      // yaw
    Q.at<double>(7, 7) = 100;       // vyaw
    Q.at<double>(8, 8) = 0.1;      // r1 
    Q.at<double>(9, 9) = 0.1;      // r2 
    Q.at<double>(10, 10) = 1.0;    // dz

    // ==================== 测量噪声 R (基础值) ====================
    R = cv::Mat::eye(4, 4, CV_64F);
    R.at<double>(0, 0) = 0.001; // yaw_cam 弧度方差 (非常信任)
    R.at<double>(1, 1) = 0.001; // pitch_cam 弧度方差 (非常信任)
    R.at<double>(2, 2) = 50.0; // distance 毫米方差 (PnP深度不准，给大一点)
    R.at<double>(3, 3) = 0.5;   // armor_yaw 弧度方差
    
    // ==================== 测量矩阵 H (4x11) 动态生成 ====================
    I = cv::Mat::eye(11, 11, CV_64F); 
}

// 2. 初始化第一帧
void ArmorEKF::init(const cv::Mat& tvec, double yaw_rad, double init_timestamp) {
    double xa = tvec.at<double>(0, 0);
    double ya = tvec.at<double>(1, 0);
    double za = tvec.at<double>(2, 0);
    
    double init_yaw = yaw_rad; 
    normalizeYaw(init_yaw);
    
    double cos_yaw = std::cos(init_yaw);
    double sin_yaw = std::sin(init_yaw);
    
    // 默认初始半径为 260mm
    double init_r = 260.0;
    
    x.at<double>(0, 0) = xa + cos_yaw * init_r; // xc
    x.at<double>(1, 0) = 0.0;                   // vxc
    x.at<double>(2, 0) = ya + sin_yaw * init_r; // yc 
    x.at<double>(3, 0) = 0.0;                   // vyc
    x.at<double>(4, 0) = za;                    // zc
    x.at<double>(5, 0) = 0.0;                   // vzc
    x.at<double>(6, 0) = init_yaw;              // yaw
    x.at<double>(7, 0) = 0.0;                   // vyaw
    
    // 初始化形状参数
    x.at<double>(8, 0) = init_r;                // r1 (前后板)
    x.at<double>(9, 0) = init_r;                // r2 (左右板)
    x.at<double>(10, 0) = 30.0;                  // dz 
    
    P = cv::Mat::eye(11, 11, CV_64F) * 1.0; 
    // 给形状参数一个较大的初始不确定度，让它能快速被观测修正
    P.at<double>(8, 8) = 100.0;
    P.at<double>(9, 9) = 100.0;
    P.at<double>(4, 4) = 1.0;       // zc 的初始不确定度较小
    P.at<double>(10, 10) = 500.0;    // dz 的初始不确定度要足够大，让它在切换前几帧迅速吸收高度差

    is_first_predict_ = true;
    prev_timestamp_ = init_timestamp;      
}

// 3. 预测步
void ArmorEKF::predict(double timestamp) {
    if (is_first_predict_) {
        prev_timestamp_ = timestamp;
        is_first_predict_ = false;
        return;
    }

    double dt = timestamp - prev_timestamp_;
    if (dt > MAX_DT || dt <= 0.0) {
        dt = 0.01; 
    }
    prev_timestamp_ = timestamp;

    cv::Mat F = cv::Mat::eye(11, 11, CV_64F);
    F.at<double>(0, 1) = dt; // xc += vxc * dt
    F.at<double>(2, 3) = dt; // yc += vyc * dt
    F.at<double>(4, 5) = dt; // zc += vzc * dt
    F.at<double>(6, 7) = dt; // yaw += vyaw * dt

    x = F * x; 
    normalizeYaw(x.at<double>(6, 0)); 
    
    P = F * P * F.t() + Q; 
}

// 4. 更新步
void ArmorEKF::update(const cv::Mat& tvec, double yaw_rad) {
    double xc  = x.at<double>(0, 0);// 车体中心点的世界坐标 X
    double yc  = x.at<double>(2, 0);// 车体中心点的世界坐标 Y
    double zc  = x.at<double>(4, 0);// 车体中心点的世界坐标 Z
    double yaw = x.at<double>(6, 0);// 车体的绝对偏航角 (世界坐标系)
    double r1  = x.at<double>(8, 0); // 前后板半径
    double r2  = x.at<double>(9, 0); // 左右板半径
    double dz  = x.at<double>(10, 0); // 左右板高度偏置
    
    double measured_yaw_car = yaw_rad;
    normalizeYaw(measured_yaw_car);
    
    int best_id = 0;
    double min_cost = 1e10;
    double best_plate_yaw = yaw;
    double current_r = r1, current_z = zc;
    for (int i = 0; i < 4; i++) {
        double pred_yaw = yaw + i * (CV_PI / 2.0);
        normalizeYaw(pred_yaw);
        
        bool is_side = (i == 1 || i == 3);
        double R = is_side ? r2 : r1;
        double Z = is_side ? zc + dz : zc;
        
        double px = xc - R * cos(pred_yaw);
        double py = yc - R * sin(pred_yaw);
        
        // 1. 距离误差
        double pred_dist = std::sqrt(px*px + py*py + Z*Z);
        double obs_dist = cv::norm(tvec);
        double distance_err = std::abs(obs_dist - pred_dist);
        
        // 2. Center Yaw 相位误差 (装甲板在视野中的位置)
        double pred_yaw_cam = std::atan2(py, px);
        double obs_yaw_cam = std::atan2(tvec.at<double>(1, 0), tvec.at<double>(0, 0));
        double center_yaw_err = std::abs(obs_yaw_cam - pred_yaw_cam);
        normalizeYaw(center_yaw_err);
        // 3. Pitch 高度角误差 (防高矮板混淆)
        double pred_pitch_cam = std::atan2(Z, std::sqrt(px*px + py*py));
        double obs_pitch_cam = std::atan2(tvec.at<double>(2, 0), std::sqrt(tvec.at<double>(0, 0)*tvec.at<double>(0, 0) + tvec.at<double>(1, 0)*tvec.at<double>(1, 0)));
        double pitch_err = std::abs(obs_pitch_cam - pred_pitch_cam);
        // 4. Armor Yaw 自身朝向误差
        double armor_yaw_err = measured_yaw_car - pred_yaw;
        normalizeYaw(armor_yaw_err);
        // 济瞄/陈君代价函数权重
        double cost = std::abs(center_yaw_err) * 10.0 + std::abs(armor_yaw_err) * 10.0 + distance_err * 0.02 + pitch_err * 7.0;
        
        if (cost < min_cost) {
            min_cost = cost;
            best_id = i;
            best_plate_yaw = pred_yaw;
            current_r = R;
            current_z = Z;
        }
    }
    if (min_cost > 150.0) {
        return; // 直接退出 update，靠惯性盲推
    }
   
    // 1. 计算球面坐标系下的预测观测值 z_pred 
    double xa_pred = xc - current_r * cos(best_plate_yaw); 
    double ya_pred = yc - current_r * sin(best_plate_yaw); 
    double za_pred = current_z; 
    double pred_d2 = xa_pred * xa_pred + ya_pred * ya_pred;
    double pred_dist = std::sqrt(pred_d2 + za_pred * za_pred);
    double pred_yaw_cam = std::atan2(ya_pred, xa_pred);
    double pred_pitch_cam = std::atan2(za_pred, std::sqrt(pred_d2));
    cv::Mat z_pred = (cv::Mat_<double>(4, 1) << pred_yaw_cam, pred_pitch_cam, pred_dist, best_plate_yaw);
    //  2. 计算球面坐标系下的实际观测值 z_obs 
    double obs_x = tvec.at<double>(0, 0);
    double obs_y = tvec.at<double>(1, 0);
    double obs_z = tvec.at<double>(2, 0);
    
    double obs_d2 = obs_x * obs_x + obs_y * obs_y;
    double obs_dist = std::sqrt(obs_d2 + obs_z * obs_z);
    double obs_yaw_cam = std::atan2(obs_y, obs_x);
    double obs_pitch_cam = std::atan2(obs_z, std::sqrt(obs_d2));
    cv::Mat z_obs = (cv::Mat_<double>(4, 1) << obs_yaw_cam, obs_pitch_cam, obs_dist, measured_yaw_car);
    // 3. 计算残差并处理角度绕回 
    cv::Mat y = z_obs - z_pred;
    normalizeYaw(y.at<double>(0, 0)); // 处理相机 yaw 跨越 180 度的问题
    normalizeYaw(y.at<double>(1, 0)); // 处理相机 pitch
    normalizeYaw(y.at<double>(3, 0)); // 用标准的角度归一化
    
    if (best_id != last_best_id_) {
        last_best_id_ = best_id;
        //return;
    }
    last_best_id_ = best_id;
    
    // 正常帧，平滑更新R
    // ==========================================================
    // 1. 获取从相机原点指向装甲板的视线角度 (Line of Sight)
    double obs_yaw_cam_world = std::atan2(tvec.at<double>(1, 0), tvec.at<double>(0, 0)); 
    
    // 2. 因为 bestYaw_rad_world 加了负号，变成了指向车心的向内法线
    // 当装甲板正对相机时，它与视线方向刚好完美重合！
    double angle_diff = measured_yaw_car - obs_yaw_cam_world;
    normalizeYaw(angle_diff);
    
    // 3. 真实的透视倾斜角：直接取绝对值，0度代表完美正对，角度越大说明越侧偏
    double abs_angle = std::abs(angle_diff); 
    // ==========================================================
    
    // 因为 d 很大导致 H 极小，我们必须同步缩小 R 才能维持 K 的拉力
    double scale = 1000.0 / (obs_dist + 1.0); 
    double base_R_angle = 1e-6 * scale * scale; // 动态自适应缩小
    R.at<double>(0, 0) = base_R_angle + 1e-5 * abs_angle; 
    R.at<double>(1, 1) = base_R_angle + 1e-5 * abs_angle; 
    R.at<double>(3, 3) = 0.3  + 0.5 * abs_angle;   // 进一步增大装甲板yaw的噪声
    
    double dist_m = cv::norm(tvec) / 1000.0;
    if (abs_angle > 0.6) { 
        R.at<double>(2, 2) = 10000.0; // 边缘化盲区，彻底拒信深度
    } else {
        R.at<double>(2, 2) = 50.0 + 150.0 * abs_angle + 20.0 * (dist_m * dist_m); 
    }
    
    // 第一次更新：只用位置观测修正中心x和y
    cv::Mat H_pos = cv::Mat::zeros(4, 11, CV_64F);
    H_pos.at<double>(0, 0) = 1; // d(xa)/d(xc)
    H_pos.at<double>(1, 2) = 1; // d(ya)/d(yc)
    
    // 针对不同装甲板的物理隔离
    if (best_id == 0 || best_id == 2) {
        H_pos.at<double>(0, 8) = -std::cos(best_plate_yaw);          
        H_pos.at<double>(1, 8) = -std::sin(best_plate_yaw);          
        H_pos.at<double>(2, 4) = 1.0;   // d(za)/d(zc)
        H_pos.at<double>(2, 10) = 0.0;  // d(za)/d(dz)
    } else {
        H_pos.at<double>(0, 9) = -std::cos(best_plate_yaw);          
        H_pos.at<double>(1, 9) = -std::sin(best_plate_yaw);          
        H_pos.at<double>(2, 4) = 1.0;   
        H_pos.at<double>(2, 10) = 1.0;  
    }
    
    // 转换为球面坐标系
    cv::Mat J_3x3 = get_ypd_jacobian(xa_pred, ya_pred, za_pred);
    cv::Mat J_4x4 = cv::Mat::zeros(4, 4, CV_64F);
    J_3x3.copyTo(J_4x4(cv::Rect(0, 0, 3, 3)));
    J_4x4.at<double>(3, 3) = 1.0;
    H_pos = J_4x4 * H_pos;
    
    // 位置更新：只修正x, y, z, r1, r2, dz，不修正yaw
    cv::Mat R_pos = R.clone();
    R_pos.at<double>(3, 3) = 1e10; // 彻底拒信yaw观测
    
    cv::Mat S_pos = H_pos * P * H_pos.t() + R_pos; 
    cv::Mat K_pos = P * H_pos.t() * S_pos.inv(); 
    x = x + K_pos * y; 
    P = (I - K_pos * H_pos) * P; 
    
    // 第二次更新：只用yaw观测修正yaw和vyaw
    cv::Mat H_yaw = cv::Mat::zeros(4, 11, CV_64F);
    H_yaw.at<double>(3, 6) = 1; // 只有yaw对装甲板yaw的偏导
    
    cv::Mat R_yaw = R.clone();
    R_yaw.at<double>(0, 0) = 1e10; // 彻底拒信位置观测
    R_yaw.at<double>(1, 1) = 1e10;
    R_yaw.at<double>(2, 2) = 1e10;
    
    cv::Mat S_yaw = H_yaw * P * H_yaw.t() + R_yaw; 
    cv::Mat K_yaw = P * H_yaw.t() * S_yaw.inv(); 
    x = x + K_yaw * y; 
    P = (I - K_yaw * H_yaw) * P; 
    
    normalizeYaw(x.at<double>(6, 0)); 
    // 1. 赋予 XY 运动“阻尼（摩擦力）”
    x.at<double>(1, 0) *= 0.995; // v_x 衰减
    x.at<double>(3, 0) *= 0.995; // v_y 衰减
    x.at<double>(7, 0) *= 0.995;  // 增大yaw阻尼，防止振荡
    // 2. 彻底锁死 Z 轴速度 
    x.at<double>(5, 0) = 0.0;
    // 将半径强行限制在 150mm 到 400mm 之间
    x.at<double>(8, 0) = std::clamp(x.at<double>(8, 0), 150.0, 400.0);
    x.at<double>(9, 0) = std::clamp(x.at<double>(9, 0), 150.0, 400.0);
    // 将左右板的高度差限制在正负 100mm 之间
    x.at<double>(10, 0) = std::clamp(x.at<double>(10, 0), -100.0, 100.0);
    // 限制最大速度，防止异常观测导致爆炸
    x.at<double>(1, 0) = std::clamp(x.at<double>(1, 0), -5000.0, 5000.0);
    x.at<double>(3, 0) = std::clamp(x.at<double>(3, 0), -5000.0, 5000.0);
    x.at<double>(7, 0) = std::clamp(x.at<double>(7, 0), -10.0, 10.0); //
    
    // std::cout << "[EKF Debug] "
    //       << "xc: " << x.at<double>(0, 0) << " | "
    //       << "yc: " << x.at<double>(2, 0) << " | "
    //       << "zc: " << x.at<double>(4, 0) << " | "
    //       << "dz: " << x.at<double>(10, 0) << std::endl;
}


// 5. 提取预测 
void ArmorEKF::getPredictedArmor(cv::Mat& out_tvec, double& out_yaw) {
    double xc  = x.at<double>(0, 0);
    double yc  = x.at<double>(2, 0);
    double zc  = x.at<double>(4, 0);
    double yaw = x.at<double>(6, 0);
    double r1  = x.at<double>(8, 0); // 输出正面时使用前板半径 r1

    out_tvec = (cv::Mat_<double>(3, 1) <<
            xc - r1 * cos(yaw),
            yc - r1 * sin(yaw),
            zc);
        
    out_yaw = yaw; 
    normalizeYaw(out_yaw);
}

// 工具函数不变
void ArmorEKF::normalizeYaw(double& yaw) {
    while (yaw > CV_PI) yaw -= 2.0 * CV_PI;
    while (yaw < -CV_PI) yaw += 2.0 * CV_PI;
}

cv::Mat ArmorEKF::get_ypd_jacobian(double x, double y, double z) {
    cv::Mat J = cv::Mat::zeros(3, 3, CV_64F);
    double d2 = x * x + y * y;
    double d = std::sqrt(d2);
    double d3 = d2 + z * z;
    double d_3d = std::sqrt(d3);

    if (d2 < 1e-5) d2 = 1e-5; // 防止除以0
    if (d3 < 1e-5) d3 = 1e-5;

    // d(yaw) / d(x, y, z)
    J.at<double>(0, 0) = -y / d2;
    J.at<double>(0, 1) = x / d2;
    J.at<double>(0, 2) = 0.0;
    
    // d(pitch) / d(x, y, z)
    J.at<double>(1, 0) = -x * z / (d * d3);
    J.at<double>(1, 1) = -y * z / (d * d3);
    J.at<double>(1, 2) = d / d3;
    
    // d(distance) / d(x, y, z)
    J.at<double>(2, 0) = x / d_3d;
    J.at<double>(2, 1) = y / d_3d;
    J.at<double>(2, 2) = z / d_3d;
    
    return J;
}