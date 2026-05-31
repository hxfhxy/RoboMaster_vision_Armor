#include "cpp08_armor_detector/tracker/kalman_filter.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 装甲板EKF构造函数，初始化默认参数
 * 状态向量x维度11：[cx, vx, cy, vy, cz, vz, yaw, vyaw, r, l, h]
 * cx/cy/cz：机器人中心坐标；vx/vy/vz：机器人速度；yaw：机器人偏航角；vyaw：偏航角速度
 * r：基准装甲板半径；l：侧板装甲板半径增量；h：侧板装甲板高度偏移
 */
ArmorEKF::ArmorEKF(){
    x.setZero();                                         // 状态向量初始化为0
    P = Eigen::Matrix<double, 11, 11>::Identity() * 100.0; // 初始协方差矩阵（单位矩阵×100）
}

/**
 * @brief 初始化EKF状态和协方差
 * @param p_armor_world 装甲板在世界坐标系的初始位置（米）
 * @param orientation_yaw 装甲板初始偏航角（弧度）
 * @param armor_num 装甲板数量（默认4）
 * @param init_r 基准装甲板初始半径（米）
 * @param init_l 侧板装甲板初始半径增量（米）
 * @param init_h 侧板装甲板初始高度偏移（米）
 * @param P0_diag 初始协方差矩阵的对角元素
 */
void ArmorEKF::init(const Eigen::Vector3d& p_armor_world,
                    double orientation_yaw,
                    int armor_num,
                    double init_r,
                    double init_l,
                    double init_h,
                    const Eigen::Matrix<double, 11, 1>& P0_diag)
{
    armor_num_ = armor_num;      // 装甲板数量
    last_id_ = 0;                // 上一帧匹配的装甲板ID
    update_count_ = 0;           // 更新次数
    converged_ = false;          // 收敛标志

    // 从装甲板中心反算机器人旋转中心坐标
    double cx = p_armor_world(0) + init_r * std::cos(orientation_yaw);
    double cy = p_armor_world(1) + init_r * std::sin(orientation_yaw);
    double cz = p_armor_world(2);

    // 初始化状态向量：[cx, vx, cy, vy, cz, vz, yaw, vyaw, r, l, h]
    x << cx, 0.0, cy, 0.0, cz, 0.0, orientation_yaw, 0.0, init_r, init_l, init_h;
    // 初始化协方差矩阵（对角矩阵）
    P = P0_diag.asDiagonal();
}

/**
 * @brief EKF预测步骤，基于匀速运动模型更新状态和协方差
 * @param dt 时间差（秒）
 */
void ArmorEKF::predict(double dt) {
    // 时间差无效时直接返回
    if (dt <= 0.0) return;

    // 1. 构建状态转移矩阵F（11x11）—— 匀速运动模型
    Eigen::Matrix<double, 11, 11> F = Eigen::Matrix<double, 11, 11>::Identity();
    F(0, 1) = dt;  // cx += vx * dt
    F(2, 3) = dt;  // cy += vy * dt
    F(4, 5) = dt;  // cz += vz * dt
    F(6, 7) = dt;  // yaw += vyaw * dt

    // 2. 构建过程噪声协方差矩阵Q（分段白噪声模型）
    double accel_var_xy = 20.0;   // 水平方向加速度方差
    double accel_var_z  = 0.01;   // Z轴加速度方差（抑制Z轴抖动）
    double ang_accel_var = 100.0; // 角加速度方差

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    double pos_corr = dt4 / 4.0;   // 位置-位置相关系数
    double vel_corr = dt3 / 2.0;   // 位置-速度相关系数
    double acc_corr = dt2;         // 速度-速度相关系数

    Eigen::Matrix<double, 11, 11> Q = Eigen::Matrix<double, 11, 11>::Zero();

    // x轴 (位置索引0，速度索引1)
    Q(0,0) = pos_corr * accel_var_xy;
    Q(0,1) = vel_corr * accel_var_xy;
    Q(1,0) = vel_corr * accel_var_xy;
    Q(1,1) = acc_corr * accel_var_xy;

    // y轴 (位置索引2，速度索引3)
    Q(2,2) = pos_corr * accel_var_xy;
    Q(2,3) = vel_corr * accel_var_xy;
    Q(3,2) = vel_corr * accel_var_xy;
    Q(3,3) = acc_corr * accel_var_xy;

    // z轴 (位置索引4，速度索引5)
    Q(4,4) = pos_corr * accel_var_z;
    Q(4,5) = vel_corr * accel_var_z;
    Q(5,4) = vel_corr * accel_var_z;
    Q(5,5) = acc_corr * accel_var_z;

    // 偏航角 (索引6) 和偏航角速度 (索引7)
    Q(6,6) = pos_corr * ang_accel_var;
    Q(6,7) = vel_corr * ang_accel_var;
    Q(7,6) = vel_corr * ang_accel_var;
    Q(7,7) = acc_corr * ang_accel_var;

    // 静态形状参数的过程噪声（小方差，缓慢变化）
    Q(8,8)   = 1e-5;   // 基准半径 r
    Q(9,9)   = 1e-5;   // 侧板半径增量 l
    Q(10,10) = 1e-5;   // 高度差 h

    // 3. 状态预测（F×x），并归一化偏航角
    Eigen::Matrix<double, 11, 1> x_pred = F * x;
    x_pred(6) = normalizeYaw(x_pred(6));

    // 4. 更新状态和协方差
    x = x_pred;
    P = F * P * F.transpose() + Q;
}

/**
 * @brief EKF更新步骤，融合观测值修正状态
 * @param z_obs 观测向量 [yaw_cam, pitch_cam, dist, yaw_armor]
 *        yaw_cam：相机坐标系下偏航角；pitch_cam：相机坐标系下俯仰角
 *        dist：装甲板到相机距离；yaw_armor：装甲板偏航角
 * @param armor_xyz 装甲板在世界坐标系的位置（米）
 */
void ArmorEKF::update(const Eigen::Vector4d& z_obs) {
    // 步骤1：装甲板ID匹配（找到最优匹配的装甲板编号）
    int best_id = 0;
    double min_error = 1e10;  // 最小误差初始值
    for (int i = 0; i < armor_num_; ++i) {
        // 计算当前ID对应的装甲板偏航角（归一化）
        double plate_yaw = normalizeYaw(x(6) + i * 2.0 * M_PI / armor_num_);
        // 预测当前ID装甲板的位置
        Eigen::Vector3d pred_xyz = hArmorXyz(x, i);
        // 转换为球坐标（yaw/pitch/dist）
        Eigen::Vector3d pred_ypd = xyz2Ypd(pred_xyz);
        // 计算观测值与预测值的误差（角度误差）
        double err = std::abs(normalizeYaw(z_obs(3) - plate_yaw))
                   + std::abs(normalizeYaw(z_obs(0) - pred_ypd(0)));
        // 选取误差最小的ID
        if (err < min_error) {
            min_error = err;
            best_id = i;
        }
    }
    last_id_ = best_id;    // 记录最优ID
    update_count_++;       // 累计更新次数

    // 步骤2：构建观测噪声协方差矩阵R（动态调整）
    double center_yaw = z_obs(0);// 观测的相机坐标系下偏航角                             
    double distance = z_obs(2);                             // 观测的装甲板距离
    double delta_angle = normalizeYaw(z_obs(3) - center_yaw);// 角度差（观测装甲板偏航角 - 预测装甲板偏航角）

    Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
    R(0,0) = 2e-3;   // yaw_cam 噪声方差
    R(1,1) = 2e-3;   // pitch_cam 噪声方差
    R(2,2) = std::log(std::abs(delta_angle) + 1.5) + 1.0;   // dist 噪声方差（随角度差增大而增大）
    R(3,3) = std::log(distance + 1.0) / 200.0 + 9e-2;       // yaw_armor 噪声方差（随距离增大而增大）

    // 步骤3：计算观测模型的雅可比矩阵H
    Eigen::Matrix<double, 4, 11> H = hJacobian(x, best_id);

    // 步骤4：计算观测残差（观测值-预测值）
    Eigen::Vector4d z_pred = hFunc(x, best_id);          // 预测观测值
    Eigen::Vector4d residual = z_obs - z_pred;           // 残差
    // 角度残差归一化（-π ~ π）
    residual(0) = normalizeYaw(residual(0));
    residual(1) = normalizeYaw(residual(1));
    residual(3) = normalizeYaw(residual(3));

    // 步骤5：EKF核心更新公式
    Eigen::Matrix4d S = H * P * H.transpose() + R;       // 创新协方差
    Eigen::Matrix<double, 11, 4> K = P * H.transpose() * S.inverse(); // 卡尔曼增益
    Eigen::Matrix<double, 11, 1> dx = K * residual;      // 状态修正量
    x += dx;                                             // 状态修正
    x(6) = normalizeYaw(x(6));                           // 偏航角归一化

   Eigen::Matrix<double, 11, 11> I = Eigen::Matrix<double, 11, 11>::Identity();
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose(); // Joseph形式更新协方差，增强数值稳定性

    // 步骤6：收敛判断（更新次数>5，且半径在合理范围）
    if (update_count_ > 5 && x(8) > 0.05 && x(8) < 0.5)
        converged_ = true;
}

/**
 * @brief 获取指定ID装甲板的中心坐标（世界坐标系）
 * @param id 装甲板ID（0-3）
 * @return 装甲板中心坐标（米）
 */
Eigen::Vector3d ArmorEKF::getArmorCenter(int id) const {
    return hArmorXyz(x, id);
}

/**
 * @brief 获取上一帧匹配装甲板的中心坐标（世界坐标系）
 * @return 装甲板中心坐标（米）
 */
Eigen::Vector3d ArmorEKF::getArmorCenter() const {
    return hArmorXyz(x, last_id_);
}

// ==================== 私有成员函数实现 ====================

/**
 * @brief 观测模型：根据状态和装甲板ID计算装甲板中心坐标
 * @param state EKF状态向量
 * @param id 装甲板ID
 * @return 装甲板中心坐标（世界坐标系，米）
 */
Eigen::Vector3d ArmorEKF::hArmorXyz(const Eigen::Matrix<double, 11, 1>& state, int id) const {
    // 计算当前装甲板的偏航角（机器人偏航角+ID偏移，归一化）
    double plate_yaw = normalizeYaw(state(6) + id * 2.0 * M_PI / armor_num_);
    // 判断是否为侧板装甲板（4装甲板时，1/3为侧板）
    bool is_side = (armor_num_ == 4) && (id == 1 || id == 3);
    
    // 计算装甲板半径（侧板=基准半径+增量l）
    double radius;
    if (is_side) {
        radius = state(8) + state(9);
    } else {
        radius = state(8);
    }
    
    // 计算装甲板高度（侧板=基准高度+偏移h）
    double height;
    if (is_side) {
        height = state(4) + state(10);
    } else {
        height = state(4);
    }
    
    // 装甲板中心坐标（机器人中心 - 半径×方向向量）
    double armor_x = state(0) - radius * std::cos(plate_yaw);
    double armor_y = state(2) - radius * std::sin(plate_yaw);
    return {armor_x, armor_y, height};
}

/**
 * @brief 观测模型：根据状态和装甲板ID计算预测观测值
 * @param state EKF状态向量
 * @param id 装甲板ID
 * @return 预测观测向量 [yaw_cam, pitch_cam, dist, yaw_armor]
 */
Eigen::Vector4d ArmorEKF::hFunc(const Eigen::Matrix<double, 11, 1>& state, int id) const {
    // 计算装甲板中心坐标
    Eigen::Vector3d armor_xyz = hArmorXyz(state, id);
    // 转换为球坐标（yaw/pitch/dist）
    Eigen::Vector3d ypd = xyz2Ypd(armor_xyz);
    // 计算装甲板偏航角
    double plate_yaw = normalizeYaw(state(6) + id * 2.0 * M_PI / armor_num_);
    // 组合预测观测向量
    return {ypd(0), ypd(1), ypd(2), plate_yaw};
}

/**
 * @brief 计算观测模型的雅可比矩阵H（4x11）
 * @param state EKF状态向量
 * @param id 装甲板ID
 * @return 雅可比矩阵H
 */
Eigen::Matrix<double, 4, 11> ArmorEKF::hJacobian(const Eigen::Matrix<double, 11, 1>& state, int id) const {
    // 装甲板偏航角（归一化）
    double plate_yaw = normalizeYaw(state(6) + id * 2.0 * M_PI / armor_num_);
    // 判断是否为侧板装甲板
    bool is_side = (armor_num_ == 4) && (id == 1 || id == 3);
    // 装甲板半径
    double radius = is_side ;
    if (is_side) {
        radius = state(8) + state(9);
    } else {
        radius = state(8);
    }

    // 构建装甲板坐标对状态的雅可比矩阵（3x11）
    Eigen::Matrix<double, 3, 11> H_xyz = Eigen::Matrix<double, 3, 11>::Zero();

    // 对cx的偏导数
    H_xyz(0,0) = 1.0;
    // 对yaw的偏导数（解决旋转偏心问题）
    H_xyz(0,6) =  radius * std::sin(plate_yaw);
    // 对r的偏导数
    H_xyz(0,8) = -std::cos(plate_yaw);
    // 侧板时，对l的偏导数
    if (is_side) H_xyz(0,9) = -std::cos(plate_yaw);

    // 对cy的偏导数
    H_xyz(1,2) = 1.0;
    // 对yaw的偏导数
    H_xyz(1,6) = -radius * std::cos(plate_yaw);
    // 对r的偏导数
    H_xyz(1,8) = -std::sin(plate_yaw);
    // 侧板时，对l的偏导数
    if (is_side) H_xyz(1,9) = -std::sin(plate_yaw);

    // 对cz的偏导数
    H_xyz(2,4) = 1.0;
    // 侧板时，对h的偏导数
    if (is_side) H_xyz(2,10) = 1.0;

    // 计算球坐标转换的雅可比矩阵（3x3）
    Eigen::Vector3d armor_xyz = hArmorXyz(state, id);
    Eigen::Matrix<double, 3, 3> J_ypd = xyz2YpdJacobian(armor_xyz);

    // 组合最终雅可比矩阵（4x11）
    Eigen::Matrix<double, 4, 11> H = Eigen::Matrix<double, 4, 11>::Zero();
    // 前3行：球坐标雅可比×装甲板坐标雅可比
    H.block<3,11>(0,0) = J_ypd * H_xyz;
    // 第4行：装甲板偏航角直接对应状态的yaw
    H(3,6) = 1.0;
    return H;
}


// ==================== 静态工具函数 ====================

/**
 * @brief 角度归一化（限制在-π ~ π）
 * @param angle 输入角度（弧度）
 * @return 归一化后的角度
 */
double ArmorEKF::normalizeYaw(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 笛卡尔坐标→球坐标转换（yaw/pitch/dist）
 * @param xyz 笛卡尔坐标（x/y/z，米）
 * @return 球坐标 [yaw, pitch, dist]（弧度/弧度/米）
 */
Eigen::Vector3d ArmorEKF::xyz2Ypd(const Eigen::Vector3d& xyz) {
    double yaw   = std::atan2(xyz(1), xyz(0));                 // 水平偏航角
    double pitch = std::atan2(xyz(2), std::sqrt(xyz(0)*xyz(0) + xyz(1)*xyz(1))); // 俯仰角
    double dist  = xyz.norm();                                 // 直线距离
    return {yaw, pitch, dist};
}

/**
 * @brief 笛卡尔坐标→球坐标的雅可比矩阵（3x3）
 * @param xyz 笛卡尔坐标（x/y/z，米）
 * @return 3x3雅可比矩阵
 */
Eigen::Matrix<double, 3, 3> ArmorEKF::xyz2YpdJacobian(const Eigen::Vector3d& xyz) {
    double x = xyz(0), y = xyz(1), z = xyz(2);
    double xy2 = x*x + y*y;      // 水平距离平方
    double xy = std::sqrt(xy2);  // 水平距离
    double r = std::sqrt(xy2 + z*z);   // 直线距离
    double r2 = r*r;             // 直线距离平方

    Eigen::Matrix<double, 3, 3> J;
    // yaw 的偏导数
    J(0,0) = -y / xy2;
    J(0,1) =  x / xy2;
    J(0,2) =  0.0;
    // pitch 的偏导数
    J(1,0) = -x * z / (r2 * xy);
    J(1,1) = -y * z / (r2 * xy);
    J(1,2) =  xy / r2;
    // distance 的偏导数
    J(2,0) = x / r;
    J(2,1) = y / r;
    J(2,2) = z / r;
    return J;
}