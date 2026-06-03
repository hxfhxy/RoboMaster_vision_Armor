#include "cpp08_armor_detector/detector/armor_detector.hpp"

/**
 * @brief 更新装甲板跟踪状态（检测到装甲板时调用）
 * @param z_obs EKF观测向量 [yaw_cam, pitch_cam, dist, yaw_armor]
 * @param armor_world 检测到的装甲板世界坐标(米)
 * @param timestamp 当前帧时间戳(秒)
 * @param smoothed_world 输出：EKF平滑后的装甲板世界坐标(米)
 * @return bool 跟踪是否成功
 */
bool ArmorDetector::updateTracking(const Eigen::Vector4d& z_obs,
                                   const Eigen::Vector3d& armor_world,
                                   double timestamp,
                                   Eigen::Vector3d& smoothed_world) {
  // 无效时间戳直接返回失败
  if (timestamp <= 0.0) {
    return false;
  }

  // EKF未初始化，执行首次初始化
  if (!ekf_initialized) {
    // 初始协方差矩阵对角元素
    // [cx, vx, cy, vy, cz, vz, yaw, vyaw, r, l, h]
    // 位置/速度/角度初始不确定性大，几何参数(r/l/h)初始不确定性小
    Eigen::Matrix<double, 11, 1> initP;
    initP << 100.0, 100.0, 100.0, // x, y, z 的初始不确定度
             10.0,                // yaw 的初始不确定度
             100.0, 100.0, 100.0, // vx, vy, vz 速度的不确定度
             10.0,                // vyaw 角速度不确定度
             0.01, 0.01, 0.01;    // r, l, h 几何参数的不确定度
    
    // 初始化EKF：初始半径260mm，侧板增量和高度偏移初始为0
    ekf.init(armor_world, z_obs(3), 4, 0.26, 0.0, 0.0, initP);
    prev_timestamp_ = timestamp;
    ekf_initialized = true;
    lost_count_ = 0;
    tracker_state_ = TrackerState::TRACKING;
    found = true;
    // 返回平滑后的装甲板坐标
    smoothed_world = ekf.getArmorCenter();
    return true;
  }

  // 计算帧间隔时间差
  double dt = timestamp - prev_timestamp_;
  // 时间差异常时使用默认33ms(30fps)，防止预测跳变
  if (dt > 0.1 || dt <= 0.0) {
    dt = 0.033;
  }
  prev_timestamp_ = timestamp;

  // EKF预测步：基于匀速模型递推状态
  ekf.predict(dt);
  // EKF更新步：融合当前观测值修正状态
  ekf.update(z_obs);
  
  // 检测到目标，重置丢失计数
  lost_count_ = 0;
  tracker_state_ = TrackerState::TRACKING;
  found = true;
  // 返回平滑后的装甲板坐标
  smoothed_world = ekf.getArmorCenter();
  return true;
}

/**
 * @brief 预测装甲板跟踪状态（未检测到装甲板时调用）
 * @param timestamp 当前帧时间戳(秒)
 * @param smoothed_world 输出：EKF预测的装甲板世界坐标(米)
 * @return bool 预测是否有效（丢失超过30帧返回false）
 */
bool ArmorDetector::predictTracking(double timestamp, Eigen::Vector3d& smoothed_world) {
  // EKF未初始化，无法预测
  if (!ekf_initialized) {
    return false;
  }

  // 计算帧间隔时间差
  double dt = timestamp - prev_timestamp_;
  // 时间差异常时使用默认33ms
  if (dt > 0.1 || dt <= 0.0) {
    dt = 0.033;
  }
  prev_timestamp_ = timestamp;

  // 仅执行EKF预测步，无观测更新
  ekf.predict(dt);
  
  // 丢失计数加1
  lost_count_++;
  tracker_state_ = TrackerState::LOST;
  
  // 丢失超过30帧(约1秒)，重置跟踪器
  if (lost_count_ > 30) {
    found = false;
    ekf_initialized = false;
    lost_count_ = 0;
    tracker_state_ = TrackerState::DETECTING;
    center_fits.clear();  // 清空历史中心数据
    return false;
  }

  // 返回预测的装甲板坐标
  smoothed_world = ekf.getArmorCenter();
  return true;
}
