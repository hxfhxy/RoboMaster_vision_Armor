#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

/**
 * @brief 装甲板扩展卡尔曼滤波器类（基于Eigen的全耦合EKF架构）
 * 状态向量维度11：[cx, vx, cy, vy, cz, vz, yaw, vyaw, r, l, h]
 *  - cx/cy/cz: 机器人中心坐标 (m)
 *  - vx/vy/vz: 机器人速度 (m/s)
 *  - yaw: 机器人偏航角 (rad)
 *  - vyaw: 机器人偏航角速度 (rad/s)
 *  - r: 基准装甲板半径 (m)
 *  - l: 侧板装甲板半径增量 (m)
 *  - h: 侧板装甲板高度偏移 (m)
 */
class ArmorEKF {
public:
    /**
     * @brief 构造函数：初始化默认参数
     */
    ArmorEKF();

    /**
     * @brief 初始化EKF状态和协方差
     * @param p_armor_world 装甲板世界坐标系初始位置 (m)
     * @param orientation_yaw 装甲板初始偏航角 (rad)
     * @param armor_num 装甲板数量（默认4）
     * @param init_r 基准装甲板初始半径 (m)
     * @param init_l 侧板装甲板初始半径增量 (m)
     * @param init_h 侧板装甲板初始高度偏移 (m)
     * @param P0_diag 初始协方差矩阵的对角元素
     */
    void init(const Eigen::Vector3d& p_armor_world,
              double orientation_yaw,
              int armor_num,
              double init_r,
              double init_l,
              double init_h,
              const Eigen::Matrix<double, 11, 1>& P0_diag);

    /**
     * @brief EKF预测步骤：基于匀速运动模型更新状态和协方差
     * @param dt 时间差 (s)
     */
    void predict(double dt);

    /**
     * @brief EKF更新步骤：融合观测值修正状态
     * @param z_obs 观测向量 [yaw_cam, pitch_cam, dist, yaw_armor]
     * @param armor_xyz 装甲板世界坐标系位置 (m)
     */
    void update(const Eigen::Vector4d& z_obs);

    /**
     * @brief 获取指定ID装甲板的中心坐标（世界坐标系）
     * @param id 装甲板ID（0~3）
     * @return 装甲板中心坐标 (m)
     */
    Eigen::Vector3d getArmorCenter(int id) const;
    
    /**
     * @brief 获取上一帧匹配装甲板的中心坐标（世界坐标系）
     * @return 装甲板中心坐标 (m)
     */
    Eigen::Vector3d getArmorCenter() const;

    // 公有成员变量（状态与协方差）
    Eigen::Matrix<double, 11, 1> x;          // 状态向量
    Eigen::Matrix<double, 11, 11> P;         // 协方差矩阵
    int armor_num_ = 4;                      // 默认4装甲板
    int last_id_ = 0;                        // 上一帧匹配的装甲板ID    
    int update_count_ = 0;                   // 更新次数
    bool converged_ = false;                 // 收敛标志

private:
    /**
     * @brief 观测模型：根据状态和装甲板ID计算装甲板中心坐标
     * @param state EKF状态向量
     * @param id 装甲板ID
     * @return 装甲板中心坐标（世界坐标系，m）
     */
    Eigen::Vector3d hArmorXyz(const Eigen::Matrix<double, 11, 1>& state, int id) const;

    /**
     * @brief 观测模型：根据状态和装甲板ID计算预测观测值
     * @param state EKF状态向量
     * @param id 装甲板ID
     * @return 预测观测向量 [yaw_cam, pitch_cam, dist, yaw_armor]
     */
    Eigen::Vector4d hFunc(const Eigen::Matrix<double, 11, 1>& state, int id) const;

    /**
     * @brief 计算观测模型的雅可比矩阵H（4x11）
     * @param state EKF状态向量
     * @param id 装甲板ID
     * @return 雅可比矩阵H
     */
    Eigen::Matrix<double, 4, 11> hJacobian(const Eigen::Matrix<double, 11, 1>& state, int id) const;

    // 静态工具函数
    static double normalizeYaw(double angle);                // 角度归一化（-π ~ π）
    static Eigen::Vector3d xyz2Ypd(const Eigen::Vector3d& xyz);  // 笛卡尔→球坐标转换
    static Eigen::Matrix<double, 3, 3> xyz2YpdJacobian(const Eigen::Vector3d& xyz);  // 球坐标转换雅可比矩阵
};

#endif // KALMAN_FILTER_HPP