#ifndef CPP08_ARMOR_DETECTOR_TRAJECTORY_SOLVER_HPP_
#define CPP08_ARMOR_DETECTOR_TRAJECTORY_SOLVER_HPP_

#include <cmath>

// 注意这里：绝对不要加 namespace 包装！

class TrajectorySolver {
public:
    TrajectorySolver();
    ~TrajectorySolver() = default;

    /**
     * @brief 基于单向空气阻力模型的前向迭代弹道求解器
     * @param x 目标水平距离 (单位: m)
     * @param target_y 目标高度 (即文档中的目标点 y 坐标，单位: m)
     * @param v0 枪口初速 (单位: m/s)
     * @return 补偿后的仰角 (单位: rad)
     */
    double solveTrajectoryPitch(double x, double target_y, double v0);

    // 允许外部动态修改参数
    void setK1(double k1) { k1_ = k1; }
    void setG(double g) { g_ = g; }

private:
    double g_;  // 重力加速度
    double k1_; // 空气阻力系数
};

#endif // CPP08_ARMOR_DETECTOR_TRAJECTORY_SOLVER_HPP_