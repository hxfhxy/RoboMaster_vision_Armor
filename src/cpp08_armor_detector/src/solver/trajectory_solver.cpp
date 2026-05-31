#include "cpp08_armor_detector/solver/trajectory_solver.hpp"

// 注意这里：同样绝对不要加 namespace 包装！

TrajectorySolver::TrajectorySolver() : g_(9.80665), k1_(0.1) {
    // k1 默认初始化为文档中的 0.1
}

double TrajectorySolver::solveTrajectoryPitch(double x, double target_y, double v0) {
    double temp_y = target_y; 
    double angle = 0.0;
    
    // 循环迭代20次，保证收敛精度 (误差小于1mm)
    for (int i = 0; i < 20; ++i) {
        // 计算仰角 angle = 枪管指向 tempPoint 的角度
        angle = std::atan2(temp_y, x);
        
        // 速度分解
        double v_x0 = v0 * std::cos(angle);
        double v_y0 = v0 * std::sin(angle);
        
        // 防止除零异常
        if (v_x0 < 1e-5) break; 
        
        // 利用水平方向位移模型反解飞行时间 t
        double t = (std::exp(k1_ * x) - 1.0) / (k1_ * v_x0);
        
        // 计算实际命中点 realPoint (仅受重力影响)
        double real_y = v_y0 * t - 0.5 * g_ * t * t;
        
        // 得到误差，即下落高度 deltaH
        double delta_h = target_y - real_y;
        
        // 更新临时目标点
        temp_y = temp_y + delta_h;
        
        // 如果高度误差小于 1mm，提前退出
        if (std::abs(delta_h) < 0.001) {
            break;
        }
    }
    
    return angle;
}