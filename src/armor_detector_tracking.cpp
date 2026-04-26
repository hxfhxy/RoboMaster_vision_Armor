#include "cpp08_armor_detector/armor_detector.hpp"

// 3. 机器人中心估计：从历史装甲板拟合结果使用最小二乘求交点
void ArmorDetector::find_robot_center()
{
    // 无目标或历史数据不足，直接跳过
    if (!found || center_fits.size() < 1)
        return;

    // 初始化最小二乘所需的矩阵A和向量b
    cv::Matx33d A = cv::Matx33d::zeros(); // 3x3系数矩阵
    cv::Vec3d b(0.0, 0.0, 0.0);           // 3x1右侧向量
    cv::Matx33d I = cv::Matx33d::eye();   // 3x3单位矩阵

    // 遍历历史装甲板拟合数据，累加构建A和b
    for (const auto &f : center_fits)
    {
        // 当前装甲板的3D位置（相机坐标系）
        cv::Vec3d p(f.position.x, f.position.y, f.position.z);
        // 当前装甲板的法向量（指向机器人中心方向）
        cv::Vec3d n(f.normalvector.x, f.normalvector.y, f.normalvector.z);
        
        // 计算法向量的外积矩阵 nn^T
        cv::Matx33d nnT(
            n[0] * n[0], n[0] * n[1], n[0] * n[2],
            n[1] * n[0], n[1] * n[1], n[1] * n[2],
            n[2] * n[0], n[2] * n[1], n[2] * n[2]);
        
        // 计算投影矩阵 P = I - nn^T：将点投影到法向量的垂直平面上
        cv::Matx33d P = I - nnT;
        
        // 累加A和b：A += P, b += P*p
        A += P;
        b += P * p;
    }

    // 求解最小二乘问题 A*center = b，得到机器人3D中心
    cv::Vec3d center = A.inv(cv::DECOMP_SVD) * b; // 使用SVD分解求逆，数值更稳定

    // 将3D中心存入列表，准备反投影
    center_3d.clear();
    center_3d.push_back(cv::Point3f((float)center[0], (float)center[1], (float)center[2]));

    // 初始化零旋转向量和零平移向量（因为是在相机坐标系下直接投影）
    cv::Mat zero_rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat zero_tvec = cv::Mat::zeros(3, 1, CV_64F);
    
    // 将3D中心反投影到2D图像平面
    center_2d.clear();
    cv::projectPoints(center_3d, zero_rvec, zero_tvec, cameraMatrix, distCoeffs, center_2d);

    // 反投影失败则跳过
    if (center_2d.empty())
        return;

    // 获取当前反投影的2D中心点
    cv::Point2f now_point = center_2d[0];
    cv::Point2f final_point;

    // 低通滤波中心点，避免抖动
    find_center = true;
    if (first_frame)
    {
        // 第一帧：直接赋值，初始化
        final_point = now_point;
        first_frame = false;
    }
    else
    {
        // 后续帧：加权平均滤波（0.6当前帧 + 0.4上一帧）
        final_point.x = 0.6f * now_point.x + 0.4f * last_center_2d.x;
        final_point.y = 0.6f * now_point.y + 0.4f * last_center_2d.y;
    }

    // 更新上一帧的滤波结果，并保存最终的2D中心点
    last_center_2d = final_point;
    center_2d.clear();
    center_2d.push_back(final_point);
}
