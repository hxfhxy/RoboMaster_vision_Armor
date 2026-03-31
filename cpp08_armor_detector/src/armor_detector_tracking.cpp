#include "cpp08_armor_detector/armor_detector.hpp"

// 3. 机器人中心估计：从历史装甲板拟合结果使用最小二乘求交点
void ArmorDetector::find_robot_center()
{
    // 无目标则跳过
    if (!found || center_fits.size() < 2)
        return;

    cv::Matx33d A = cv::Matx33d::zeros();
    cv::Vec3d b(0.0, 0.0, 0.0);
    cv::Matx33d I = cv::Matx33d::eye();

    for (const auto &f : center_fits)
    {
        cv::Vec3d p(f.position.x, f.position.y, f.position.z);
        cv::Vec3d n(f.normalvector.x, f.normalvector.y, f.normalvector.z);
        cv::Matx33d nnT(
            n[0] * n[0], n[0] * n[1], n[0] * n[2],
            n[1] * n[0], n[1] * n[1], n[1] * n[2],
            n[2] * n[0], n[2] * n[1], n[2] * n[2]);
        cv::Matx33d P = I - nnT;
        A += P;
        b += P * p;
    }

    // 求解最小二乘中心点
    cv::Vec3d center = A.inv(cv::DECOMP_SVD) * b;

    // 反投影到图像平面
    center_3d.clear();
    center_3d.push_back(cv::Point3f((float)center[0], (float)center[1], (float)center[2]));

    cv::Mat zero_rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat zero_tvec = cv::Mat::zeros(3, 1, CV_64F);
    center_2d.clear();
    cv::projectPoints(center_3d, zero_rvec, zero_tvec, cameraMatrix, distCoeffs, center_2d);

    if (center_2d.empty())
        return;

    cv::Point2f now_point = center_2d[0];
    cv::Point2f final_point;

    // 低通滤波中心点，避免抖动
    find_center = true;
    if (first_frame)
    {
        final_point = now_point;
        first_frame = false;
    }
    else
    {
        final_point.x = 0.6f * now_point.x + 0.4f * last_center_2d.x;
        final_point.y = 0.6f * now_point.y + 0.4f * last_center_2d.y;
    }

    last_center_2d = final_point;
    center_2d.clear();
    center_2d.push_back(final_point);
}