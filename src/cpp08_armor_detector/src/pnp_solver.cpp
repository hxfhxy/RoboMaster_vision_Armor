#include "cpp08_armor_detector/armor_detector_matching.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include <set>
#include <algorithm>

/**
 * @brief PnP解算：严格使用 IPPE 算法和 FLU 零点模型，并引入双解验证魔法
 */
void calculatePnP(DetectedArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Point3f>& /*objectPoints*/) {
    // 顺序：0=左上(LT), 1=左下(LB), 2=右下(RB), 3=右上(RT)
    std::vector<cv::Point2f> image_points = armor.pnp_corners;

    // 大小装甲板判断
    float pixel_width = cv::norm(armor.pnp_corners[0] - armor.pnp_corners[3]);
    float pixel_height = cv::norm(armor.pnp_corners[0] - armor.pnp_corners[1]);
    bool is_small = (pixel_width / pixel_height) < 3.2f;

    double half_w = is_small ? (135.0 / 2.0) : (230.0 / 2.0);
    double half_h = 55.0 / 2.0;

    double tilt = 15.0 * CV_PI / 180.0; // 真实的装甲板后倾角

    // 计算倾斜后的 Y 和 Z 坐标投影
    double z_top = half_h * std::sin(tilt);
    double z_bottom = -half_h * std::sin(tilt);
    double y_top = -half_h * std::cos(tilt);
    double y_bottom = half_h * std::cos(tilt);

    std::vector<cv::Point3f> object_points_cv = {
        cv::Point3f(-half_w, y_top, z_top),       // 0: 左上
        cv::Point3f(-half_w, y_bottom, z_bottom), // 1: 左下
        cv::Point3f( half_w, y_bottom, z_bottom), // 2: 右下
        cv::Point3f( half_w, y_top, z_top)        // 3: 右上v       
    };

    // IPPE算法解算PnP，返回两个解
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;
    int solutions = cv::solvePnPGeneric(
        object_points_cv, image_points, cameraMatrix, distCoeffs,
        rvecs, tvecs, false, cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), errors);

    if (solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        armor.distance = 0; armor.yaw = 0; armor.pitch = 0; 
        return;
    }

    // 双解验证
    int best = 0;
    if (rvecs.size() > 1 && tvecs.size() > 1) {
        auto yaw_from_rvec = [](cv::Mat &rv) {
            cv::Mat R;
            cv::Rodrigues(rv, R);
            return std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * 180.0 / CV_PI;
        };

        double yaw0 = yaw_from_rvec(rvecs[0]);
        double yaw1 = yaw_from_rvec(rvecs[1]);
        
        // 提取2D图像倾斜趋势
        cv::Point2f left_axis = armor.pnp_corners[0] - armor.pnp_corners[1]; // 左侧灯条向量
        cv::Point2f right_axis = armor.pnp_corners[3] - armor.pnp_corners[2];// 右侧灯条向量
        double armor_angle_trend = (std::atan2(left_axis.x, left_axis.y) + 
                                    std::atan2(right_axis.x, right_axis.y));
        
        // 选择符合物理事实的解
        if ((armor_angle_trend > 0 && yaw1 > 0 && yaw0 < 0) || 
            (armor_angle_trend < 0 && yaw1 < 0 && yaw0 > 0)) {
            best = 1;
        }
    }

    // 位姿提取
    armor.tvec = tvecs[best].clone(); 
    armor.rvec = rvecs[best].clone();
    
    double x = armor.tvec.at<double>(0, 0);
    double y = armor.tvec.at<double>(1, 0);
    double z = armor.tvec.at<double>(2, 0);
    
    armor.distance = std::sqrt(x*x + y*y + z*z); 

    // 计算yaw角
    cv::Mat R_best;
    cv::Rodrigues(armor.rvec, R_best);
    cv::Vec3d normal(R_best.at<double>(0, 2), R_best.at<double>(1, 2), R_best.at<double>(2, 2));
    cv::Vec3d pos(x, y, z);
    if (normal.dot(pos) > 0.0) {
        normal = -normal;
    }
    armor.yaw = std::atan2(normal[0], normal[2]) * 180.0 / CV_PI;

    // 计算pitch角
    armor.pitch = std::atan2(-y, z) * 180.0 / CV_PI;
}