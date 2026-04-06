#ifndef ARMOR_DETECTOR_MATCHING_HPP
#define ARMOR_DETECTOR_MATCHING_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

/**
 * @brief 检测到的装甲板结构体
 */
struct DetectedArmor {
    cv::RotatedRect rect;                        // 装甲板旋转矩形
    std::vector<cv::Point2f> pnp_corners;       // PnP角点（用于3D解算）
    double yaw;                                  // 偏航角（度）
    double pitch;                                // 俯仰角（度）
    double distance;                             // 装甲板距离
    double dist_to_center;                       // 到图像中心的距离
    double confidence;                           // 置信度分数
    std::pair<int, int> light_bar_ids;          // 配对的灯条索引 (i, j)
    cv::Mat tvec;
    cv::Mat rvec; 
};

/**
 * @brief 进行灯条配对，找出可能的装甲板
 * @param lightBars 输入的灯条列表
 * @param imgSize 图像大小
 * @return 配对后的装甲板列表（包含未去重的所有候选）
 */
std::vector<DetectedArmor> matchLightBars(const std::vector<cv::RotatedRect>& lightBars, 
                                         const cv::Size& imgSize);

/**
 * @brief 对检测到的装甲板进行去重：每个灯条只能属于一个置信度最高的装甲板
 * @param armors 输入的装甲板列表
 * @return 去重后的装甲板列表
 */
std::vector<DetectedArmor> deduplicateArmors(const std::vector<DetectedArmor>& armors);

/**
 * @brief 使用PnP解算计算装甲板的yaw、pitch和distance
 * @param armor 待解算的装甲板
 * @param cameraMatrix 相机内参矩阵
 * @param distCoeffs 相机畸变系数
 * @param objectPoints 装甲板3D物理坐标
 */
void calculatePnP(DetectedArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Point3f>& objectPoints);

#endif // ARMOR_DETECTOR_MATCHING_HPP
