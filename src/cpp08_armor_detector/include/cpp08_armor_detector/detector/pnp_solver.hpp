#ifndef ARMOR_DETECTOR_PNP_SOLVER_HPP
#define ARMOR_DETECTOR_PNP_SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

struct DetectedArmor;

/**
 * @brief 使用PnP解算计算装甲板的yaw、pitch和distance
 * @param armor 待解算的装甲板
 * @param cameraMatrix 相机内参矩阵
 * @param distCoeffs 相机畸变系数
 * @param objectPoints 装甲板3D物理坐标
 */
void calculatePnP(DetectedArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Point3f>& objectPoints);

#endif // ARMOR_DETECTOR_PNP_SOLVER_HPP

