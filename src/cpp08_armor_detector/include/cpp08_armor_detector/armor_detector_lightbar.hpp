#ifndef ARMOR_DETECTOR_LIGHTBAR_HPP
#define ARMOR_DETECTOR_LIGHTBAR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 灯条检测和处理相关函数
 */

/**
 * @brief 将四个顶点按顺时针顺序排序
 * @param corners 输入/输出的四个点
 */
void sortCornersClockwise(std::vector<cv::Point2f>& corners);

/**
 * @brief 获取灯条的上下端点
 * @param bar 旋转矩形灯条
 * @param out_top 输出的上端点
 * @param out_bottom 输出的下端点
 */
void getBarEndpoints(const cv::RotatedRect& bar, cv::Point2f& out_top, cv::Point2f& out_bottom);

/**
 * @brief 强制修正灯条为标准形式：height为长边，width为短边
 * @param rect 原始旋转矩形
 * @return 修正后的旋转矩形
 */
cv::RotatedRect fixLightBarSize(cv::RotatedRect rect);

/**
 * @brief 从二值图像中提取灯条
 * @param mask 预处理后的二值图像掩码
 * @param min_area 灯条最小面积阈值
 * @return 检测到的灯条列表
 */
std::vector<cv::RotatedRect> extractLightBars(const cv::Mat& mask, int min_area = 10);

#endif // ARMOR_DETECTOR_LIGHTBAR_HPP
