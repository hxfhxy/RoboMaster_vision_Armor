#include "cpp08_armor_detector/armor_detector.hpp"

// 2. 绘制Yaw曲线函数（调试用）
void ArmorDetector::drawYawPlot()
{
    cv::Mat plot = cv::Mat::zeros(400, 800, CV_8UC3);
    cv::line(plot, cv::Point(0, 200), cv::Point(800, 200), cv::Scalar(100, 100, 100), 1);
    float scale = 20;
    for (size_t i = 1; i < filteredYawList.size(); i++)
    {
        int x = i * 2;
        if (x >= 800)
            break;
        cv::circle(plot, cv::Point(x, 200 - rawYawList[i] * scale), 1, cv::Scalar(0, 0, 255), -1);
        cv::line(plot, cv::Point((i - 1) * 2, 200 - filteredYawList[i - 1] * scale),
                cv::Point(x, 200 - filteredYawList[i] * scale), cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("Yaw Filter Plot", plot);
}