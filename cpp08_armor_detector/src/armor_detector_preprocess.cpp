#include "cpp08_armor_detector/armor_detector.hpp"

cv::Mat ArmorDetector::preprocess(cv::Mat img)
{
    const int IMAGE_BRIGHT = 30;       // 全局亮度增益（过暗就加大）
    const int THRESHOLD_VALUE = 200;   // 二值化阈值（噪点多就加大）
    const int KERNEL_SIZE = 2;          // 核大小

    cv::Mat dst_BR, dst;
    std::vector<cv::Mat> channels;

    // 全局亮度调整
    {
        cv::Mat BrightnessLut(1, 256, CV_8UC1); 
        for (int i = 0; i < 256; i++) {
            BrightnessLut.at<uchar>(i) = cv::saturate_cast<uchar>(i + IMAGE_BRIGHT);
        }
        cv::LUT(img, BrightnessLut, dst_BR);
    }

    // 颜色通道差分
    cv::split(dst_BR, channels);

    // 二值化
    cv::threshold(channels[0], dst, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

    // 垂直方向模糊
    cv::blur(dst, dst, cv::Size(1, 3)); // 只模糊垂直方向，保留灯条形状


    // 形态学膨胀
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE, KERNEL_SIZE));
    // cv::dilate(dst, dst, kernel, cv::Point(-1, -1), 1);
    //cv::erode(dst, dst, kernel, cv::Point(-1, -1), 1);
    //cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    //调试显示（
    cv::imshow("最终掩码", dst);
    cv::waitKey(1);

    return dst;
}