#include "cpp08_armor_detector/armor_detector.hpp"

cv::Mat ArmorDetector::preprocess(cv::Mat img)
{
    const int IMAGE_BRIGHT = 30;       // 全局亮度增益
    const int THRESHOLD_VALUE = 200;   // BGR二值化阈值
    const int MIN_SATURATION = 150;    // 最小饱和度阈值 (0-255)，低于这个值的直接过滤

    cv::Mat dst_BR, dst;
    std::vector<cv::Mat> channels;

    // 1. 全局亮度调整
    {
        cv::Mat BrightnessLut(1, 256, CV_8UC1); 
        for (int i = 0; i < 256; i++) {
            BrightnessLut.at<uchar>(i) = cv::saturate_cast<uchar>(i + IMAGE_BRIGHT);
        }
        cv::LUT(img, BrightnessLut, dst_BR);
    }

    // 计算饱和度掩码
    cv::Mat hsv, mask_saturation;
    cv::cvtColor(dst_BR, hsv, cv::COLOR_BGR2HSV); // 把亮度调整后的图转成HSV
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    cv::Mat S = hsv_channels[1]; // 提取饱和度通道
    
    // 对饱和度进行二值化：只有饱和度够高的地方才是白色
    cv::threshold(S, mask_saturation, MIN_SATURATION, 255, cv::THRESH_BINARY);

    // 2. 颜色通道差分 
    cv::split(dst_BR, channels);

    // 3. 二值化，蓝色0,红色1
    cv::threshold(channels[0], dst, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

    // 合并结果 (颜色且饱和度)
    cv::bitwise_and(dst, mask_saturation, dst);

    // 4. 垂直方向模糊
    cv::blur(dst, dst, cv::Size(1, 3)); 

    // 调试显示
    //cv::imshow("饱和度掩码", mask_saturation); 
    cv::imshow("最终掩码", dst);
    cv::waitKey(1);

    return dst;
}
