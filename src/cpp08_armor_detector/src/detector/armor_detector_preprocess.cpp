#include "cpp08_armor_detector/detector/armor_detector.hpp"

//这里都是传统识别，已没用了，后续可以删除

cv::Mat ArmorDetector::preprocess(cv::Mat img)
{
    // 预处理常量设置
    const int GRAY_THRESH = 60; // 灰度二值化阈值，根据场地光线和相机曝光调整

    cv::Mat gray_img, binary_img;

    // 1. 灰度化：丢弃彩色信息，只保留亮度
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    // 2. 固定阈值二值化：把亮度大于 GRAY_THRESH (160) 的变成纯白，其余变纯黑
    cv::threshold(gray_img, binary_img, GRAY_THRESH, 255, cv::THRESH_BINARY);

    // 3. 形态学膨胀：使用 3x3 矩形核，将被二值化切断的微小碎块连接起来，让灯条更完整
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binary_img, binary_img, kernel);

    cv::imshow("Simple Preprocess", binary_img);
    cv::waitKey(1);

    // 返回处理好的二值图给外层的 extractLightBars 使用
    return binary_img;
}


// {
//     const int IMAGE_BRIGHT = 30;       // 全局亮度增益
//     const int THRESHOLD_VALUE = 100;   // BGR二值化阈值
//     const int MIN_SATURATION = 50;    // 最小饱和度阈值 (0-255)
//     const int MIN_BRIGHTNESS = 150;    // [新增] 最小亮度阈值 (0-255)

//     cv::Mat dst_BR, dst;
//     std::vector<cv::Mat> channels;

//     // 1. 全局亮度调整
//     {
//         cv::Mat BrightnessLut(1, 256, CV_8UC1); 
//         for (int i = 0; i < 256; i++) {
//             BrightnessLut.at<uchar>(i) = cv::saturate_cast<uchar>(i + IMAGE_BRIGHT);
//         }
//         cv::LUT(img, BrightnessLut, dst_BR);
//     }

//     // 计算饱和度掩码 和 亮度掩码
//     cv::Mat hsv, mask_saturation, mask_brightness;
//     cv::cvtColor(dst_BR, hsv, cv::COLOR_BGR2HSV);
//     std::vector<cv::Mat> hsv_channels;
//     cv::split(hsv, hsv_channels);
    
//     cv::Mat S = hsv_channels[1]; // 提取饱和度通道
//     cv::Mat V = hsv_channels[2]; // 提取亮度通道

//     // 饱和度二值化
//     cv::threshold(S, mask_saturation, MIN_SATURATION, 255, cv::THRESH_BINARY);
//     // 亮度二值化：只有亮度 > 150 的地方才是白色
//     cv::threshold(V, mask_brightness, MIN_BRIGHTNESS, 255, cv::THRESH_BINARY);

//     // 2. 颜色通道差分 (这里是提取B通道)
//     cv::split(dst_BR, channels);

//     // 3. 二值化，蓝色0,红色1 (这里是对B通道阈值化)
//     cv::threshold(channels[0], dst, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

//     // 合并结果 (颜色 且 饱和度 且 亮度)
//     cv::bitwise_and(dst, mask_saturation, dst);
//     cv::bitwise_and(dst, mask_brightness, dst); 

//     // 4. 垂直方向模糊
//     cv::blur(dst, dst, cv::Size(1, 3)); 

//     // 调试显示 
//     cv::imshow("最终掩码", dst);
//     cv::waitKey(1);

//     return dst;
// }