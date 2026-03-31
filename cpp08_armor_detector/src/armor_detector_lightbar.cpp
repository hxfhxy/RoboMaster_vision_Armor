#include "cpp08_armor_detector/armor_detector_lightbar.hpp"

void sortCornersClockwise(std::vector<cv::Point2f>& corners) {  
    if (corners.size() != 4) return;

    // 1. 计算中心点
    cv::Point2f center(0, 0);
    for (const auto& p : corners) center += p;
    center *= 0.25f;

    // 2. 使用 atan2 按照极角排序
    std::sort(corners.begin(), corners.end(), [center](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - center.y, a.x - center.x) <     
            std::atan2(b.y - center.y, b.x - center.x);
    });
}

void getBarEndpoints(const cv::RotatedRect& bar, cv::Point2f& out_top, cv::Point2f& out_bottom) {
    std::vector<cv::Point2f> corners(4);
    cv::Point2f pts[4];
    bar.points(pts);
    for(int i=0; i<4; i++) corners[i] = pts[i];

    // 顺时针排序
    sortCornersClockwise(corners);

    // 在顺时针序列中，灯条的两个长边是相对的，短边也是相对的
    // 计算 0-1 边和 1-2 边的长度
    double d01 = cv::norm(corners[0] - corners[1]);
    double d12 = cv::norm(corners[1] - corners[2]);

    cv::Point2f mid_a, mid_b;
    if (d01 < d12) {
        // 0-1 是短边（顶/底），2-3 是另一条短边
        mid_a = (corners[0] + corners[1]) * 0.5f;
        mid_b = (corners[2] + corners[3]) * 0.5f;
    } else {
        // 1-2 是短边，3-0 是另一条短边
        mid_a = (corners[1] + corners[2]) * 0.5f;
        mid_b = (corners[3] + corners[0]) * 0.5f;
    }

    // 最后简单判断一下上下（y小的是top）
    if (mid_a.y < mid_b.y) {
        out_top = mid_a; out_bottom = mid_b;
    } else {
        out_top = mid_b; out_bottom = mid_a;
    }
}

cv::RotatedRect fixLightBarSize(cv::RotatedRect rect) {
    if (rect.size.width > rect.size.height) {
        std::swap(rect.size.width, rect.size.height); // 宽高互换
        rect.angle += 90.0f; // 角度同步修正
    }
    return rect;
}

std::vector<cv::RotatedRect> extractLightBars(const cv::Mat& mask, int min_area) {
    std::vector<cv::RotatedRect> lightBars;
    std::vector<std::vector<cv::Point>> contours;
    
    // 轮廓提取
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (auto &cnt : contours) {
        // 1. 面积筛选
        if (cv::contourArea(cnt) <= min_area) {
            continue;
        }
        
        // 2. 用旋转矩形拟合灯条
        cv::RotatedRect light_rect = cv::minAreaRect(cnt);
        light_rect = fixLightBarSize(light_rect);
        
        // 3. 加入列表
        lightBars.push_back(light_rect);
    }
    
    return lightBars;
}
