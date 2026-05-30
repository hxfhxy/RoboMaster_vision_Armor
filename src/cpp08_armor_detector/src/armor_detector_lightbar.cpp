#include "cpp08_armor_detector/armor_detector_lightbar.hpp"

/**
 * @brief 顺时针排序四个角点（用于规整旋转矩形的角点顺序）
 * @param corners 输入4个角点，输出排序后的4个角点
 */
void sortCornersClockwise(std::vector<cv::Point2f>& corners) {  
    // 必须是4个点，否则直接返回
    if (corners.size() != 4) return;

    // 1. 计算四个点的中心点
    cv::Point2f center(0, 0);
    for (const auto& p : corners) center += p;
    center *= 0.25f;  // 累加后除以4

    // 2. 按极角从小到大排序 → 顺时针排序
    // atan2(y差, x差) 计算点相对于中心点的角度
    std::sort(corners.begin(), corners.end(), [center](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - center.y, a.x - center.x) <     
               std::atan2(b.y - center.y, b.x - center.x);
    });
}

/**
 * @brief 从灯条旋转矩形中，提取灯条的**上端点**和**下端点**
 * @param bar 输入的灯条旋转矩形
 * @param out_top 输出：灯条上端点
 * @param out_bottom 输出：灯条下端点
 */
void getBarEndpoints(const cv::RotatedRect& bar, cv::Point2f& out_top, cv::Point2f& out_bottom) {
    // 定义容器存储4个角点
    std::vector<cv::Point2f> corners(4);
    cv::Point2f pts[4];
    bar.points(pts);  // 获取旋转矩形的4个顶点
    for(int i=0; i<4; i++) corners[i] = pts[i];

    // 顺时针排序4个角点
    sortCornersClockwise(corners);

    // 灯条是细长矩形，长边很长，短边很短
    // 计算相邻两个边的长度：0-1 和 1-2
    double d01 = cv::norm(corners[0] - corners[1]);
    double d12 = cv::norm(corners[1] - corners[2]);

    cv::Point2f mid_a, mid_b;
    if (d01 < d12) {
        // 0-1 是短边 → 取中点作为灯条的一个端点
        mid_a = (corners[0] + corners[1]) * 0.5f;
        mid_b = (corners[2] + corners[3]) * 0.5f;
    } else {
        // 1-2 是短边 → 取中点作为灯条的一个端点
        mid_a = (corners[1] + corners[2]) * 0.5f;
        mid_b = (corners[3] + corners[0]) * 0.5f;
    }

    // 比较y坐标：y越小越靠上
    if (mid_a.y < mid_b.y) {
        out_top = mid_a; 
        out_bottom = mid_b;
    } else {
        out_top = mid_b; 
        out_bottom = mid_a;
    }
}

/**
 * @brief 修正灯条矩形：保证 高度 > 宽度，让灯条永远是竖直的
 * @param rect 原始旋转矩形
 * @return 修正后的旋转矩形
 */
cv::RotatedRect fixLightBarSize(cv::RotatedRect rect) {
    // 如果宽度比高度大，说明矩形是横过来的
    if (rect.size.width > rect.size.height) {
        std::swap(rect.size.width, rect.size.height); // 宽高互换
        rect.angle += 90.0f; // 角度同步修正
    }
    return rect;
}

/**
 * @brief 从二值化图像中提取所有符合条件的灯条
 * @param mask 二值化图像（预处理后的灯条区域）
 * @param min_area 最小面积阈值
 * @return 筛选后的灯条旋转矩形列表
 */
std::vector<cv::RotatedRect> extractLightBars(const cv::Mat& mask, int min_area) {
    std::vector<cv::RotatedRect> lightBars;  // 最终输出的灯条
    std::vector<std::vector<cv::Point>> contours;  // 轮廓容器
    
    // 查找图像中所有独立轮廓（只检测最外层轮廓）
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 遍历每一个轮廓
    for (auto &cnt : contours)
    {
        // 1. 面积筛选：过滤掉太小的噪点
        if (cv::contourArea(cnt) <= 20) {
            continue;
        }

        // 2. 用最小外接矩形拟合轮廓 → 得到灯条旋转矩形
        cv::RotatedRect light_rect = cv::minAreaRect(cnt);

        // 3. 修正灯条矩形：保证高 > 宽
        light_rect = fixLightBarSize(light_rect);

        // 4. 加入灯条列表
        lightBars.push_back(light_rect);
    }
    
    return lightBars;
}
