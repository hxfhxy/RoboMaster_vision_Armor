#include "cpp08_armor_detector/armor_detector_matching.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include <set>
#include <algorithm>

std::vector<DetectedArmor> matchLightBars(const std::vector<cv::RotatedRect>& lightBars, 
                                         const cv::Size& imgSize) {
        std::vector<DetectedArmor> all_armors; 
        //bool foundBest = false;
        //double minDist = 999999, bestYaw = 0, bestPitch = 0, bestDist = 0;
        cv::Point2f imgCenter(imgSize.width / 2.0f, imgSize.height / 2.0f);
        cv::RotatedRect bestArmorRotatedRect;
        std::vector<cv::Point2f> best_pnp_corners;

        
        // 4.4 灯条配对
        for (int i = 0; i < static_cast<int>(lightBars.size()); i++)
        {
            for (int j = i + 1; j < static_cast<int>(lightBars.size()); j++)
            {
                auto bar1 = lightBars[i];
                auto bar2 = lightBars[j];   
                //判断是否为相同灯条
                float Contour_angle = std::abs(bar1.angle - bar2.angle); //角度差
                if (Contour_angle > 90.0f) {
                    Contour_angle = 180.0f - Contour_angle;
                }
                if (Contour_angle >= 20.0f){
                    continue;
                }
                // //长度差比率
                float Contour_Len1 = abs(bar1.size.height - bar2.size.height) / std::max(bar1.size.height, bar2.size.height);
                // //宽度差比率
                float Contour_Len2 = abs(bar1.size.width - bar2.size.width) / std::max(bar1.size.width, bar2.size.width);
                if (Contour_Len1 > 0.5 || Contour_Len2 > 0.8){
                    continue;            
                }
                 // 计算两个灯条中心点的垂直距离
                float y_diff = std::abs(bar1.center.y - bar2.center.y);
                // 如果 y_diff 太大，说明两个灯条一个在上一个在下，显然不是装甲板
                float max_h = std::max(bar1.size.height, bar2.size.height);
                if (y_diff > max_h * 1.2f) { 
                    continue; 
                }       
                // 计算两个灯条中心点的水平距离
                float center_dist = cv::norm(bar1.center - bar2.center);

                // 计算水平距离与灯条高度的比值
                float wh_ratio = center_dist / max_h;

                // 装甲板不能太窄，也不能太宽
                if (wh_ratio < 1.0f||wh_ratio > 6.0f) {
                    continue;
                }
                
                // 1. 左右灯条区分 (X 更小的在左边)
                cv::RotatedRect left_bar = bar1.center.x < bar2.center.x ? bar1 : bar2;
                cv::RotatedRect right_bar = bar1.center.x < bar2.center.x ? bar2 : bar1;

                // 2. 获取稳定的灯条上下端点
                cv::Point2f left_top, left_bottom;
                getBarEndpoints(left_bar, left_top, left_bottom);
                //left_top={(left_top.x + left_bottom.x) / 2.0f, left_top.y}; // 灯条中心点
                //left_bottom={(left_top.x + left_bottom.x) / 2.0f, left_bottom.y}; // 灯条中心点

                cv::Point2f right_top, right_bottom;
                getBarEndpoints(right_bar, right_top, right_bottom);
                //right_top={(right_top.x + right_bottom.x) / 2.0f, right_top.y}; // 灯条中心点
                //right_bottom={(right_top.x + right_bottom.x) / 2.0f, right_bottom.y}; // 灯条中心点
                
            
                // 组装PnP角点顺序：左上 左下 右下 右上
                std::vector<cv::Point2f> pnp_corners = {left_top, left_bottom, right_bottom, right_top};
                cv::RotatedRect armor_rotated_rect = cv::minAreaRect(pnp_corners);

                // 创建装甲板对象
                DetectedArmor armor;
                armor.rect = armor_rotated_rect;
                armor.pnp_corners = pnp_corners;
                armor.light_bar_ids = {i, j};
                armor.dist_to_center = cv::norm(armor_rotated_rect.center - imgCenter);

                // 计算置信度 (尺寸匹配 + 宽高比接近理想值 + 平行度)
                double score_size = (1.0 - Contour_Len1) * (1.0 - Contour_Len2);
                
                double ideal_ratio = 2.4;
                double ratio_diff = std::abs(wh_ratio - ideal_ratio);
                double score_ratio = 1.0 / (1.0 + ratio_diff);
                
                double score_angle = 1.0 / (1.0 + Contour_angle / 5.0);
                
                armor.confidence = score_size * score_ratio * score_angle;

                all_armors.push_back(armor);
            }
        }

    return all_armors;
}

std::vector<DetectedArmor> deduplicateArmors(const std::vector<DetectedArmor>& armors) {
    if (armors.empty()) return {};

    // 1. 先按置信度从高到低排序
    std::vector<DetectedArmor> sorted_armors = armors;
    std::sort(sorted_armors.begin(), sorted_armors.end(),
        [](const DetectedArmor& a, const DetectedArmor& b) {
            return a.confidence > b.confidence;
        });

    std::vector<DetectedArmor> final_armors;
    std::set<int> used_light_bars;

    // 2. 遍历排序后的装甲板
    for (const auto& armor : sorted_armors) {
        int id1 = armor.light_bar_ids.first;
        int id2 = armor.light_bar_ids.second;

        // 3. 如果这两个灯条都没被用过，就保留这个装甲板
        if (used_light_bars.find(id1) == used_light_bars.end() &&
            used_light_bars.find(id2) == used_light_bars.end()) {
            
            final_armors.push_back(armor);
            used_light_bars.insert(id1);
            used_light_bars.insert(id2);
        }
    }

    return final_armors;
}

void calculatePnP(DetectedArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Point3f>& objectPoints) {
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(objectPoints, armor.pnp_corners, cameraMatrix, distCoeffs, rvec, tvec);
    
    if (success) {
        armor.tvec = tvec.clone();
        armor.rvec = rvec.clone();
        // 提取空间平移坐标 (X, Y, Z)
        double x = tvec.at<double>(0, 0);
        double y = tvec.at<double>(1, 0);
        double z = tvec.at<double>(2, 0);
        
        // 1. 计算距离 (单位取决于你的 objectPoints，通常是 mm)
        armor.distance = std::sqrt(x*x + y*y + z*z); // 或者 cv::norm(tvec)
        
        // 2. 计算偏航角 (yaw) 和俯仰角 (pitch)
        double delta_yaw_rad = std::atan2(tvec.at<double>(0),tvec.at<double>(2));
        double delta_pitch_rad = std::atan2(-tvec.at<double>(1),tvec.at<double>(2));
        double distance = std::sqrt(x*x + y*y + z*z);

        armor.yaw = -delta_yaw_rad * 180.0 / CV_PI;   // 取反：目标在右，Yaw减小
        armor.pitch = delta_pitch_rad * 180.0 / CV_PI; // 这里如果实测方向反了，再加个负号

    }else{
        armor.distance = 0;
        armor.yaw = 0;
        armor.pitch = 0;    
    }
}
