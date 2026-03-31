#include "cpp08_armor_detector/armor_detector_matching.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include <set>
#include <algorithm>

std::vector<DetectedArmor> matchLightBars(const std::vector<cv::RotatedRect>& lightBars, 
                                         const cv::Size& imgSize) {
    std::vector<DetectedArmor> all_armors;
    cv::Point2f imgCenter(imgSize.width / 2.0f, imgSize.height / 2.0f);

    // 灯条配对
    for (int i = 0; i < static_cast<int>(lightBars.size()); i++) {
        for (int j = i + 1; j < static_cast<int>(lightBars.size()); j++) {
            auto bar1 = lightBars[i];
            auto bar2 = lightBars[j];   

            // 角度差检查
            float Contour_angle = std::abs(bar1.angle - bar2.angle);
            if (Contour_angle >= 6.0f) {
                continue;
            }

            // 长度差比率和宽度差比率检查
            float Contour_Len1 = std::abs(bar1.size.height - bar2.size.height) / std::max(bar1.size.height, bar2.size.height);
            float Contour_Len2 = std::abs(bar1.size.width - bar2.size.width) / std::max(bar1.size.width, bar2.size.width);
            if (Contour_Len1 > 0.5 || Contour_Len2 > 0.8) {
                continue;            
            }

            // 中心点距离检查
            cv::Point2f center_diff = bar1.center - bar2.center;
            float center_dist = cv::norm(center_diff);
            float max_h = std::max(bar1.size.height, bar2.size.height);
            float wh_ratio = center_dist / max_h;
            if (wh_ratio < 1.0f || wh_ratio > 4.0f) {
                continue;
            }

            // 左右灯条区分 (X 更小的在左边)
            cv::RotatedRect left_bar = bar1.center.x < bar2.center.x ? bar1 : bar2;
            cv::RotatedRect right_bar = bar1.center.x < bar2.center.x ? bar2 : bar1;

            // 获取稳定的灯条上下端点
            cv::Point2f left_top, left_bottom;
            getBarEndpoints(left_bar, left_top, left_bottom);

            cv::Point2f right_top, right_bottom;
            getBarEndpoints(right_bar, right_top, right_bottom);
            
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
            
            double ideal_ratio = 2.2;
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
