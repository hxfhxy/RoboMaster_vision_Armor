#include "cpp08_armor_detector/armor_detector_matching.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include <set>
#include <algorithm>

/**
 * @brief 灯条两两配对：通过几何规则筛选出符合装甲板特征的组合
 * @param lightBars 预处理后提取到的所有灯条旋转矩形
 * @param imgSize 输入图像的尺寸（用于计算装甲板到图像中心的距离）
 * @return 所有初步匹配成功的装甲板列表
 */
std::vector<DetectedArmor> matchLightBars(const std::vector<cv::RotatedRect>& lightBars, 
                                         const cv::Size& imgSize) {
        std::vector<DetectedArmor> all_armors; 
        // 定义图像中心点坐标
        cv::Point2f imgCenter(imgSize.width / 2.0f, imgSize.height / 2.0f);
        cv::RotatedRect bestArmorRotatedRect;
        std::vector<cv::Point2f> best_pnp_corners;

        
        // 4.4 灯条配对
        // 双重循环遍历所有灯条的两两组合（i<j避免重复配对）
        for (int i = 0; i < static_cast<int>(lightBars.size()); i++)
        {
            for (int j = i + 1; j < static_cast<int>(lightBars.size()); j++)
            {
                auto bar1 = lightBars[i];
                auto bar2 = lightBars[j];   
                // ===================== 筛选条件 0：几何过滤 =====================
                float ratio1 = bar1.size.width / bar1.size.height;
                float ratio2 = bar2.size.width / bar2.size.height;
                if (ratio1 > 0.5f || ratio1 < 0.1f || ratio2 > 0.5f || ratio2 < 0.1f) {
                    continue; // 灯条太宽或太窄，直接跳过
                }
                //判断是否为相同灯条
                // ===================== 筛选条件1：灯条角度差（保证两个灯条近似平行） =====================
                float Contour_angle = std::abs(bar1.angle - bar2.angle); //角度差
                // 角度差取最小夹角（0-90°，因为灯条可能是镜像的）
                if (Contour_angle > 90.0f) {
                    Contour_angle = 180.0f - Contour_angle;
                }
                // 角度差超过5°，说明不平行，不可能是同一装甲板的灯条，直接跳过
                if (Contour_angle >= 5.0f){
                    continue;
                }
                // ===================== 筛选条件2：灯条尺寸差（保证两个灯条大小相近） =====================
                // //长度差比率
                // 灯条高度方向的长度差比率（差值除以较大值，归一化到0-1）
                float Contour_Len1 = abs(bar1.size.height - bar2.size.height) / std::max(bar1.size.height, bar2.size.height);
                // //宽度差比率
                // 灯条宽度方向的宽度差比率
                float Contour_Len2 = abs(bar1.size.width - bar2.size.width) / std::max(bar1.size.width, bar2.size.width);
                // 长度差超过50%或宽度差超过80%，直接跳过
                if (Contour_Len1 > 0.5 || Contour_Len2 > 0.5){
                    continue;            
                }
                 // ===================== 筛选条件3：灯条垂直距离（保证两个灯条在同一水平线上） =====================
                 // 计算两个灯条中心点的垂直距离（y坐标差的绝对值）
                float y_diff = std::abs(bar1.center.y - bar2.center.y);
                // 如果 y_diff 太大，说明两个灯条一个在上一个在下，显然不是装甲板
                float max_h = std::max(bar1.size.height, bar2.size.height);
                // 灯条垂直距离不能超过最大灯条高度的1.2倍（根据实际装甲板尺寸调整）
                if (y_diff > max_h * 0.8f) { 
                    continue; 
                }       
                // ===================== 筛选条件4：装甲板宽高比（保证形状合理） =====================
                // 计算两个灯条中心点的直线距离
                float center_dist = cv::norm(bar1.center - bar2.center);

                // 计算水平距离与灯条高度的比值（近似装甲板的宽高比）
                float wh_ratio = center_dist / max_h;

                // 装甲板宽高比应该在合理范围内，过大或过小都不合理（根据实际装甲板尺寸调整）
                if (wh_ratio < 0.8f||wh_ratio > 3.0f) {
                    continue;
                }
                
                // ===================== 1. 左右灯条区分 (X 更小的在左边) =====================
                cv::RotatedRect left_bar = bar1.center.x < bar2.center.x ? bar1 : bar2;
                cv::RotatedRect right_bar = bar1.center.x < bar2.center.x ? bar2 : bar1;

                // ===================== 2. 获取稳定的灯条上下端点 =====================
                cv::Point2f left_top, left_bottom;
                getBarEndpoints(left_bar, left_top, left_bottom);
                //left_top={(left_top.x + left_bottom.x) / 2.0f, left_top.y}; // 灯条中心点
                //left_bottom={(left_top.x + left_bottom.x) / 2.0f, left_bottom.y}; // 灯条中心点

                cv::Point2f right_top, right_bottom;
                getBarEndpoints(right_bar, right_top, right_bottom);
                //right_top={(right_top.x + right_bottom.x) / 2.0f, right_top.y}; // 灯条中心点
                //right_bottom={(right_top.x + right_bottom.x) / 2.0f, right_bottom.y}; // 灯条中心点
                
            
                // ===================== 3. 组装PnP角点顺序：左上 左下 右下 右上 =====================
                std::vector<cv::Point2f> pnp_corners = {left_top, left_bottom, right_bottom, right_top};

                // ===================== 4. 生成装甲板的最小外接旋转矩形 =====================
                cv::RotatedRect armor_rotated_rect = cv::minAreaRect(pnp_corners);

                // ===================== 5. 填充装甲板对象的属性 =====================
                DetectedArmor armor;
                armor.rect = armor_rotated_rect;
                armor.pnp_corners = pnp_corners;
                armor.light_bar_ids = {i, j}; // 记录组成该装甲板的两个灯条的索引（用于后续去重）
                armor.dist_to_center = cv::norm(armor_rotated_rect.center - imgCenter); // 计算装甲板中心到图像中心的距离

                // ===================== 6. 计算装甲板的综合置信度（加权平均） =====================
                // 尺寸匹配分：尺寸越接近，分数越高（0-1）
                double score_size = (1.0 - Contour_Len1) * (1.0 - Contour_Len2);

                double ideal_ratio = 2.4; // 理想的宽高比，根据装甲板实际尺寸调整
                double ratio_diff = std::abs(wh_ratio - ideal_ratio);
                // 比例匹配分：越接近理想宽高比，分数越高（0-1）
                double score_ratio = 1.0 / (1.0 + ratio_diff);

                // 角度匹配分：越平行，分数越高（0-1）
                double score_angle = 1.0 / (1.0 + Contour_angle / 5.0);

                // 加权求和：尺寸30% + 比例40% + 角度30%
                armor.confidence = 0.3 * score_size + 0.4 * score_ratio + 0.3 * score_angle;


                // ===================== 7. 将匹配成功的装甲板加入列表 =====================
                all_armors.push_back(armor);
            }
        }

    return all_armors;
}


// 装甲板去重：保证每个灯条只属于一个装甲板，优先保留置信度高的
std::vector<DetectedArmor> deduplicateArmors(const std::vector<DetectedArmor>& armors) {
    // 空输入直接返回空
    if (armors.empty()) return {};

    // 复制装甲板列表，准备排序
    std::vector<DetectedArmor> sorted_armors = armors;
    // 按置信度 从高到低 排序
    std::sort(sorted_armors.begin(), sorted_armors.end(),
        [](const DetectedArmor& a, const DetectedArmor& b) {
            return a.confidence > b.confidence;
        });

    // 最终去重后的装甲板列表
    std::vector<DetectedArmor> final_armors;
    // 记录已经被使用过的灯条ID，防止重复利用
    std::set<int> used_light_bars;

    // 遍历已经按置信度排好序的装甲板
    for (const auto& armor : sorted_armors) {
        // 获取组成这个装甲板的两个灯条的ID
        int id1 = armor.light_bar_ids.first;
        int id2 = armor.light_bar_ids.second;

        // 如果两个灯条都没有被使用过
        if (used_light_bars.find(id1) == used_light_bars.end() &&
            used_light_bars.find(id2) == used_light_bars.end()) {
            
            // 保留这个装甲板
            final_armors.push_back(armor);
            // 标记这两个灯条为已使用
            used_light_bars.insert(id1);
            used_light_bars.insert(id2);
        }
    }
    // 返回去重后的结果
    return final_armors;
}


/**
 * @brief PnP解算：严格使用 IPPE 算法和 FLU 零点模型，并引入双解验证魔法
 */
void calculatePnP(DetectedArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Point3f>& /*objectPoints*/) {
    std::vector<cv::Point2f> image_points = {
        armor.pnp_corners[1], // 左下
        armor.pnp_corners[0], // 左上
        armor.pnp_corners[3], // 右上
        armor.pnp_corners[2]  // 右下
    };

    float pixel_width = cv::norm(armor.pnp_corners[0] - armor.pnp_corners[3]);
    float pixel_height = cv::norm(armor.pnp_corners[0] - armor.pnp_corners[1]);
    bool is_small = (pixel_width / pixel_height) < 3.2f;

    double half_w = is_small ? (135.0 / 2.0) : (230.0 / 2.0);
    double half_h = 55.0 / 2.0;

    // 【核心修复 1】：必须使用标准 OpenCV 坐标系 (X右, Y下, Z前)
    // 只有这样，PnP 算出来的法向量才是 Z 轴！
    std::vector<cv::Point3f> object_points_cv = {
        cv::Point3f(-half_w,  half_h, 0), // 左下
        cv::Point3f(-half_w, -half_h, 0), // 左上
        cv::Point3f( half_w, -half_h, 0), // 右上
        cv::Point3f( half_w,  half_h, 0)  // 右下
    };

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;
    int solutions = cv::solvePnPGeneric(
        object_points_cv, image_points, cameraMatrix, distCoeffs,
        rvecs, tvecs, false, cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), errors);

    if (solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        armor.distance = 0; armor.yaw = 0; armor.pitch = 0; 
        return;
    }

    int best = 0;
    if (rvecs.size() > 1 && tvecs.size() > 1) {
        auto yaw_from_rvec = [](cv::Mat &rv) {
            cv::Mat R;
            cv::Rodrigues(rv, R);
            // 提取 Z 轴向量，计算 Yaw 角
            return std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * 180.0 / CV_PI;
        };

        double yaw0 = yaw_from_rvec(rvecs[0]);
        double yaw1 = yaw_from_rvec(rvecs[1]);
        
        // 提取装甲板在 2D 图像上的倾斜趋势
        cv::Point2f left_axis = armor.pnp_corners[0] - armor.pnp_corners[1]; // 左侧灯条向量
        cv::Point2f right_axis = armor.pnp_corners[3] - armor.pnp_corners[2];// 右侧灯条向量
        // 根据图像特征计算一个粗略的 2D 偏转角趋势（正负号能反映是往左偏还是往右偏）
        double armor_angle_trend = (std::atan2(left_axis.x, left_axis.y) + 
                                    std::atan2(right_axis.x, right_axis.y));
        
        // 利用 2D 图像特征的倾向，去选择那个符合物理事实的 3D 解
        if ((armor_angle_trend > 0 && yaw1 > 0 && yaw0 < 0) || 
            (armor_angle_trend < 0 && yaw1 < 0 && yaw0 > 0)) {
            best = 1;
        }
    }
    armor.tvec = tvecs[best].clone(); 
    armor.rvec = rvecs[best].clone();
    
    double x = armor.tvec.at<double>(0, 0);
    double y = armor.tvec.at<double>(1, 0);
    double z = armor.tvec.at<double>(2, 0);
    
    armor.distance = std::sqrt(x*x + y*y + z*z); 
    cv::Mat R_best;
    cv::Rodrigues(armor.rvec, R_best);
    cv::Vec3d normal(R_best.at<double>(0, 2), R_best.at<double>(1, 2), R_best.at<double>(2, 2));
    cv::Vec3d pos(x, y, z);
    if (normal.dot(pos) > 0.0) {
        normal = -normal;
    }
    armor.yaw = std::atan2(normal[0], normal[2]) * 180.0 / CV_PI; // 这是真正的物体朝向
    
    armor.pitch = std::atan2(-y, z) * 180.0 / CV_PI;
}