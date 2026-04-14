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
        // 定义图像中心点坐标（用于后续计算装甲板优先级）
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
                //判断是否为相同灯条
                // ===================== 筛选条件1：灯条角度差（保证两个灯条近似平行） =====================
                float Contour_angle = std::abs(bar1.angle - bar2.angle); //角度差
                // 角度差取最小夹角（0-90°，因为灯条可能是镜像的）
                if (Contour_angle > 90.0f) {
                    Contour_angle = 180.0f - Contour_angle;
                }
                // 角度差超过20°，说明不平行，不可能是同一装甲板的灯条，直接跳过
                if (Contour_angle >= 20.0f){
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
                if (Contour_Len1 > 0.5 || Contour_Len2 > 0.8){
                    continue;            
                }
                 // ===================== 筛选条件3：灯条垂直距离（保证两个灯条在同一水平线上） =====================
                 // 计算两个灯条中心点的垂直距离（y坐标差的绝对值）
                float y_diff = std::abs(bar1.center.y - bar2.center.y);
                // 如果 y_diff 太大，说明两个灯条一个在上一个在下，显然不是装甲板
                float max_h = std::max(bar1.size.height, bar2.size.height);
                if (y_diff > max_h * 1.2f) { 
                    continue; 
                }       
                // ===================== 筛选条件4：装甲板宽高比（保证形状合理） =====================
                // 计算两个灯条中心点的直线距离
                float center_dist = cv::norm(bar1.center - bar2.center);

                // 计算水平距离与灯条高度的比值（近似装甲板的宽高比）
                float wh_ratio = center_dist / max_h;

                // 装甲板不能太窄（<1.0），也不能太宽（>6.0）
                if (wh_ratio < 1.0f||wh_ratio > 6.0f) {
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
 * @brief PnP解算：通过2D图像角点 → 3D空间位姿（距离、yaw、pitch）
 * @param armor 装甲板对象（输入：2D图像角点；输出：3D位姿）
 * @param cameraMatrix 相机内参矩阵（提前标定好）
 * @param distCoeffs 相机畸变系数（提前标定好）
 * @param objectPoints 装甲板的3D物理坐标（单位：mm，根据实际装甲板尺寸定义）
 */
void calculatePnP(DetectedArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Point3f>& objectPoints) {
    cv::Mat rvec, tvec; // 旋转向量、平移向量（PnP解算的输出）
    // 调用OpenCV的PnP解算函数，求解3D-2D的对应关系
    bool success = cv::solvePnP(objectPoints, armor.pnp_corners, cameraMatrix, distCoeffs, rvec, tvec);
    
    if (success) {
        armor.tvec = tvec.clone(); // 保存平移向量（3D空间位置：X水平、Y垂直、Z深度）
        armor.rvec = rvec.clone(); // 保存旋转向量（3D空间姿态）
        // 提取空间平移坐标 (X, Y, Z)
        double x = tvec.at<double>(0, 0);
        double y = tvec.at<double>(1, 0);
        double z = tvec.at<double>(2, 0);
        
        // ===================== 1. 计算目标距离（欧氏距离） =====================
        armor.distance = std::sqrt(x*x + y*y + z*z); 
        
        // ===================== 2. 计算偏航角 (yaw) 和俯仰角 (pitch) =====================
        // yaw：水平方向角度（x/z的反正切，目标在相机右侧为正）
        double delta_yaw_rad = std::atan2(tvec.at<double>(0),tvec.at<double>(2));
        // pitch：垂直方向角度（-y/z的反正切，目标在相机上方为正）
        double delta_pitch_rad = std::atan2(-tvec.at<double>(1),tvec.at<double>(2));
        double distance = std::sqrt(x*x + y*y + z*z);

        // 弧度转角度，存入装甲板对象
        armor.yaw = delta_yaw_rad * 180.0 / CV_PI;   // 取反：目标在右，Yaw减小
        armor.pitch = delta_pitch_rad * 180.0 / CV_PI; 

    }else{
        // PnP解算失败，将距离、角度置零
        armor.distance = 0;
        armor.yaw = 0;
        armor.pitch = 0;    
    }
}