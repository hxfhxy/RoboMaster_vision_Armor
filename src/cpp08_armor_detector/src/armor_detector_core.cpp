#include "cpp08_armor_detector/armor_detector.hpp"
#include "cpp08_armor_detector/uart_protocol.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include "cpp08_armor_detector/armor_detector_matching.hpp"

// ================== 【新增】setGimbalCurrent 实现 ==================
void ArmorDetector::setGimbalCurrent(float yaw_current, float pitch_current) {
    gimbal_yaw_current_ = yaw_current;
    gimbal_pitch_current_ = pitch_current;
}
// ====================================================================


    // 4. detect 主流程：找到装甲板并输出 ArmorTarget
    cpp08_armor_detector::msg::ArmorTarget ArmorDetector::detect(cv::Mat img)
    {
        cpp08_armor_detector::msg::ArmorTarget target_msg;
        target_msg.is_detected = false;

        // 4.1 预处理
        cv::Mat mask = preprocess(img);

        // 4.2 提取灯条
        std::vector<cv::RotatedRect> lightBars = extractLightBars(mask, 10);

        // 4.3 灯条配对和装甲板匹配
        std::vector<DetectedArmor> all_armors = matchLightBars(lightBars, img.size());

        // 4.3.1 PnP解算
        for (auto& armor : all_armors) {
            calculatePnP(armor, cameraMatrix, distCoeffs, objectPoints);
        }

        // 4.4 去重
        std::vector<DetectedArmor> final_armors = deduplicateArmors(all_armors);

        // 4.5 选最优装甲板
        cv::Point2f imgCenter(img.cols / 2.0f, img.rows / 2.0f);
        bool foundBest = false;
        double minDist = 999999, bestYaw = 0, bestPitch = 0, bestDist = 0;
        cv::RotatedRect bestArmorRotatedRect;
        std::vector<cv::Point2f> best_pnp_corners;

        if (!final_armors.empty()) {
            // 排序规则：1. 优先置信度高的；2. 置信度差不超过0.05时，选离中心近的
            std::sort(final_armors.begin(), final_armors.end(),
                [&imgCenter](const DetectedArmor& a, const DetectedArmor& b) {
                    if (std::abs(a.confidence - b.confidence) > 0.05) {
                        return a.confidence > b.confidence;
                    }
                    return a.dist_to_center < b.dist_to_center;
                });

            // 选排序后的第一个作为最优
            const auto& best_armor = final_armors[0];
            
            minDist = best_armor.dist_to_center;
            bestYaw = best_armor.yaw;
            bestPitch = best_armor.pitch;
            bestDist = best_armor.distance;
            bestArmorRotatedRect = best_armor.rect;
            best_pnp_corners = best_armor.pnp_corners;
            foundBest = true;
            found = true;
    
            minDist = best_armor.dist_to_center;
            bestYaw = best_armor.yaw;     // 相对偏移角
            bestPitch = best_armor.pitch; // 相对偏移角
            bestDist = best_armor.distance;
            bestArmorRotatedRect = best_armor.rect;
            best_pnp_corners = best_armor.pnp_corners;
            foundBest = true;
            found = true;

            // 绝对目标角度计算
            // 1. 基础叠加：当前角度 + 相对偏移 + hpp 里的 YAW_OFFSET/PITCH_OFFSET
            target_yaw_ = gimbal_yaw_current_ + bestYaw + YAW_OFFSET;
            target_pitch_ = gimbal_pitch_current_ + bestPitch + PITCH_OFFSET;

            // 2. 子弹下坠补偿（简单的物理模型，假设子弹飞行时间和距离成正比，重力加速度为9.8m/s²）
            // if (bestDist > 0) {
            //     float distance_m = bestDist / 1000.0f; // mm 转 m
            //     float fly_time = distance_m / BULLET_SPEED;
            //     float drop_m = 0.5f * 9.8f * fly_time * fly_time; // 重力下坠
            //     float pitch_comp = std::atan2(drop_m, distance_m) * 180.0f / CV_PI;
            //     target_pitch_ += pitch_comp; // 抬枪
            // }

            // 3. Yaw 归一化到 [-180, 180]
            while (target_yaw_ > 180.0f)  target_yaw_ -= 360.0f;
            while (target_yaw_ < -180.0f) target_yaw_ += 360.0f;

            // 4. Pitch 硬限位
            if (target_pitch_ > PITCH_MAX) target_pitch_ = PITCH_MAX;
            if (target_pitch_ < PITCH_MIN) target_pitch_ = PITCH_MIN;
            // ==============================================================
        }

        // 绘制所有去重后的装甲板
        for (auto &armor : final_armors) {
            // 画装甲板框（绿色）
            cv::Point2f armor_corners[4];
            armor.rect.points(armor_corners);
            for (int k = 0; k < 4; k++) {
                cv::line(img, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(0, 255, 0), 2);
            }

            // 画角点（红色）
            for (auto &pt : armor.pnp_corners) {
                cv::circle(img, pt, 4, cv::Scalar(0, 0, 255), -1);
            }
            
            // 把置信度画在旁边，方便调试
            cv::putText(img, "Conf:" + std::to_string((int)(armor.confidence*100)) + "%", 
                        armor.rect.center + cv::Point2f(-20, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1);
        }

        // 4.6 卡尔曼滤波和结果输出
        if (foundBest)
        {
            kf.predict();
            cv::Mat z = (cv::Mat_<float>(1, 1) << (float)bestYaw);
            kf.correct(z);
            float filteredYaw = kf.x.at<float>(0);

            target_msg.is_detected = true;
            target_msg.yaw = static_cast<float>(bestYaw);
            target_msg.pitch = static_cast<float>(bestPitch);
            target_msg.distance = static_cast<float>(bestDist);
            target_msg.filtered_yaw = filteredYaw;
            target_msg.center_x = static_cast<float>(bestArmorRotatedRect.center.x);
            target_msg.center_y = static_cast<float>(bestArmorRotatedRect.center.y);

            cv::Point2f armor_corners[4];
            bestArmorRotatedRect.points(armor_corners);
            for (int k = 0; k < 4; k++) {
                cv::line(img, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(0, 0, 255), 2);
            }

            // 画坐标点
            for (auto &pt : best_pnp_corners)
            {
                cv::putText(img, "(" + std::to_string((int)pt.x) + "," + std::to_string((int)pt.y) + ")", pt + cv::Point2f(5, -5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }

            rawYawList.push_back((float)bestYaw);
            filteredYawList.push_back(filteredYaw);
            if (rawYawList.size() > 400)
                rawYawList.pop_front();
            if (filteredYawList.size() > 400)
                filteredYawList.pop_front();

            // 在装甲板上方画文字
            cv::Point2f text_pos = bestArmorRotatedRect.center;
            text_pos.y -= bestArmorRotatedRect.size.height / 2 + 20;
            cv::putText(img, "Yaw:" + std::to_string(bestYaw) + " Pitch:" + std::to_string(bestPitch) + " Dist:" + std::to_string(bestDist),
                        text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

            // 计算机器人中心
            find_robot_center();
            if (find_center && !center_2d.empty())
            {
                cv::circle(img, center_2d[0], 8, cv::Scalar(0, 255, 255), -1);
                cv::putText(img, "Robot Center", center_2d[0] + cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
        }
        else
        {
            found = false;
        }

        drawYawPlot();

        return target_msg;
    }
