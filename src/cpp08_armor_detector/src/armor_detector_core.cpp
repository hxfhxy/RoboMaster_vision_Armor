#include "cpp08_armor_detector/armor_detector.hpp"
#include "cpp08_armor_detector/uart_protocol.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include "cpp08_armor_detector/armor_detector_matching.hpp"

/**
 * @brief 设置当前云台的yaw和pitch角度
 * @param yaw_current 云台当前yaw角（水平角度）
 * @param pitch_current 云台当前pitch角（垂直角度）
 */
void ArmorDetector::setGimbalCurrent(float yaw_current, float pitch_current) {
    gimbal_yaw_current_ = yaw_current;
    gimbal_pitch_current_ = pitch_current;
}

/**
 * @brief 加载ONNX数字识别模型
 * @param model_path 模型文件路径
 */
void ArmorDetector::loadModel(const std::string& model_path) {
    // 从ONNX文件读取深度学习网络
    net_ = cv::dnn::readNetFromONNX(model_path);
    // 如果你有显卡环境，可以开启 CUDA 加速
    // net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    // net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}

/**
 * @brief 对检测到的装甲板进行数字分类识别（0-9）
 * @param src 原始图像
 * @param armors 检测到的装甲板列表（输出识别结果）
 */
void ArmorDetector::classifyArmors(const cv::Mat& src, std::vector<DetectedArmor>& armors) {
    // 模型未加载 或 无装甲板，直接返回
    if (net_.empty() || armors.empty()) return;

    // 模型输入尺寸：28x28（匹配训练模型输入）
    const cv::Size model_input_size(28, 28); 
    // 可视化显示尺寸
    const cv::Size display_size(300, 300);
    // 垂直方向扩展比例：防止裁剪装甲板数字区域
    const float vertical_padding_ratio = 0.6f;

    // 遍历所有装甲板，逐个识别数字
    for (auto& armor : armors) {
        // ===================== 1. 扩展装甲板ROI，只在垂直方向扩展 =====================
        std::vector<cv::Point2f> roi_corners = armor.pnp_corners;
        // 计算装甲板左右两边的高度
        float left_height = cv::norm(roi_corners[0] - roi_corners[1]);
        float right_height = cv::norm(roi_corners[3] - roi_corners[2]);
        // 平均高度
        float avg_height = (left_height + right_height) / 2.0f;
        // 计算垂直方向偏移量
        float y_offset = avg_height * vertical_padding_ratio;

        // 上下扩展ROI区域
        roi_corners[0].y -= y_offset;
        roi_corners[3].y -= y_offset;
        roi_corners[1].y += y_offset;
        roi_corners[2].y += y_offset;

        // ===================== 2. 透视变换：将倾斜装甲板校正为正矩形 =====================
        // 目标输出点（标准矩形）
        std::vector<cv::Point2f> dst_pts = {
            {0, 0},
            {0, (float)model_input_size.height},
            {(float)model_input_size.width, (float)model_input_size.height},
            {(float)model_input_size.width, 0}
        };
        // 计算透视变换矩阵
        cv::Mat warp_mat = cv::getPerspectiveTransform(roi_corners, dst_pts);
        cv::Mat roi_for_model;
        // 执行透视变换，得到模型输入图像
        cv::warpPerspective(src, roi_for_model, warp_mat, model_input_size);

        // ===================== 3. 放大图像，用于可视化调试 =====================
        cv::Mat roi_for_display;
        cv::resize(roi_for_model, roi_for_display, display_size, 0, 0, cv::INTER_NEAREST);
        cv::imshow("放大后的ROI", roi_for_display);

        // ===================== 4. 图像预处理：灰度化 =====================
        cv::Mat gray;
        cv::cvtColor(roi_for_model, gray, cv::COLOR_BGR2GRAY);

        cv::resize(gray,gray, display_size, 0, 0, cv::INTER_NEAREST);
        cv::imshow("gray", gray);

        // 构建神经网络输入blob（归一化到0~1）
        cv::Mat blob = cv::dnn::blobFromImage(gray, 1.0/255.0, model_input_size, cv::Scalar(0), false, false);

        // ===================== 5. 模型推理，识别数字 =====================
        net_.setInput(blob);
        cv::Mat prob = net_.forward();

        // 提取分类概率（忽略背景类，取1~最后一列对应数字0-9）
        cv::Mat cls_prob = prob.colRange(1, prob.cols);
        
        cv::Point classIdPoint;
        double confidence;
        // 获取最大概率对应的类别（数字）
        cv::minMaxLoc(cls_prob.reshape(1, 1), nullptr, &confidence, nullptr, &classIdPoint);

        // 赋值识别结果：classIdPoint.x 对应 0-9
        armor.number = classIdPoint.x; 
        armor.class_confidence = (float)confidence;
    }
}

// ===================== 核心检测主流程：从图像中识别装甲板并输出目标 =====================
/**
 * @brief 装甲板检测主函数
 * @param img 输入的相机图像
 * @return 检测结果（ROS2消息：是否检测到、角度、距离、坐标等）
 */
cpp08_armor_detector::msg::ArmorTarget ArmorDetector::detect(cv::Mat img)
{
    // 初始化输出消息，默认未检测到目标
    cpp08_armor_detector::msg::ArmorTarget target_msg;
    target_msg.is_detected = false;

    // ===================== 4.1 图像预处理：二值化、去噪，得到灯条掩码 =====================
    cv::Mat mask = preprocess(img);

    // ===================== 4.2 提取灯条：轮廓筛选、几何特征过滤 =====================
    std::vector<cv::RotatedRect> lightBars = extractLightBars(mask, 10);

    // ===================== 4.3 灯条配对 + 装甲板匹配 =====================
    std::vector<DetectedArmor> all_armors = matchLightBars(lightBars, img.size());

    // ===================== 4.3.1 PnP解算：2D像素点 → 3D空间位姿（距离、角度） =====================
    for (auto& armor : all_armors) {
        calculatePnP(armor, cameraMatrix, distCoeffs, objectPoints);
    }   

    // ===================== 4.4 装甲板去重：去除重复检测的同一目标 =====================
    std::vector<DetectedArmor> final_armors = deduplicateArmors(all_armors);

    // ===================== 数字识别：对最终装甲板进行0-9分类 =====================
    classifyArmors(img, final_armors);

    // ===================== 4.5 EKF 滤波：平滑目标角度、距离，抑制抖动 =====================
    // 图像中心点（用于计算目标偏离中心程度）
    cv::Point2f imgCenter(img.cols / 2.0f, img.rows / 2.0f);
    cv::RotatedRect bestArmorRotatedRect;  // 最优装甲板矩形
    std::vector<cv::Point2f> best_pnp_corners; // 最优装甲板角点
    double bestYaw = 0, bestPitch = 0, bestDist = 0;

    // 检测到装甲板
    if (!final_armors.empty()) {
        // ===================== 排序选择最优装甲板 =====================
        // 规则：置信度优先 → 图像中心距离优先
        std::sort(final_armors.begin(), final_armors.end(),
            [&imgCenter](const DetectedArmor& a, const DetectedArmor& b) {
                if (std::abs(a.confidence - b.confidence) > 0.05) {
                    return a.confidence > b.confidence;
                }
                return a.dist_to_center < b.dist_to_center;
            });

        // 取排序后第一个为最优目标
        const auto& best_armor = final_armors[0];

        // 赋值观测值：原始解算的yaw/pitch/距离
        bestYaw = best_armor.yaw;
        bestPitch = best_armor.pitch;
        bestDist = best_armor.distance;
        bestArmorRotatedRect = best_armor.rect;
        best_pnp_corners = best_armor.pnp_corners;
        
        // ===================== EKF 卡尔曼滤波核心逻辑 =====================
        // 角度转弧度
        double bestYaw_rad = best_armor.yaw * CV_PI / 180.0;

        if (!ekf_initialized) {
            // 首次检测到目标：初始化EKF
            ekf.init(best_armor.tvec, bestYaw_rad);
            ekf_initialized = true;
            found = true;
        } else {
            // 非首次：EKF预测 + 观测更新
            ekf.predict(1.0 / 50.0); // 50Hz控制周期
            ekf.update(best_armor.tvec, bestYaw_rad);
        }

        // 获取EKF滤波后的3D预测位置
        cv::Mat tvec_pred;
        double yaw_pred_rad;
        ekf.getPredictedArmor(tvec_pred, yaw_pred_rad);

        // ===================== 滤波后数据计算 =====================
        // 滤波后yaw角（角度制）
        double filtered_yaw = std::atan2(tvec_pred.at<double>(0), tvec_pred.at<double>(2)) * 180.0 / CV_PI;
        // 滤波后pitch角（角度制）
        double filtered_pitch = std::atan2(-tvec_pred.at<double>(1), tvec_pred.at<double>(2)) * 180.0 / CV_PI;
        // 滤波后目标距离
        double filtered_dist = cv::norm(tvec_pred);
        // 原始PnP解算角度（未滤波）
        double raw_yaw = std::atan2(best_armor.tvec.at<double>(0), best_armor.tvec.at<double>(2)) * 180.0 / CV_PI;
        double raw_pitch = std::atan2(-best_armor.tvec.at<double>(1), best_armor.tvec.at<double>(2)) * 180.0 / CV_PI;


        // ===================== 计算最终云台控制角度 =====================
        // 目标yaw = 云台当前yaw + 滤波后偏差 + 手动偏移量
        target_yaw_ = gimbal_yaw_current_ + filtered_yaw + YAW_OFFSET;
        // 目标pitch = 云台当前pitch + 滤波后偏差 + 手动偏移量
        target_pitch_ = gimbal_pitch_current_ + filtered_pitch + PITCH_OFFSET;

        // Yaw角归一化到 [-180, 180] 范围
        while (target_yaw_ > 180.0f)  target_yaw_ -= 360.0f;
        while (target_yaw_ < -180.0f) target_yaw_ += 360.0f;

        // ===================== 填充ROS2消息 =====================
        target_msg.is_detected = true;
        target_msg.yaw = static_cast<float>(raw_yaw);          // 原始yaw
        target_msg.pitch = static_cast<float>(raw_pitch);      // 原始pitch
        target_msg.distance = static_cast<float>(filtered_dist); // 滤波后距离
        target_msg.filtered_yaw = static_cast<float>(filtered_yaw); // 滤波后yaw
        target_msg.center_x = static_cast<float>(bestArmorRotatedRect.center.x); // 目标中心x
        target_msg.center_y = static_cast<float>(bestArmorRotatedRect.center.y); // 目标中心y

        // ===================== 可视化绘制：所有最终装甲板（绿色） =====================
        for (auto &armor : final_armors) {
            // 绘制装甲板矩形框
            cv::Point2f armor_corners[4];
            armor.rect.points(armor_corners);
            for (int k = 0; k < 4; k++) {
                cv::line(img, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(0, 255, 0), 2);
            }
            // 绘制PnP角点（红色）
            for (auto &pt : armor.pnp_corners) {
                cv::circle(img, pt, 4, cv::Scalar(0, 0, 255), -1);
            }
            // 标注角点坐标
            for (auto &pt : best_pnp_corners)
            {
                cv::putText(img, "(" + std::to_string((int)pt.x) + "," + std::to_string((int)pt.y) + ")", pt + cv::Point2f(5, -5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
            // 标注识别的数字 + 置信度
            if (armor.number != -1) {
            std::string text = "Num:" + std::to_string(armor.number) + " (" + std::to_string((int)(armor.class_confidence*100)) + "%)";
            cv::putText(img, text, armor.pnp_corners[0] - cv::Point2f(0, 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
            // 标注装甲板检测置信度
            cv::putText(img, "Conf:" + std::to_string((int)(armor.confidence*100)) + "%", 
                        armor.rect.center + cv::Point2f(-20, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1);
        }

        // ===================== 绘制最优装甲板（红色加粗，突出显示） =====================
        cv::Point2f final_armor_corners[4];
        bestArmorRotatedRect.points(final_armor_corners);
        for (int k = 0; k < 4; k++) {
            cv::line(img, final_armor_corners[k], final_armor_corners[(k+1)%4], cv::Scalar(0, 0, 255), 3);
        }

        // ===================== 调试文字绘制：原始值 & 滤波后值 =====================
        rawYawList.push_back((float)bestYaw);
        filteredYawList.push_back((float)filtered_yaw);
        // 限制列表长度，防止内存溢出
        if (rawYawList.size() > 400) rawYawList.pop_front();
        if (filteredYawList.size() > 400) filteredYawList.pop_front();

        // 在装甲板上方显示角度、距离信息
        cv::Point2f text_pos = bestArmorRotatedRect.center;
        text_pos.y -= bestArmorRotatedRect.size.height / 2 + 20;
        char text_buf[256];
        sprintf(text_buf, "Raw: Y=%f P=%f D=%f| EKF: Y=%f P=%f", 
                bestYaw, bestPitch, bestDist, filtered_yaw, filtered_pitch);
        cv::putText(img, text_buf, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        
        // ===================== 机器人几何中心拟合：多帧装甲板法向量拟合中心 =====================
        CenterFit fit;
        fit.position = cv::Point3f(
            best_armor.tvec.at<double>(0, 0),
            best_armor.tvec.at<double>(1, 0),
            best_armor.tvec.at<double>(2, 0));
        // 旋转向量转旋转矩阵
        cv::Mat rot_mat;
        cv::Rodrigues(best_armor.rvec, rot_mat);
        // 计算装甲板法向量（正面朝向）
        cv::Point3f normal_vector(
            rot_mat.at<double>(0, 2),
            rot_mat.at<double>(1, 2),
            rot_mat.at<double>(2, 2)
        );
        fit.normalvector = normal_vector;

        // 加入历史列表
        center_fits.push_back(fit);
        // 限制列表最大长度50
        if (center_fits.size() > 50) center_fits.erase(center_fits.begin());

        // 拟合机器人中心并绘制
        find_robot_center();
        if (find_center && !center_2d.empty()) {
            cv::circle(img, center_2d[0], 8, cv::Scalar(0, 255, 255), -1);
            cv::putText(img, "Geo Center", center_2d[0] + cv::Point2f(10, 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }
    }
    else
    {
        // ===================== 未检测到目标：重置状态 =====================
        found = false;
        ekf_initialized = false;
        center_fits.clear(); // 清空几何中心拟合历史
    }

    // 绘制yaw角滤波对比曲线
    drawYawPlot();
    // 返回检测结果
    return target_msg;
}
