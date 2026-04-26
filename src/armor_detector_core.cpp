#include "cpp08_armor_detector/armor_detector.hpp"
#include "cpp08_armor_detector/uart_protocol.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include "cpp08_armor_detector/armor_detector_matching.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 用当前PnP结果重投影其他3块装甲板（可视化四块装甲板）
 * @param img 原图
 * @param rvec 当前装甲板的旋转向量
 * @param tvec 当前装甲板的平移向量
 */
void ArmorDetector::drawFourArmorsFromOne(cv::Mat& img, const cv::Mat& rvec, const cv::Mat& tvec) {
    // 获取机器人几何参数（装甲板宽、高、装甲板到中心的距离）
    const double ARMOR_W = robot_geom_.armor_width;
    const double ARMOR_H = robot_geom_.armor_height;
    const double CAR_RADIUS = robot_geom_.armor_plate_distance;

    // 将旋转向量转换为旋转矩阵
    cv::Mat R_cur;
    cv::Rodrigues(rvec, R_cur);
    // 计算当前装甲板的yaw角（绕Y轴旋转）
    double yaw = std::atan2(R_cur.at<double>(0, 2), R_cur.at<double>(2, 2));
    if (R_cur.at<double>(2, 2) < 0) {
        yaw += CV_PI; 
    }

    // 计算机器人中心在相机坐标系下的坐标
    double car_center_cam_x = tvec.at<double>(0) + CAR_RADIUS * std::sin(yaw);
    double car_center_cam_y = tvec.at<double>(1);
    double car_center_cam_z = tvec.at<double>(2) + CAR_RADIUS * std::cos(yaw);

    // 定义四块装甲板的绘制颜色
    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),   // 绿色
        cv::Scalar(255, 0, 0),   // 蓝色
        cv::Scalar(0, 0, 255),   // 红色
        cv::Scalar(255, 255, 0)  // 黄色
    };

    // 重投影时使用的零旋转和零平移（因为3D点已经在相机坐标系下）
    cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);

    // 循环生成四块装甲板的3D点并投影到2D图像
    for (int i = 0; i < 4; ++i) {
        // 计算当前装甲板的yaw角（每隔90度一块）
        double current_yaw = yaw + i * (CV_PI / 2.0);
        // 计算当前装甲板中心在相机坐标系下的坐标
        double cx = car_center_cam_x + CAR_RADIUS * std::sin(current_yaw);
        double cy = car_center_cam_y;
        double cz = car_center_cam_z + CAR_RADIUS * std::cos(current_yaw);

        // 计算当前装甲板的四个角点（3D）
        std::vector<cv::Point3d> corners_cam;
        double cos_y = std::cos(current_yaw);
        double sin_y = std::sin(current_yaw);
        // 装甲板宽度方向向量（在相机坐标系下）
        cv::Point3d w_vec(cos_y * (ARMOR_W/2.0), 0, -sin_y * (ARMOR_W/2.0));
        // 装甲板高度方向向量（在相机坐标系下）
        cv::Point3d h_vec(0, ARMOR_H/2.0, 0);

        // 组装四个角点
        corners_cam.push_back(cv::Point3d(cx, cy, cz) - w_vec - h_vec);
        corners_cam.push_back(cv::Point3d(cx, cy, cz) + w_vec - h_vec);
        corners_cam.push_back(cv::Point3d(cx, cy, cz) + w_vec + h_vec);
        corners_cam.push_back(cv::Point3d(cx, cy, cz) - w_vec + h_vec);

        // 将3D角点投影到2D图像
        std::vector<cv::Point2d> img_pts;
        cv::projectPoints(corners_cam, rvec_zero, tvec_zero,
                        cameraMatrix, distCoeffs, img_pts);

        // 绘制装甲板边框和标签
        if (img_pts.size() == 4) {
            for (int j = 0; j < 4; ++j) {
                cv::line(img, img_pts[j], img_pts[(j+1)%4], colors[i], 2);
            }
            cv::putText(img, "Plate " + std::to_string(i), img_pts[1] + cv::Point2d(0, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 2);
        }
    }

    // 绘制机器人中心（在图像上）
    std::vector<cv::Point2d> center_2d;
    cv::projectPoints(std::vector<cv::Point3d>{cv::Point3d(car_center_cam_x, car_center_cam_y, car_center_cam_z)},
                    rvec_zero, tvec_zero, cameraMatrix, distCoeffs, center_2d);
    if (!center_2d.empty()) {
        cv::circle(img, center_2d[0], 8, cv::Scalar(255, 255, 255), -1);
        cv::circle(img, center_2d[0], 8, cv::Scalar(0, 0, 0), 2);
    }
}

/**
 * @brief 设置云台当前角度
 * @param yaw_current 云台当前yaw角
 * @param pitch_current 云台当前pitch角
 */
void ArmorDetector::setGimbalCurrent(float yaw_current, float pitch_current) {
    gimbal_yaw_current_ = yaw_current;
    gimbal_pitch_current_ = pitch_current;
}

/**
 * @brief 加载深度学习模型（ONNX格式）
 * @param model_path 模型文件路径
 */
void ArmorDetector::loadModel(const std::string& model_path) {
    net_ = cv::dnn::readNetFromONNX(model_path);
}

/**
 * @brief 对检测到的装甲板进行数字分类
 * @param src 原图
 * @param armors 检测到的装甲板列表（会修改其中的number和class_confidence）
 */
void ArmorDetector::classifyArmors(const cv::Mat& src, std::vector<DetectedArmor>& armors) {
    if (net_.empty() || armors.empty()) return;

    // 模型输入尺寸
    const cv::Size model_input_size(28, 28); 
    const cv::Size display_size(300, 300);
    // 垂直方向padding比例（防止数字被截断）
    const float vertical_padding_ratio = 0.6f;

    for (auto& armor : armors) {
        std::vector<cv::Point2f> roi_corners = armor.pnp_corners;
        // 计算装甲板平均高度
        float left_height = cv::norm(roi_corners[0] - roi_corners[1]);
        float right_height = cv::norm(roi_corners[3] - roi_corners[2]);
        float avg_height = (left_height + right_height) / 2.0f;
        // 计算垂直方向偏移量
        float y_offset = avg_height * vertical_padding_ratio;

        // 扩展ROI区域（上下各扩展y_offset）
        roi_corners[0].y -= y_offset;
        roi_corners[3].y -= y_offset;
        roi_corners[1].y += y_offset;
        roi_corners[2].y += y_offset;

        // 透视变换目标点（模型输入尺寸）
        std::vector<cv::Point2f> dst_pts = {
            {0, 0},
            {0, (float)model_input_size.height},
            {(float)model_input_size.width, (float)model_input_size.height},
            {(float)model_input_size.width, 0}
        };
        // 计算透视变换矩阵
        cv::Mat warp_mat = cv::getPerspectiveTransform(roi_corners, dst_pts);
        // 执行透视变换，得到模型输入ROI
        cv::Mat roi_for_model;
        cv::warpPerspective(src, roi_for_model, warp_mat, model_input_size);

        // 转换为灰度图
        cv::Mat gray;
        cv::cvtColor(roi_for_model, gray, cv::COLOR_BGR2GRAY);
        // 归一化并转换为模型输入blob
        cv::Mat blob = cv::dnn::blobFromImage(gray, 1.0/255.0, model_input_size, cv::Scalar(0), false, false);

        // 模型推理
        net_.setInput(blob);
        cv::Mat prob = net_.forward();
        // 提取分类概率（排除背景类）
        cv::Mat cls_prob = prob.colRange(1, prob.cols);
        
        // 找到最大概率对应的类别
        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(cls_prob.reshape(1, 1), nullptr, &confidence, nullptr, &classIdPoint);

        // 保存分类结果
        armor.number = classIdPoint.x; 
        armor.class_confidence = (float)confidence;
    }
}

/**
 * @brief 主检测函数：处理图像，检测装甲板，计算位姿，预测目标
 * @param img 输入图像
 * @return 装甲板目标消息（包含位姿、距离等信息）
 */
cpp08_armor_detector::msg::ArmorTarget ArmorDetector::detect(cv::Mat img,rclcpp::Time timestamp)
{
    cpp08_armor_detector::msg::ArmorTarget target_msg;
    target_msg.is_detected = false; // 默认未检测到

    // 1. 图像预处理（颜色分割等）
    cv::Mat mask = preprocess(img);
    // 2. 提取灯条
    std::vector<cv::RotatedRect> lightBars = extractLightBars(mask, 10);
    // 3. 匹配灯条对，得到候选装甲板
    std::vector<DetectedArmor> all_armors = matchLightBars(lightBars, img.size());

    // 4. 对每个候选装甲板进行PnP位姿解算
    for (auto& armor : all_armors) {
        calculatePnP(armor, cameraMatrix, distCoeffs, objectPoints);
    }   

    // 5. 装甲板去重（防止重复检测同一装甲板）
    std::vector<DetectedArmor> final_armors = deduplicateArmors(all_armors);
    // 6. 对去重后的装甲板进行数字分类
    classifyArmors(img, final_armors);

    // 图像中心
    cv::Point2f imgCenter(img.cols / 2.0f, img.rows / 2.0f);
    cv::RotatedRect bestArmorRotatedRect;
    std::vector<cv::Point2f> best_pnp_corners;
//RCLCPP_INFO(rclcpp::get_logger("ArmorDetector"), "检测到 %zu 块装甲板", final_armors.size());
    if (!final_armors.empty()) {
        // 7. 选择最佳装甲板（优先置信度，其次距离图像中心的距离）
        std::sort(final_armors.begin(), final_armors.end(),
            [&imgCenter](const DetectedArmor& a, const DetectedArmor& b) {
                if (std::abs(a.confidence - b.confidence) > 0.05) {
                    return a.confidence > b.confidence;
                }
                return a.dist_to_center < b.dist_to_center;
            });

        const auto& best_armor = final_armors[0];
        bestArmorRotatedRect = best_armor.rect;
        best_pnp_corners = best_armor.pnp_corners;
        // 8. 可视化四块装甲板（基于当前最佳装甲板重投影）
        drawFourArmorsFromOne(img, best_armor.rvec, best_armor.tvec);
        //RCLCPP_INFO(rclcpp::get_logger("ArmorDetector"), "检测到装甲板: 置信度=%.2f, 距离中心=%.1f",
                    //best_armor.confidence, best_armor.dist_to_center);

        static const cv::Mat P_cv2ros = (cv::Mat_<double>(3, 3) << 
             0,  0,  1, 
            -1,  0,  0, 
             0, -1,  0
        );            
        // ==================== 1. 定义 OpenCV 到 ROS 的转换矩阵 P_cv2ros ====================
        static const cv::Mat M_obj2flu = (cv::Mat_<double>(3, 3) << 
             0,  0, -1, 
            -1,  0,  0, 
             0, -1,  0
        );
        // ==================== 2. 提取原始的 OpenCV 位姿 ====================
        cv::Mat R_cv;
        cv::Rodrigues(best_armor.rvec, R_cv);
        cv::Mat t_cv = best_armor.tvec.clone(); // 单位是毫米

        // ==================== 3. 施加数学魔法：相似变换 ====================
        // 平移向量转换：t_ros = P * t_cv
        cv::Mat t_ros = P_cv2ros * t_cv;
        
        // 旋转矩阵转换：R_ros = P * R_cv * P^T 
        cv::Mat R_ros = P_cv2ros * R_cv * M_obj2flu.t(); // 注意这里是 M_obj2flu 的转置
        // ==================== 4. 替换原有的 RViz TF 广播 ====================
        if (tf_armor_broadcaster_) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp = timestamp;
            ts.header.frame_id = "camera_frame";
            ts.child_frame_id = "armor_link";

            // 直接赋值，再也不用手动调换 X/Y/Z 顺位了！F
            ts.transform.translation.x = t_ros.at<double>(0) / 1000.0;
            ts.transform.translation.y = t_ros.at<double>(1) / 1000.0;
            ts.transform.translation.z = t_ros.at<double>(2) / 1000.0;


            // 直接将 R_ros 塞给 tf2，再也不用手动配凑 9 个元素的正负号了！
            tf2::Matrix3x3 tf2_R(
                R_ros.at<double>(0, 0), R_ros.at<double>(0, 1), R_ros.at<double>(0, 2),
                R_ros.at<double>(1, 0), R_ros.at<double>(1, 1), R_ros.at<double>(1, 2),
                R_ros.at<double>(2, 0), R_ros.at<double>(2, 1), R_ros.at<double>(2, 2)
            );
            tf2::Quaternion q;
            tf2_R.getRotation(q);
            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();

            tf_armor_broadcaster_->sendTransform(ts);
        }// ==================== 5. 使用 PoseStamped 完美转换位置与姿态 ====================
        geometry_msgs::msg::PoseStamped pose_cam;
        pose_cam.header.frame_id = "camera_frame";
        pose_cam.header.stamp = timestamp; 
        
        // 5.1 填入位置 (平移)
        pose_cam.pose.position.x = t_ros.at<double>(0) / 1000.0;
        pose_cam.pose.position.y = t_ros.at<double>(1) / 1000.0;
        pose_cam.pose.position.z = t_ros.at<double>(2) / 1000.0;

        // 5.2 填入姿态 (旋转)
        // 使用我们上一轮适配好法向量的 tf2_R
        tf2::Matrix3x3 tf2_R_cam(
            R_ros.at<double>(0, 0), R_ros.at<double>(0, 1), R_ros.at<double>(0, 2),
            R_ros.at<double>(1, 0), R_ros.at<double>(1, 1), R_ros.at<double>(1, 2),
            R_ros.at<double>(2, 0), R_ros.at<double>(2, 1), R_ros.at<double>(2, 2)
        );
        tf2::Quaternion q_cam;
        tf2_R_cam.getRotation(q_cam);
        pose_cam.pose.orientation.x = q_cam.x();
        pose_cam.pose.orientation.y = q_cam.y();
        pose_cam.pose.orientation.z = q_cam.z();
        pose_cam.pose.orientation.w = q_cam.w();

        // 5.3 严格的三维空间 TF 变换 (这一步底层就是 R_world_camera * R_camera_armor)
        geometry_msgs::msg::PoseStamped pose_world;
        try {
            pose_world = tf_buffer_->transform(pose_cam, "world_frame");
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(rclcpp::get_logger("ArmorDetector"), "世界坐标转换失败: %s", ex.what());
            pose_world = pose_cam; // 降级
        }

        // ==================== 6. 提取纯正的 World Frame 数据喂给 EKF ====================
        
        // 6.1 提取世界系毫米级坐标
        cv::Mat tvec_world = (cv::Mat_<double>(3,1) <<
            pose_world.pose.position.x * 1000.0,
            pose_world.pose.position.y * 1000.0,
            pose_world.pose.position.z * 1000.0);

        // 6.2 从变换后的世界系四元数中，提取绝对 Yaw 角
        tf2::Quaternion q_world(
            pose_world.pose.orientation.x,
            pose_world.pose.orientation.y,
            pose_world.pose.orientation.z,
            pose_world.pose.orientation.w
        );
        tf2::Matrix3x3 m_world(q_world);
        double roll_w, pitch_w, yaw_w;
        m_world.getRPY(roll_w, pitch_w, yaw_w); 

        double ray_angle = std::atan2(pose_world.pose.position.y, pose_world.pose.position.x);
        double expected_yaw = ray_angle + CV_PI; 
        ArmorEKF::normalizeYaw(expected_yaw);

        // 3. 计算 PnP 解算的 Yaw 与期望 Yaw 的偏差
        double angle_diff = yaw_w - expected_yaw;
        ArmorEKF::normalizeYaw(angle_diff);

        // 4. 如果偏差超过 90 度，说明 PnP 发生了致命的 180 度翻转
        if (std::abs(angle_diff) > CV_PI / 2.0) {
            yaw_w += CV_PI; // 强行翻转 180 度，纠正 OpenCV 的多解错误！
            ArmorEKF::normalizeYaw(yaw_w);
        }
        
        double bestYaw_rad_world = yaw_w;
        // 10. EKF初始化或预测更新
        double current_timestamp = (double)cv::getTickCount() / cv::getTickFrequency();
        if (!ekf_initialized) {
            // 首次检测到装甲板，初始化EKF
            ekf.init(tvec_world, bestYaw_rad_world, current_timestamp);
            ekf_initialized = true;
            found = true;
        } else {
            // 非首次，先预测，再更新
            ekf.predict(current_timestamp);
            ekf.update(tvec_world, bestYaw_rad_world);
        }   

        // 11. 获取EKF预测的装甲板位姿（世界坐标系）
        cv::Mat tvec_pred_world;
        double yaw_pred_rad_world;
        ekf.getPredictedArmor(tvec_pred_world, yaw_pred_rad_world);

        // 转换为米
        double Xw = tvec_pred_world.at<double>(0) / 1000.0;
        double Yw = tvec_pred_world.at<double>(1) / 1000.0;
        double Zw = tvec_pred_world.at<double>(2) / 1000.0;

        // 12. 计算预测点对应的云台角度（Yaw和Pitch）
        // Yaw：绕Z轴旋转，看X-Y水平面 (X前, Y左)
        double yaw_rad = std::atan2(Yw, Xw);
        // Pitch：绕Y轴旋转，看高度-水平距离剖面 (Z上)
        double horizontal_dist = std::sqrt(Xw*Xw + Yw*Yw);
        double pitch_rad = std::atan2(Zw, horizontal_dist);

        // 保存不带偏移的原始角度（用于filtered_yaw显示）
        double target_yaw_abs = yaw_rad * 180.0 / CV_PI;
        double target_pitch_abs = pitch_rad * 180.0 / CV_PI;

        // 加上偏移量，转换为发给电控的最终角度
        target_yaw_   = target_yaw_abs + YAW_OFFSET;
        target_yaw_ =-target_yaw_; // 方向调整（根据实际情况）
        target_pitch_ = target_pitch_abs + PITCH_OFFSET;

        // 角度归一化到[-180, 180]
        while (target_yaw_ > 180.0f)  target_yaw_ -= 360.0f;
        while (target_yaw_ < -180.0f) target_yaw_ += 360.0f;

        // 保存滤波后的角度、距离等信息
        double filtered_yaw   = target_yaw_abs;
        double filtered_pitch = target_pitch_abs;
        double filtered_dist  = cv::norm(tvec_pred_world);
        // ==================== 新增：广播 EKF 预测点到 RViz ====================
        if (tf_armor_broadcaster_) {
            geometry_msgs::msg::TransformStamped ts_pred;
            ts_pred.header.stamp = timestamp;
            ts_pred.header.frame_id = "world_frame";    // 预测点是在世界坐标系下的
            ts_pred.child_frame_id = "armor_pred_link"; // 预测装甲板坐标系

            ts_pred.transform.translation.x = Xw;
            ts_pred.transform.translation.y = Yw;
            ts_pred.transform.translation.z = Zw;

            // 预测点我们只看位置，旋转设为单位阵即可
            ts_pred.transform.rotation.x = 0.0;
            ts_pred.transform.rotation.y = 0.0;
            ts_pred.transform.rotation.z = 0.0;
            ts_pred.transform.rotation.w = 1.0;

            tf_armor_broadcaster_->sendTransform(ts_pred);
        }
        // 计算原始角度（未滤波）
        double raw_yaw = std::atan2(-best_armor.tvec.at<double>(0), best_armor.tvec.at<double>(2)) * 180.0 / CV_PI;
        double raw_pitch = std::atan2(-best_armor.tvec.at<double>(1), best_armor.tvec.at<double>(2)) * 180.0 / CV_PI;

        // 13. 将预测点和机器人中心从世界坐标系转换回相机坐标系（用于可视化）
        geometry_msgs::msg::PointStamped pred_world_pt, pred_cam_pt;
        pred_world_pt.header.frame_id = "world_frame";
        pred_world_pt.header.stamp = timestamp;
        pred_world_pt.point.x = Xw;
        pred_world_pt.point.y = Yw;
        pred_world_pt.point.z = Zw;

        geometry_msgs::msg::PointStamped center_world_pt, center_cam_pt;
        center_world_pt.header.frame_id = "world_frame";
        center_world_pt.header.stamp = timestamp;
        center_world_pt.point.x = ekf.x.at<double>(0, 0) / 1000.0;
        center_world_pt.point.y = ekf.x.at<double>(2, 0) / 1000.0;
        center_world_pt.point.z = ekf.x.at<double>(4, 0) / 1000.0;

        try {
            // TF转换：世界系 -> 相机系
            pred_cam_pt = tf_buffer_->transform(pred_world_pt, "camera_frame");
            center_cam_pt = tf_buffer_->transform(center_world_pt, "camera_frame");

         
        } catch (const tf2::TransformException& ex) {
           // RCLCPP_WARN(rclcpp::get_logger("ArmorDetector"), "反向TF转换失败: %s", ex.what());
            pred_cam_pt = pred_world_pt;
            center_cam_pt = center_world_pt;
        }
            cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);   
            
            // 投影预测装甲板点到图像并绘制
        std::vector<cv::Point3f> pred_armor_3d;
        // 【关键修复】：将 ROS 坐标 (前左上) 反向映射回 OpenCV 坐标 (右下前)
        // OpenCV_X(右) = -ROS_Y(左)
        // OpenCV_Y(下) = -ROS_Z(上)
        // OpenCV_Z(前) = ROS_X(前)
        pred_armor_3d.push_back(cv::Point3f(
            -pred_cam_pt.point.y * 1000.0, 
            -pred_cam_pt.point.z * 1000.0, 
             pred_cam_pt.point.x * 1000.0
        ));
        
        std::vector<cv::Point2f> pred_armor_2d;
        cv::projectPoints(pred_armor_3d, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, pred_armor_2d);
        
        if (!pred_armor_2d.empty()) {
            cv::circle(img, pred_armor_2d[0], 6, cv::Scalar(0, 255, 255), -1);
            cv::putText(img, "Pred Armor", pred_armor_2d[0] + cv::Point2f(10, 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }

        // 2. 投影机器人中心点
        std::vector<cv::Point3f> robot_center_3d;
        robot_center_3d.push_back(cv::Point3f(
            -center_cam_pt.point.y * 1000.0, 
            -center_cam_pt.point.z * 1000.0, 
             center_cam_pt.point.x * 1000.0
        ));
        
        std::vector<cv::Point2f> robot_center_2d;
        cv::projectPoints(robot_center_3d, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, robot_center_2d);
        
        if (!robot_center_2d.empty()) {
            cv::circle(img, robot_center_2d[0], 8, cv::Scalar(255, 255, 255), -1);
            cv::circle(img, robot_center_2d[0], 8, cv::Scalar(0, 0, 0), 2);
            cv::putText(img, "Robot Center", robot_center_2d[0] + cv::Point2f(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);   
        }
        // 14. 填充目标消息
        target_msg.is_detected = true;
        target_msg.yaw = static_cast<float>(raw_yaw);
        target_msg.pitch = static_cast<float>(raw_pitch);
        target_msg.distance = static_cast<float>(filtered_dist);
        target_msg.filtered_yaw = static_cast<float>(filtered_yaw);
        target_msg.center_x = static_cast<float>(bestArmorRotatedRect.center.x);
        target_msg.center_y = static_cast<float>(bestArmorRotatedRect.center.y);

        // 15. 可视化所有检测到的装甲板
        for (auto &armor : final_armors) {
            cv::Point2f armor_corners[4];
            armor.rect.points(armor_corners);
            // 绘制装甲板边框
            for (int k = 0; k < 4; k++) {
                cv::line(img, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(0, 255, 0), 2);
            }
            // 绘制PnP角点
            for (auto &pt : armor.pnp_corners) {
                cv::circle(img, pt, 4, cv::Scalar(0, 0, 255), -1);
            }
            // 绘制最佳装甲板的角点坐标
            for (auto &pt : best_pnp_corners) {
                cv::putText(img, "(" + std::to_string((int)pt.x) + "," + std::to_string((int)pt.y) + ")", pt + cv::Point2f(5, -5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
            // 绘制分类结果
            if (armor.number != -1) {
                std::string text = "Num:" + std::to_string(armor.number) + " (" + std::to_string((int)(armor.class_confidence*100)) + "%)";
                cv::putText(img, text, armor.pnp_corners[0] - cv::Point2f(0, 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
            // 绘制置信度
            cv::putText(img, "Conf:" + std::to_string((int)(armor.confidence*100)) + "%", 
                        armor.rect.center + cv::Point2f(-20, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1);
        }

        // 16. 用粗线绘制最佳装甲板
        cv::Point2f final_armor_corners[4];
        bestArmorRotatedRect.points(final_armor_corners);
        for (int k = 0; k < 4; k++) {
            cv::line(img, final_armor_corners[k], final_armor_corners[(k+1)%4], cv::Scalar(0, 0, 255), 3);
        }

        // 17. 保存世界原始yaw和滤波yaw数据（用于绘制曲线）
        double raw_yaw_world = std::atan2(pose_world.pose.position.y, pose_world.pose.position.x) * 180.0 / CV_PI;
        double raw_pitch_world = std::atan2(pose_world.pose.position.z, std::sqrt(pose_world.pose.position.x*pose_world.pose.position.x + pose_world.pose.position.y*pose_world.pose.position.y)) * 180.0 / CV_PI;
        rawYawList.push_back((float)raw_yaw_world);     // 统一推入世界系原始数据
        filteredYawList.push_back((float)filtered_yaw); // 世界系滤波数据
        
        if (rawYawList.size() > 400) rawYawList.pop_front();
        if (filteredYawList.size() > 400) filteredYawList.pop_front();

        // 18. 绘制位姿信息文本
        cv::Point2f text_pos = bestArmorRotatedRect.center;
        text_pos.y -= bestArmorRotatedRect.size.height / 2 + 20;
        char text_buf[256];
        // 屏幕上的文字也统一显示世界系的角度，方便肉眼对比
        sprintf(text_buf, "Raw World: Y=%.5f P=%.5f | EKF: Y=%.5f P=%.5f", 
                raw_yaw_world, raw_pitch_world, filtered_yaw, filtered_pitch);
        cv::putText(img, text_buf, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
    } else { 
        // 未检测到装甲板，重置状态
        found = false;
        ekf_initialized = false;
        center_fits.clear(); 
    }

    // 19. 绘制yaw滤波曲线
    drawYawPlot();
    return target_msg;
}
