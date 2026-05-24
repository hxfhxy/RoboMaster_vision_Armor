#include "cpp08_armor_detector/armor_detector.hpp"
#include "cpp08_armor_detector/uart_protocol.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include "cpp08_armor_detector/armor_detector_matching.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief   重投影 (在世界坐标系下建好3D装甲板，通过TF树投影)
 */
void ArmorDetector::drawFourArmorsFromWorld(cv::Mat& img, double xc_w, double yc_w, double zc_w, double yaw_w, 
                                            double r1, double r2, double dz, rclcpp::Time timestamp) {
    const double W = robot_geom_.armor_width / 1000.0 / 2.0;   
    const double H = robot_geom_.armor_height / 1000.0 / 2.0;  

    cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);

    for (int i = 0; i < 4; ++i) {
        double a = yaw_w + i * (CV_PI / 2.0);
        
        // 区分侧板和前后板的半径和高度差
        bool is_side = (i == 1 || i == 3);
        double R = is_side ? r2 : r1;
        double Z = is_side ? zc_w + dz : zc_w;

        double cx = xc_w - R * std::cos(a);
        double cy = yc_w - R * std::sin(a);
        double cz = Z;
        
        double tilt_angle = 15.0 * CV_PI / 180.0; // 根据你们车体的实际机械后倾角调整（通常为 15 度）

        // a 是装甲板法线的绝对 Yaw 角。向内倾斜的方向与法线相反。
        cv::Point3d h_vec(
            -std::cos(a) * std::sin(tilt_angle) * H, // X 轴向内收缩
            -std::sin(a) * std::sin(tilt_angle) * H, // Y 轴向内收缩
            -std::cos(tilt_angle) * H                // 实际的 Z 轴高度会有所降低
        );
        cv::Point3d w_vec(-std::sin(a) * W, std::cos(a) * W, 0); 
        
        
        // 3. 组装 4 个角点 
        std::vector<cv::Point3d> corners_w = {
            cv::Point3d(cx, cy, cz) + w_vec + h_vec, // 左上
            cv::Point3d(cx, cy, cz) - w_vec + h_vec, // 右上
            cv::Point3d(cx, cy, cz) - w_vec - h_vec, // 右下
            cv::Point3d(cx, cy, cz) + w_vec - h_vec  // 左下
        };
        
        std::vector<cv::Point3f> corners_cv;
        bool tf_ok = true;
        
        // 4. 通过 ROS 的 TF 树，把这 4 个世界坐标系的点，转回当前的相机坐标系
        for (int j = 0; j < 4; ++j) {
            geometry_msgs::msg::PointStamped pt_w, pt_c;
            pt_w.header.frame_id = "world_frame";
            pt_w.header.stamp = timestamp;
            pt_w.point.x = corners_w[j].x;
            pt_w.point.y = corners_w[j].y;
            pt_w.point.z = corners_w[j].z;
            
            try {
                pt_c = tf_buffer_->transform(pt_w, "camera_frame");
            } catch (...) {
                pt_c = pt_w; 
            }
            
            // 将 ROS 坐标 (前左上) 反向映射回 OpenCV 坐标 (右下前)，乘以 1000 转回毫米投影
            corners_cv.push_back(cv::Point3f(
                -pt_c.point.y * 1000.0,
                -pt_c.point.z * 1000.0,
                 pt_c.point.x * 1000.0
            ));
        }
        
        //if (!tf_ok) continue;
        
        // 5. 如果有任何一个点在相机后面（z<0），就不画这个装甲板了
        bool is_behind = false;
        for (auto& pt : corners_cv) {
            if (pt.z < 10.0) is_behind = true;
        }
        if (is_behind) continue;

        // 6. 把这 4 个点投影到图像平面上，并连接成装甲板的四条边
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(corners_cv, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, img_pts);

        if (img_pts.size() == 4) {
            for (int j = 0; j < 4; ++j) {
                cv::line(img, img_pts[j], img_pts[(j+1)%4], cv::Scalar(255, 0, 0), 2);
            }
             cv::putText(img, std::to_string(i), img_pts[1] + cv::Point2f(0, -10),
                         cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }
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

cpp08_armor_detector::msg::ArmorTarget ArmorDetector::detect(cv::Mat img, rclcpp::Time timestamp)
{
    cpp08_armor_detector::msg::ArmorTarget target_msg;
    target_msg.is_detected = false; 

    const int INPUT_WIDTH = 640;
    const int INPUT_HEIGHT = 640;
    
    // 生成 blob (swapRB=true，YOLO通常需要BGR转RGB)
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);

    std::vector<cv::Mat> net_outputs;
    net_.forward(net_outputs, net_.getUnconnectedOutLayersNames());

    // ==================== 2. 解析 YOLO 输出 ====================
    cv::Mat output = net_outputs[0];
    cv::Mat output_buffer(output.size[1], output.size[2], CV_32F, output.ptr<float>());

    float conf_threshold = 0.65f;
    float nms_threshold = 0.45f;
    int detect_color = 0; // 0: 找蓝色, 1: 找红色

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<DetectedArmor> temp_armors;

    float x_factor = img.cols / (float)INPUT_WIDTH;
    float y_factor = img.rows / (float)INPUT_HEIGHT;

    for (int i = 0; i < output_buffer.rows; i++) {
        float confidence = output_buffer.at<float>(i, 8);
        confidence = 1.0f / (1.0f + std::exp(-confidence)); // sigmoid 激活

        if (confidence < conf_threshold) continue;

        cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);
        cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22);
        
        cv::Point class_id, color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);

        if (color_id.x == 2 || color_id.x == 3) continue;
        if (detect_color == 0 && color_id.x == 1) continue; 
        if (detect_color == 1 && color_id.x == 0) continue; 

        // 提取并还原 4 个角点坐标
        cv::Point2f p1_lt(output_buffer.at<float>(i, 0) * x_factor, output_buffer.at<float>(i, 1) * y_factor);
        cv::Point2f p2_lb(output_buffer.at<float>(i, 2) * x_factor, output_buffer.at<float>(i, 3) * y_factor); 
        cv::Point2f p3_rb(output_buffer.at<float>(i, 4) * x_factor, output_buffer.at<float>(i, 5) * y_factor); 
        cv::Point2f p4_rt(output_buffer.at<float>(i, 6) * x_factor, output_buffer.at<float>(i, 7) * y_factor); 

        float min_x = std::min({p1_lt.x, p2_lb.x, p3_rb.x, p4_rt.x});
        float max_x = std::max({p1_lt.x, p2_lb.x, p3_rb.x, p4_rt.x});
        float min_y = std::min({p1_lt.y, p2_lb.y, p3_rb.y, p4_rt.y});
        float max_y = std::max({p1_lt.y, p2_lb.y, p3_rb.y, p4_rt.y});
        cv::Rect bounding_box(min_x, min_y, max_x - min_x, max_y - min_y);

        DetectedArmor armor;
        armor.pnp_corners = {p1_lt, p2_lb, p3_rb, p4_rt}; 
        armor.number = class_id.x;
        armor.class_confidence = (float)score_num;
        armor.confidence = confidence;
        armor.rect = cv::minAreaRect(armor.pnp_corners); 

        boxes.push_back(bounding_box);
        confidences.push_back(confidence);
        temp_armors.push_back(armor);
    }

    // ==================== 3. NMS 去重 ====================
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    std::vector<DetectedArmor> final_armors;
    cv::Point2f imgCenter(img.cols / 2.0f, img.rows / 2.0f);
    
    for (int valid_index : indices) {
        auto armor = temp_armors[valid_index];
        armor.dist_to_center = cv::norm(armor.rect.center - imgCenter);
        final_armors.push_back(armor);
    }

    // 无缝衔接 PnP 与 EKF 
    cv::RotatedRect bestArmorRotatedRect;
    std::vector<cv::Point2f> best_pnp_corners;

    if (!final_armors.empty()) {
        lost_count_ = 0;
        // 第一步：对 YOLO 检测到的所有装甲板进行 PnP 解算
        for (auto& armor : final_armors) {
            float pixel_width = cv::norm(armor.pnp_corners[0] - armor.pnp_corners[3]);
            float pixel_height = cv::norm(armor.pnp_corners[0] - armor.pnp_corners[1]);
            bool is_small = (pixel_width / pixel_height) < 3.2f;
            
            double half_w = is_small ? (135.0 / 2.0) : (230.0 / 2.0);
            double half_h = 55.0 / 2.0;

            // 1. 严格对应 YOLO 的 2D 顺序: 0:左上(LT), 1:左下(LB), 2:右下(RB), 3:右上(RT)
            // 2. 必须定义在 Z=0 平面上！确保法线是 Z 轴，这样 P_cv2ros 才能把它正确映射为 ROS 的 X 轴！
            std::vector<cv::Point3f> correct_object_points = {
                cv::Point3f(-half_w, -half_h, 0), // 左上
                cv::Point3f(-half_w,  half_h, 0), // 左下
                cv::Point3f( half_w,  half_h, 0), // 右下
                cv::Point3f( half_w, -half_h, 0)  // 右上
            };

            // 使用纯正的 3D 点解算 PnP
            calculatePnP(armor, cameraMatrix, distCoeffs, correct_object_points);
        }
    

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

        // OpenCV(RDF) 到 ROS(FLU) 的转换矩阵 
        static const cv::Mat P_cv2ros = (cv::Mat_<double>(3, 3) << 
             0,  0,  1, 
            -1,  0,  0, 
             0, -1,  0
        );            
        
        //  提取原始的 OpenCV 位姿
        cv::Mat R_cv;
        cv::Rodrigues(best_armor.rvec, R_cv);
        cv::Mat t_cv = best_armor.tvec.clone(); // 单位是毫米

        cv::Vec3d normal_c(R_cv.at<double>(0, 2), R_cv.at<double>(1, 2), R_cv.at<double>(2, 2));
        cv::Vec3d pos_c(t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2));
        // 真正的装甲板法线必须是指向相机的！所以它俩的夹角必然 > 90度，即点乘 < 0
        if (normal_c.dot(pos_c) > 0) {
            // 如果点乘 > 0，说明 PnP 解出来的法线背对相机，强行绕自身翻转 180 度！
            cv::Mat R_flip = (cv::Mat_<double>(3, 3) << -1, 0, 0,  0, 1, 0,  0, 0, -1);
            R_cv = R_cv * R_flip;
        }

        // 保留相机系转换
        cv::Mat t_ros = P_cv2ros * t_cv;
        cv::Mat R_ros = P_cv2ros * R_cv; 

        // RViz TF 广播 
        if (tf_armor_broadcaster_) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp = timestamp;
            ts.header.frame_id = "camera_frame";
            ts.child_frame_id = "armor_link";

            // 直接赋值
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
        }// 5. 使用 PoseStamped 转换位置与姿态 
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

        // 6. 提取纯正的 World Frame 数据喂给 EKF 
        
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


        double roll, pitch, yaw;
        m_world.getRPY(roll, pitch, yaw);

        // 说明：因为装甲板的 X 轴实际上是贴着表面的切线（指向右侧），
        // 我们只需给提取出来的 yaw 加上 90度（PI/2），就能得到完美的垂直法线角度。
        double bestYaw_rad_world = yaw + CV_PI/2.0;
        bestYaw_rad_world = -bestYaw_rad_world;
        ArmorEKF::normalizeYaw(bestYaw_rad_world);

        // 数据关联 (让车体连续旋转)
        double current_timestamp = timestamp.seconds();
        double continuous_car_yaw = bestYaw_rad_world; 

        // t_ros 是相机坐标系下装甲板的位置，它就是相机的视线向量
        cv::Vec3d ray_cam(t_ros.at<double>(0), t_ros.at<double>(1), t_ros.at<double>(2));
        ray_cam = cv::normalize(ray_cam);
        
        // R_ros 的 X 轴就是装甲板在相机坐标系下的法向量
        cv::Vec3d normal_cam(R_ros.at<double>(0, 0), R_ros.at<double>(1, 0), R_ros.at<double>(2, 0));

        if (!ekf_initialized) {
            ekf.init(tvec_world, bestYaw_rad_world, current_timestamp);
            ekf_initialized = true;
            found = true;
        } else {
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
        // 广播 EKF 预测点到 RViz 
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
        // 将 ROS 坐标 (前左上) 反向映射回 OpenCV 坐标 (右下前)
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
            // cv::putText(img, "Pred Armor", pred_armor_2d[0] + cv::Point2f(10, 10),
            //             cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
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
            // cv::putText(img, "Robot Center", robot_center_2d[0] + cv::Point2f(10, -10),
            //             cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);   
        }

        // // 调用新函数：中心点传入 EKF 的 robot_center_3d[0]
        // drawFourArmorsFromCenter(img, robot_center_3d[0], draw_yaw);
        double xc_w = ekf.x.at<double>(0, 0) / 1000.0;
        double yc_w = ekf.x.at<double>(2, 0) / 1000.0;
        double zc_w = ekf.x.at<double>(4, 0) / 1000.0;
        double ekf_yaw_w = ekf.x.at<double>(6, 0);
        double r1_w = ekf.x.at<double>(8, 0) / 1000.0;
        double r2_w = ekf.x.at<double>(9, 0) / 1000.0;
        double dz_w = ekf.x.at<double>(10, 0) / 1000.0;

        // 调用画图函数
        drawFourArmorsFromWorld(img, xc_w, yc_w, zc_w, ekf_yaw_w, r1_w, r2_w, dz_w, timestamp);
        if (tf_armor_broadcaster_) {
            // 1. 发布整车中心 TF (父节点: world_frame)
            geometry_msgs::msg::TransformStamped tf_center;
            tf_center.header.stamp = timestamp;
            tf_center.header.frame_id = "world_frame";
            tf_center.child_frame_id = "robot_center_link";

            // 填入 EKF 算出的世界坐标系绝对位置
            tf_center.transform.translation.x = xc_w; 
            tf_center.transform.translation.y = yc_w;
            tf_center.transform.translation.z = zc_w;

            // 填入 EKF 算出的车体偏航角 (Yaw)
            tf2::Quaternion q_center;
            q_center.setRPY(0, 0, ekf_yaw_w);
            tf_center.transform.rotation.x = q_center.x();
            tf_center.transform.rotation.y = q_center.y();
            tf_center.transform.rotation.z = q_center.z();
            tf_center.transform.rotation.w = q_center.w();

            tf_armor_broadcaster_->sendTransform(tf_center);

            // 2. 循环发布 4 个装甲板的 TF (父节点:robot_center_link)
            double R_m = robot_geom_.armor_plate_distance / 1000.0; // 半径转换为米
            
            for (int i = 0; i < 4; ++i) {
                geometry_msgs::msg::TransformStamped tf_plate;
                tf_plate.header.stamp = timestamp;
                tf_plate.header.frame_id = "robot_center_link"; 
                tf_plate.child_frame_id = "armor_link_" + std::to_string(i);

                // 因为父节点已经带着车体 Yaw 旋转了，所以这里的坐标是"相对坐标"
                double plate_yaw_rel = i * (CV_PI / 2.0)+ CV_PI; // 0, 90, 180, 270 度的相对角度
                tf_plate.transform.translation.x = R_m * std::cos(plate_yaw_rel); 
                tf_plate.transform.translation.y = R_m * std::sin(plate_yaw_rel);
                tf_plate.transform.translation.z = 0.0; // 假设装甲板和中心点高度一致

                // 装甲板朝向外部的相对旋转角
                tf2::Quaternion q_plate;
                q_plate.setRPY(0, 0, plate_yaw_rel);
                tf_plate.transform.rotation.x = q_plate.x();
                tf_plate.transform.rotation.y = q_plate.y();
                tf_plate.transform.rotation.z = q_plate.z();
                tf_plate.transform.rotation.w = q_plate.w();

                tf_armor_broadcaster_->sendTransform(tf_plate);
            }
        }
        // 14. 填充目标消息
        target_msg.is_detected = true;
        target_msg.yaw = static_cast<float>(raw_yaw);
        target_msg.pitch = static_cast<float>(raw_pitch);
        target_msg.distance = static_cast<float>(filtered_dist);
        target_msg.filtered_yaw = static_cast<float>(filtered_yaw);
        target_msg.center_x = static_cast<float>(bestArmorRotatedRect.center.x);
        target_msg.center_y = static_cast<float>(bestArmorRotatedRect.center.y);
        target_msg.number = best_armor.number;

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
                std::string text = "Num:" + std::to_string(armor.number) ;
                cv::putText(img, text, armor.pnp_corners[0] - cv::Point2f(0, 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
            }
        }

        // 16. 绘制最佳装甲板
        cv::Point2f final_armor_corners[4];
        bestArmorRotatedRect.points(final_armor_corners);
        for (int k = 0; k < 4; k++) {
            cv::line(img, final_armor_corners[k], final_armor_corners[(k+1)%4], cv::Scalar(0, 0, 255), 1);
        }

        // 17. 保存装甲板实际朝向角数据（用于绘制曲线验证陀螺状态）
        // 实际观测到的装甲板朝向角 (由 PnP 提取并转为世界系的 bestYaw_rad_world)
        float observed_armor_yaw = bestYaw_rad_world * 180.0 / CV_PI;

        // EKF 滤波后的装甲板/车体朝向角 (状态矩阵的第7维：绝对 yaw)
        float ekf_armor_yaw = ekf.x.at<double>(6, 0) * 180.0 / CV_PI;

        rawYawList.push_back(observed_armor_yaw); // 传入绘图：红点（实际识别的装甲板朝向）
        filteredYawList.push_back(ekf_armor_yaw); // 传入绘图：蓝线（EKF 滤波后的装甲板朝向）
        
        if (rawYawList.size() > 400) rawYawList.pop_front();
        if (filteredYawList.size() > 400) filteredYawList.pop_front();

        // 18. 绘制位姿信息文本
        // cv::Point2f text_pos = bestArmorRotatedRect.center;
        // text_pos.y -= bestArmorRotatedRect.size.height / 2 + 20;
        // char text_buf[256];
         //屏幕上的文字也统一显示世界系的角度，方便肉眼对比
        // sprintf(text_buf, "Raw World: Y=%.5f P=%.5f | EKF: Y=%.5f P=%.5f", 
        //         raw_yaw_world, raw_pitch_world, filtered_yaw, filtered_pitch);
        // cv::putText(img, text_buf, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
    } else { 
        // 未检测到装甲板：触发丢帧保护状态机
        lost_count_++;
        if (lost_count_ > 30) { 
            // 连续丢失超过 30 帧 (约 0.3 秒)，说明真丢了，彻底重置
            found = false;
            ekf_initialized = false;
            center_fits.clear(); 
        } else if (ekf_initialized) {
            // 短暂掉帧，依靠 EKF 惯性进行盲推预测！
            double current_timestamp = timestamp.seconds();
            ekf.predict(current_timestamp); 

            cv::Mat tvec_pred_world;
            double yaw_pred_rad_world;
            ekf.getPredictedArmor(tvec_pred_world, yaw_pred_rad_world); 

            double Xw = tvec_pred_world.at<double>(0) / 1000.0;
            double Yw = tvec_pred_world.at<double>(1) / 1000.0;
            double Zw = tvec_pred_world.at<double>(2) / 1000.0;

            double yaw_rad = std::atan2(Yw, Xw);
            double horizontal_dist = std::sqrt(Xw*Xw + Yw*Yw);
            double pitch_rad = std::atan2(Zw, horizontal_dist);

            // 盲推数据依然发给电控，保证云台不会因为闪烁而抽搐
            target_yaw_ = yaw_rad * 180.0 / CV_PI + YAW_OFFSET;
            target_yaw_ = -target_yaw_;
            target_pitch_ = pitch_rad * 180.0 / CV_PI + PITCH_OFFSET;

            while (target_yaw_ > 180.0f)  target_yaw_ -= 360.0f;
            while (target_yaw_ < -180.0f) target_yaw_ += 360.0f;

            target_msg.is_detected = true; // 欺骗电控，告诉它目标还在
            target_msg.filtered_yaw = static_cast<float>(yaw_rad * 180.0 / CV_PI);
            found = true;
        }
    }
    // 19. 绘制yaw滤波曲线
    drawYawPlot();
    return target_msg;
}
