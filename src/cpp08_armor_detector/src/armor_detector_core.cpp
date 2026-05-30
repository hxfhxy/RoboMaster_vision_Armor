#include "cpp08_armor_detector/armor_detector.hpp"
#include "cpp08_armor_detector/uart_protocol.hpp"
#include "cpp08_armor_detector/armor_detector_lightbar.hpp"
#include "cpp08_armor_detector/armor_detector_matching.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 从世界坐标系构建4个装甲板的3D模型，并通过TF变换投影到图像平面绘制
 * @param img 待绘制的图像矩阵
 * @param xc_w 机器人中心在世界坐标系的x坐标（米）
 * @param yc_w 机器人中心在世界坐标系的y坐标（米）
 * @param zc_w 机器人中心在世界坐标系的z坐标（米）
 * @param yaw_w 机器人在世界坐标系的偏航角（弧度）
 * @param r1 基准装甲板半径（米）
 * @param r2 侧板装甲板半径（米）
 * @param dz 侧板装甲板高度偏移（米）
 * @param timestamp 时间戳，用于TF变换同步
 */
void ArmorDetector::drawFourArmorsFromWorld(cv::Mat& img, double xc_w, double yc_w, double zc_w, double yaw_w, 
                                            double r1, double r2, double dz, rclcpp::Time timestamp) {
    // 装甲板半宽/半高，转换为米（原始单位为毫米）
    const double W = robot_geom_.armor_width / 1000.0 / 2.0;   
    const double H = robot_geom_.armor_height / 1000.0 / 2.0;  

    // 旋转向量和平移向量初始化为0（仅使用相机内参投影）
    cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);

    // 遍历4个装甲板（前/右/后/左）
    for (int i = 0; i < 4; ++i) {
        // 计算当前装甲板的偏航角（每个装甲板间隔90度）
        double a = yaw_w + i * (CV_PI / 2.0);
        
        // 判断是否为侧板装甲板（1/3号为侧板，0/2号为前后板）
        bool is_side = (i == 1 || i == 3);
        double R = is_side ? r2 : r1;    // 侧板使用r2半径，前后板使用r1
        double Z = is_side ? zc_w + dz : zc_w; // 侧板高度偏移dz

        // 计算装甲板中心在世界坐标系的坐标
        double cx = xc_w - R * std::cos(a);
        double cy = yc_w - R * std::sin(a);
        double cz = Z;
        
        // 装甲板倾斜角度（15度转换为弧度）
        double tilt_angle = 15.0 * CV_PI / 180.0;

        // 装甲板高度方向向量（考虑倾斜）
        cv::Point3d h_vec(
            -std::cos(a) * std::sin(tilt_angle) * H, 
            -std::sin(a) * std::sin(tilt_angle) * H, 
            -std::cos(tilt_angle) * H                
        );
        // 装甲板宽度方向向量
        cv::Point3d w_vec(-std::sin(a) * W, std::cos(a) * W, 0); 
        
        // 构建装甲板的4个3D角点（世界坐标系）
        std::vector<cv::Point3d> corners_w = {
            cv::Point3d(cx, cy, cz) + w_vec + h_vec,  // 右上角
            cv::Point3d(cx, cy, cz) - w_vec + h_vec,  // 左上角
            cv::Point3d(cx, cy, cz) - w_vec - h_vec,  // 左下角
            cv::Point3d(cx, cy, cz) + w_vec - h_vec   // 右下角
        };
        
        // 存储转换到相机坐标系的3D角点（OpenCV格式）
        std::vector<cv::Point3f> corners_cv;
        
        // 遍历每个角点，完成世界坐标系→相机坐标系的TF变换
        for (int j = 0; j < 4; ++j) {
            geometry_msgs::msg::PointStamped pt_w, pt_c;
            pt_w.header.frame_id = "world_frame";
            pt_w.header.stamp = timestamp;
            pt_w.point.x = corners_w[j].x;
            pt_w.point.y = corners_w[j].y;
            pt_w.point.z = corners_w[j].z;
            
            try {
                // TF变换：世界坐标系转换到相机坐标系
                pt_c = tf_buffer_->transform(pt_w, "camera_frame");
            } catch (...) {
                // 变换失败时直接使用世界坐标
                pt_c = pt_w; 
            }
            
            // 坐标系转换（适配OpenCV与ROS的坐标差异），并转换为毫米
            corners_cv.push_back(cv::Point3f(
                -pt_c.point.y * 1000.0,
                -pt_c.point.z * 1000.0,
                 pt_c.point.x * 1000.0
            ));
        }
        
        // 判断装甲板是否在相机后方（z<10mm视为后方，不绘制）
        bool is_behind = false;
        for (auto& pt : corners_cv) {
            if (pt.z < 10.0) is_behind = true;
        }
        if (is_behind) continue;

        // 将3D相机坐标投影到2D图像平面
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(corners_cv, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, img_pts);

        // 绘制装甲板边框和编号
        if (img_pts.size() == 4) {
            // 绘制四边形边框
            for (int j = 0; j < 4; ++j) {
                cv::line(img, img_pts[j], img_pts[(j+1)%4], cv::Scalar(255, 0, 0), 2);
            }
            // 绘制装甲板编号
            cv::putText(img, std::to_string(i), img_pts[1] + cv::Point2f(0, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }
    }
}

/**
 * @brief 设置云台当前的偏航和俯仰角度
 * @param yaw_current 云台当前偏航角（度）
 * @param pitch_current 云台当前俯仰角（度）
 */
void ArmorDetector::setGimbalCurrent(float yaw_current, float pitch_current) {
    gimbal_yaw_current_ = yaw_current;
    gimbal_pitch_current_ = pitch_current;
}

/**
 * @brief 加载ONNX格式的装甲板检测模型
 * @param model_path ONNX模型文件路径
 */
void ArmorDetector::loadModel(const std::string& model_path) {
    net_ = cv::dnn::readNetFromONNX(model_path);
}

/**
 * @brief 装甲板核心检测函数，完成模型推理、后处理、PnP解算、EKF滤波、坐标变换
 * @param img 输入的原始图像
 * @param timestamp 图像对应的时间戳
 * @return 装甲板检测结果消息，包含位姿、距离、编号等信息
 */
cpp08_armor_detector::msg::ArmorTarget ArmorDetector::detect(cv::Mat img, rclcpp::Time timestamp)
{
    // 初始化检测结果消息，默认未检测到目标
    cpp08_armor_detector::msg::ArmorTarget target_msg;
    target_msg.is_detected = false; 

    // 模型输入尺寸（640x640）
    const int INPUT_WIDTH = 640;
    const int INPUT_HEIGHT = 640;
    
    // 图像预处理：归一化、缩放、减均值、通道转换
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);

    // 模型前向推理，获取输出
    std::vector<cv::Mat> net_outputs;
    net_.forward(net_outputs, net_.getUnconnectedOutLayersNames());

    // 提取推理输出（维度转换）
    cv::Mat output = net_outputs[0];
    cv::Mat output_buffer(output.size[1], output.size[2], CV_32F, output.ptr<float>());

    // 检测阈值设置
    float conf_threshold = 0.65f;    // 置信度阈值
    float nms_threshold = 0.45f;     // NMS非极大值抑制阈值
    int detect_color = 0;            // 检测颜色筛选（0-蓝色，1-红色，其他-全部）

    // 存储检测框、置信度、临时装甲板信息
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<DetectedArmor> temp_armors;

    // 计算图像缩放因子（模型输出→原始图像）
    float x_factor = img.cols / (float)INPUT_WIDTH;
    float y_factor = img.rows / (float)INPUT_HEIGHT;

    // 遍历模型输出的每一行检测结果
    for (int i = 0; i < output_buffer.rows; i++) {
        // 获取置信度（sigmoid转换）
        float confidence = output_buffer.at<float>(i, 8);
        confidence = 1.0f / (1.0f + std::exp(-confidence));

        // 置信度低于阈值则跳过
        if (confidence < conf_threshold) continue;

        // 提取颜色得分和数字编号得分
        cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);
        cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22);
        
        // 获取最高分对应的颜色ID和数字ID
        cv::Point class_id, color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);

        // 过滤不需要的颜色（2/3为无效颜色，detect_color筛选指定颜色）
        if (color_id.x == 2 || color_id.x == 3) continue;
        if (detect_color == 0 && color_id.x == 1) continue; 
        if (detect_color == 1 && color_id.x == 0) continue; 

        // 提取装甲板4个角点（模型输出坐标→原始图像坐标）
        cv::Point2f p1_lt(output_buffer.at<float>(i, 0) * x_factor, output_buffer.at<float>(i, 1) * y_factor);
        cv::Point2f p2_lb(output_buffer.at<float>(i, 2) * x_factor, output_buffer.at<float>(i, 3) * y_factor); 
        cv::Point2f p3_rb(output_buffer.at<float>(i, 4) * x_factor, output_buffer.at<float>(i, 5) * y_factor); 
        cv::Point2f p4_rt(output_buffer.at<float>(i, 6) * x_factor, output_buffer.at<float>(i, 7) * y_factor); 

        // 计算检测框的最小外接矩形
        float min_x = std::min({p1_lt.x, p2_lb.x, p3_rb.x, p4_rt.x});
        float max_x = std::max({p1_lt.x, p2_lb.x, p3_rb.x, p4_rt.x});
        float min_y = std::min({p1_lt.y, p2_lb.y, p3_rb.y, p4_rt.y});
        float max_y = std::max({p1_lt.y, p2_lb.y, p3_rb.y, p4_rt.y});
        cv::Rect bounding_box(min_x, min_y, max_x - min_x, max_y - min_y);

        // 填充装甲板检测信息
        DetectedArmor armor;
        armor.pnp_corners = {p1_lt, p2_lb, p3_rb, p4_rt};  // PnP解算用角点
        armor.number = class_id.x;                         // 装甲板数字编号
        armor.class_confidence = (float)score_num;         // 编号置信度
        armor.confidence = confidence;                     // 整体置信度
        armor.rect = cv::minAreaRect(armor.pnp_corners);   // 最小面积旋转矩形

        // 存入临时容器，用于后续NMS
        boxes.push_back(bounding_box);
        confidences.push_back(confidence);
        temp_armors.push_back(armor);
    }

    // 非极大值抑制，去除重复检测框
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    // 存储NMS后的有效装甲板
    std::vector<DetectedArmor> final_armors;
    cv::Point2f imgCenter(img.cols / 2.0f, img.rows / 2.0f);  // 图像中心坐标
    
    // 遍历NMS筛选后的有效索引
    for (int valid_index : indices) {
        auto armor = temp_armors[valid_index];
        // 计算装甲板中心到图像中心的距离（用于后续排序）
        armor.dist_to_center = cv::norm(armor.rect.center - imgCenter);
        final_armors.push_back(armor);
    }

    // 存储最优装甲板的旋转矩形和角点
    cv::RotatedRect bestArmorRotatedRect;
    std::vector<cv::Point2f> best_pnp_corners;

    // 存在有效装甲板时的处理逻辑
    if (!final_armors.empty()) {
        lost_count_ = 0;  // 重置丢失计数
        // 对每个有效装甲板进行PnP解算（获取位姿）
        for (auto& armor : final_armors) {
            calculatePnP(armor, cameraMatrix, distCoeffs, {});
        }

        // 排序：优先按置信度，置信度接近时按到图像中心距离
        std::sort(final_armors.begin(), final_armors.end(),
            [&imgCenter](const DetectedArmor& a, const DetectedArmor& b) {
                if (std::abs(a.confidence - b.confidence) > 0.05) {
                    return a.confidence > b.confidence;
                }
                return a.dist_to_center < b.dist_to_center;
            });

        // 选取最优装甲板（排序后第一个）
        const auto& best_armor = final_armors[0];
        bestArmorRotatedRect = best_armor.rect;
        best_pnp_corners = best_armor.pnp_corners;

        // OpenCV→ROS坐标系转换矩阵（适配坐标方向差异）
        static const cv::Mat P_cv2ros = (cv::Mat_<double>(3, 3) << 
             0,  0,  1, 
            -1,  0,  0, 
             0, -1,  0
        );            
        
        // 旋转向量→旋转矩阵（Rodrigues变换）
        cv::Mat R_cv;
        cv::Rodrigues(best_armor.rvec, R_cv);
        cv::Mat t_cv = best_armor.tvec.clone();  // 平移向量

        // 装甲板法向量（防止法向量朝向相机后方）
        cv::Vec3d normal_c(R_cv.at<double>(0, 2), R_cv.at<double>(1, 2), R_cv.at<double>(2, 2));
        cv::Vec3d pos_c(t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2));
        if (normal_c.dot(pos_c) > 0) {
            // 法向量朝后时，翻转旋转矩阵
            cv::Mat R_flip = (cv::Mat_<double>(3, 3) << -1, 0, 0,  
                                                        0, 1, 0,  
                                                        0, 0, -1);
            R_cv = R_cv * R_flip;
        }

        // 转换平移/旋转矩阵到ROS坐标系
        cv::Mat t_ros = P_cv2ros * t_cv;
        cv::Mat R_ros = P_cv2ros * R_cv; 

        // 广播装甲板的TF变换（camera_frame→armor_link）
        if (tf_armor_broadcaster_) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp = timestamp;
            ts.header.frame_id = "camera_frame";
            ts.child_frame_id = "armor_link";

            // 平移量（毫米→米）
            ts.transform.translation.x = t_ros.at<double>(0) / 1000.0;
            ts.transform.translation.y = t_ros.at<double>(1) / 1000.0;
            ts.transform.translation.z = t_ros.at<double>(2) / 1000.0;

            // 旋转矩阵→四元数
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
        }

        // 构建相机坐标系下的装甲板位姿
        geometry_msgs::msg::PoseStamped pose_cam;
        pose_cam.header.frame_id = "camera_frame";
        pose_cam.header.stamp = timestamp; 
        
        // 平移量（毫米→米）
        pose_cam.pose.position.x = t_ros.at<double>(0) / 1000.0;
        pose_cam.pose.position.y = t_ros.at<double>(1) / 1000.0;
        pose_cam.pose.position.z = t_ros.at<double>(2) / 1000.0;

        // 旋转矩阵→四元数（填充到位姿消息）
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

        // 转换装甲板位姿到世界坐标系
        geometry_msgs::msg::PoseStamped pose_world;
        try {
            pose_world = tf_buffer_->transform(pose_cam, "world_frame");
        } catch (const tf2::TransformException & ex) {
            // 变换失败时使用相机坐标系位姿
            pose_world = pose_cam; 
        }

        // 提取世界坐标系下的平移向量（米→毫米）
        cv::Mat tvec_world = (cv::Mat_<double>(3,1) <<
            pose_world.pose.position.x * 1000.0,
            pose_world.pose.position.y * 1000.0,
            pose_world.pose.position.z * 1000.0);

        // 四元数→旋转矩阵，提取RPY角
        tf2::Quaternion q_world(
            pose_world.pose.orientation.x,
            pose_world.pose.orientation.y,
            pose_world.pose.orientation.z,
            pose_world.pose.orientation.w
        );
        tf2::Matrix3x3 m_world(q_world);
        double roll, pitch, yaw;
        m_world.getRPY(roll, pitch, yaw);

        // 修正装甲板偏航角（适配坐标系）
        double bestYaw_rad_world = yaw + CV_PI/2.0;
        bestYaw_rad_world = -bestYaw_rad_world;
        
        // 角度归一化（-π ~ π）
        while (bestYaw_rad_world > CV_PI) bestYaw_rad_world -= 2.0 * CV_PI;
        while (bestYaw_rad_world < -CV_PI) bestYaw_rad_world += 2.0 * CV_PI;

        // 获取当前时间戳（秒）
        double current_timestamp = timestamp.seconds();

        // 计算相机坐标系下的视线方向向量（归一化）
        cv::Vec3d ray_cam(t_ros.at<double>(0), t_ros.at<double>(1), t_ros.at<double>(2));
        ray_cam = cv::normalize(ray_cam);
        cv::Vec3d normal_cam(R_ros.at<double>(0, 0), R_ros.at<double>(1, 0), R_ros.at<double>(2, 0));

        // EKF初始化（首次检测到装甲板时）
        if (!ekf_initialized) {
            // 装甲板位置（毫米→米）
            Eigen::Vector3d p_armor(
                tvec_world.at<double>(0) / 1000.0, 
                tvec_world.at<double>(1) / 1000.0, 
                tvec_world.at<double>(2) / 1000.0
            );
            // EKF初始协方差对角矩阵
            Eigen::Matrix<double, 11, 1> P0_diag;
            P0_diag << 100, 100, 100, 100, 100, 100, 10, 10, 0.01, 0.01, 0.01;
            
            // 初始化EKF（初始半径0.26米）
            ekf.init(p_armor, bestYaw_rad_world, 4, 0.26, 0.0, 0.0, P0_diag);
            
            // 记录上一时刻时间戳，标记EKF初始化完成
            prev_timestamp_ = current_timestamp;
            ekf_initialized = true;
            found = true;
        } else {
            // EKF预测与更新（非首次检测）
            // 计算时间差（限制最大/最小值，避免异常）
            double dt = current_timestamp - prev_timestamp_;
            if (dt > 0.1 || dt <= 0.0) dt = 0.033;
            prev_timestamp_ = current_timestamp;

            // EKF预测步骤
            ekf.predict(dt);

            // 装甲板位置（毫米→米）
            Eigen::Vector3d armor_xyz(
                tvec_world.at<double>(0) / 1000.0, 
                tvec_world.at<double>(1) / 1000.0, 
                tvec_world.at<double>(2) / 1000.0
            );
            
            // 计算观测值：距离、相机坐标系下的偏航/俯仰角、装甲板偏航角
            double dist = armor_xyz.norm();
            double yaw_cam = std::atan2(armor_xyz(1), armor_xyz(0));
            double pitch_cam = std::atan2(armor_xyz(2), std::sqrt(armor_xyz(0)*armor_xyz(0) + armor_xyz(1)*armor_xyz(1)));
            
            Eigen::Vector4d z_obs(yaw_cam, pitch_cam, dist, bestYaw_rad_world);
            
            // EKF更新步骤
            ekf.update(z_obs);
        }

        // 提取EKF预测的装甲板中心位置
        Eigen::Vector3d pred_armor = ekf.getArmorCenter(); 
        double Xw = pred_armor(0);
        double Yw = pred_armor(1);
        double Zw = pred_armor(2);

        // 计算目标偏航/俯仰角（弹道解算）
        double yaw_rad = std::atan2(Yw, Xw);
        double horizontal_dist = std::sqrt(Xw*Xw + Yw*Yw);
        double current_bullet_speed = 25.0;  // 子弹速度（米/秒）
        double pitch_rad = trajectory_solver_.solveTrajectoryPitch(horizontal_dist, Zw, current_bullet_speed);

        // 角度转换为度，并添加偏移补偿
        double target_yaw_abs = yaw_rad * 180.0 / CV_PI;
        double target_pitch_abs = pitch_rad * 180.0 / CV_PI;

        target_yaw_   = target_yaw_abs + YAW_OFFSET;
        target_yaw_ =-target_yaw_; 
        target_pitch_ = target_pitch_abs + PITCH_OFFSET;

        // 偏航角归一化（-180 ~ 180度）
        while (target_yaw_ > 180.0f)  target_yaw_ -= 360.0f;
        while (target_yaw_ < -180.0f) target_yaw_ += 360.0f;

        // 滤波后的偏航角和距离（米→毫米）
        double filtered_yaw   = target_yaw_abs;
        double filtered_dist  = pred_armor.norm() * 1000.0;

        // 广播EKF预测的装甲板TF变换（world_frame→armor_pred_link）
        if (tf_armor_broadcaster_) {
            geometry_msgs::msg::TransformStamped ts_pred;
            ts_pred.header.stamp = timestamp;
            ts_pred.header.frame_id = "world_frame";    
            ts_pred.child_frame_id = "armor_pred_link"; 

            ts_pred.transform.translation.x = Xw;
            ts_pred.transform.translation.y = Yw;
            ts_pred.transform.translation.z = Zw;

            ts_pred.transform.rotation.x = 0.0;
            ts_pred.transform.rotation.y = 0.0;
            ts_pred.transform.rotation.z = 0.0;
            ts_pred.transform.rotation.w = 1.0;

            tf_armor_broadcaster_->sendTransform(ts_pred);
        }

        // 发布速度向量可视化Marker（RViz显示）
        if (marker_pub_) {
            visualization_msgs::msg::Marker vel_marker;
            vel_marker.header.stamp = timestamp;
            vel_marker.header.frame_id = "world_frame"; 
            vel_marker.ns = "velocity_vector";
            vel_marker.id = 0;
            vel_marker.type = visualization_msgs::msg::Marker::ARROW;
            vel_marker.action = visualization_msgs::msg::Marker::ADD;
            vel_marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // 生命周期0.1秒

            // 速度向量起点（EKF状态中的位置）
            geometry_msgs::msg::Point start_pt;
            start_pt.x = ekf.x(0);
            start_pt.y = ekf.x(2);
            start_pt.z = ekf.x(4);

            // 提取EKF状态中的速度
            double vx = ekf.x(1);
            double vy = ekf.x(3);
            double vz = ekf.x(5); 

            // 速度向量缩放时间（0.5秒）
            double scale_time = 0.5;
            geometry_msgs::msg::Point end_pt;
            end_pt.x = start_pt.x + vx * scale_time;
            end_pt.y = start_pt.y + vy * scale_time;
            end_pt.z = start_pt.z + vz * scale_time;

            // 设置向量起点和终点
            vel_marker.points.push_back(start_pt);
            vel_marker.points.push_back(end_pt);

            // 设置箭头尺寸
            vel_marker.scale.x = 0.03; 
            vel_marker.scale.y = 0.06; 
            vel_marker.scale.z = 0.06; 

            // 黄色箭头（RGBA）
            vel_marker.color.r = 1.0f;
            vel_marker.color.g = 1.0f;
            vel_marker.color.b = 0.0f;
            vel_marker.color.a = 1.0f; 

            marker_pub_->publish(vel_marker);
        }

        // 计算原始偏航/俯仰角（未滤波）
        double raw_yaw = std::atan2(-best_armor.tvec.at<double>(0), best_armor.tvec.at<double>(2)) * 180.0 / CV_PI;
        double raw_pitch = std::atan2(-best_armor.tvec.at<double>(1), best_armor.tvec.at<double>(2)) * 180.0 / CV_PI;

        // 转换预测位置和机器人中心位置到相机坐标系（用于绘制）
        geometry_msgs::msg::PointStamped pred_world_pt, pred_cam_pt;
        pred_world_pt.header.frame_id = "world_frame";
        pred_world_pt.header.stamp = timestamp;
        pred_world_pt.point.x = Xw;
        pred_world_pt.point.y = Yw;
        pred_world_pt.point.z = Zw;

        geometry_msgs::msg::PointStamped center_world_pt, center_cam_pt;
        center_world_pt.header.frame_id = "world_frame";
        center_world_pt.header.stamp = timestamp;
        
        center_world_pt.point.x = ekf.x(0);
        center_world_pt.point.y = ekf.x(2);
        center_world_pt.point.z = ekf.x(4);

        try {
            pred_cam_pt = tf_buffer_->transform(pred_world_pt, "camera_frame");
            center_cam_pt = tf_buffer_->transform(center_world_pt, "camera_frame");
        } catch (const tf2::TransformException& ex) {
            // 变换失败时直接使用世界坐标
            pred_cam_pt = pred_world_pt;
            center_cam_pt = center_world_pt;
        }
        cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);   
        
        // 绘制预测装甲板中心（黄色圆点）
        std::vector<cv::Point3f> pred_armor_3d;
        pred_armor_3d.push_back(cv::Point3f(
            -pred_cam_pt.point.y * 1000.0, 
            -pred_cam_pt.point.z * 1000.0, 
             pred_cam_pt.point.x * 1000.0
        ));
        
        std::vector<cv::Point2f> pred_armor_2d;
        cv::projectPoints(pred_armor_3d, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, pred_armor_2d);
        
        if (!pred_armor_2d.empty()) {
            cv::circle(img, pred_armor_2d[0], 6, cv::Scalar(0, 255, 255), -1);
        }

        // 绘制机器人中心（白色实心圆+黑色边框）
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
        }

        // 提取EKF状态中的机器人中心和装甲板参数
        double xc_w = ekf.x(0);
        double yc_w = ekf.x(2);
        double zc_w = ekf.x(4);
        double ekf_yaw_w = ekf.x(6);
        double r1_w = ekf.x(8);
        double r2_w = ekf.x(8) + ekf.x(9); 
        double dz_w = ekf.x(10);

        // 绘制4个装甲板的投影边框
        drawFourArmorsFromWorld(img, xc_w, yc_w, zc_w, ekf_yaw_w, r1_w, r2_w, dz_w, timestamp);
        
        // 广播机器人中心和4个装甲板的TF变换
        if (tf_armor_broadcaster_) {
            // 广播机器人中心TF（world_frame→robot_center_link）
            geometry_msgs::msg::TransformStamped tf_center;
            tf_center.header.stamp = timestamp;
            tf_center.header.frame_id = "world_frame";
            tf_center.child_frame_id = "robot_center_link";

            tf_center.transform.translation.x = xc_w; 
            tf_center.transform.translation.y = yc_w;
            tf_center.transform.translation.z = zc_w;

            // 机器人偏航角转换为四元数
            tf2::Quaternion q_center;
            q_center.setRPY(0, 0, ekf_yaw_w);
            tf_center.transform.rotation.x = q_center.x();
            tf_center.transform.rotation.y = q_center.y();
            tf_center.transform.rotation.z = q_center.z();
            tf_center.transform.rotation.w = q_center.w();

            tf_armor_broadcaster_->sendTransform(tf_center);

            // 装甲板到机器人中心的距离（毫米→米）
            double R_m = robot_geom_.armor_plate_distance / 1000.0; 
            
            // 广播4个装甲板的TF变换（robot_center_link→armor_link_i）
            for (int i = 0; i < 4; ++i) {
                geometry_msgs::msg::TransformStamped tf_plate;
                tf_plate.header.stamp = timestamp;
                tf_plate.header.frame_id = "robot_center_link"; 
                tf_plate.child_frame_id = "armor_link_" + std::to_string(i);

                // 装甲板相对机器人中心的偏航角
                double plate_yaw_rel = i * (CV_PI / 2.0)+ CV_PI; 
                // 装甲板相对机器人中心的平移
                tf_plate.transform.translation.x = R_m * std::cos(plate_yaw_rel); 
                tf_plate.transform.translation.y = R_m * std::sin(plate_yaw_rel);
                tf_plate.transform.translation.z = 0.0; 

                // 装甲板偏航角转换为四元数
                tf2::Quaternion q_plate;
                q_plate.setRPY(0, 0, plate_yaw_rel);
                tf_plate.transform.rotation.x = q_plate.x();
                tf_plate.transform.rotation.y = q_plate.y();
                tf_plate.transform.rotation.z = q_plate.z();
                tf_plate.transform.rotation.w = q_plate.w();

                tf_armor_broadcaster_->sendTransform(tf_plate);
            }
        }
        
        // 填充检测结果消息
        target_msg.is_detected = true;
        target_msg.yaw = static_cast<float>(raw_yaw);
        target_msg.pitch = static_cast<float>(raw_pitch);
        target_msg.distance = static_cast<float>(filtered_dist);
        target_msg.filtered_yaw = static_cast<float>(filtered_yaw);
        target_msg.center_x = static_cast<float>(bestArmorRotatedRect.center.x);
        target_msg.center_y = static_cast<float>(bestArmorRotatedRect.center.y);
        target_msg.number = best_armor.number;

        // 绘制所有有效装甲板的边框、角点、编号
        for (auto &armor : final_armors) {
            cv::Point2f armor_corners[4];
            armor.rect.points(armor_corners);
            // 绘制旋转矩形边框（绿色）
            for (int k = 0; k < 4; k++) {
                cv::line(img, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(0, 255, 0), 2);
            }
            // 绘制PnP角点（红色实心圆）
            for (auto &pt : armor.pnp_corners) {
                cv::circle(img, pt, 4, cv::Scalar(0, 0, 255), -1);
            }
            // 绘制最优装甲板角点坐标
            for (auto &pt : best_pnp_corners) {
                cv::putText(img, "(" + std::to_string((int)pt.x) + "," + std::to_string((int)pt.y) + ")", pt + cv::Point2f(5, -5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
            // 绘制装甲板数字编号
            if (armor.number != -1) {
                std::string text = "Num:" + std::to_string(armor.number) ;
                cv::putText(img, text, armor.pnp_corners[0] - cv::Point2f(0, 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
            }
        }

        // 绘制最优装甲板的边框（红色）
        cv::Point2f final_armor_corners[4];
        bestArmorRotatedRect.points(final_armor_corners);
        for (int k = 0; k < 4; k++) {
            cv::line(img, final_armor_corners[k], final_armor_corners[(k+1)%4], cv::Scalar(0, 0, 255), 1);
        }

        // 记录观测和滤波后的偏航角（用于绘制曲线）
        float observed_armor_yaw = bestYaw_rad_world * 180.0 / CV_PI;
        
        // 修正装甲板偏航角（叠加装甲板ID偏移，归一化）
        double current_plate_yaw = ekf.x(6) + ekf.last_id_ * (CV_PI / 2.0);
        while (current_plate_yaw > CV_PI) current_plate_yaw -= 2.0 * CV_PI;
        while (current_plate_yaw < -CV_PI) current_plate_yaw += 2.0 * CV_PI;
        
        float ekf_armor_yaw = current_plate_yaw * 180.0 / CV_PI;

        // 存储偏航角数据（用于绘制曲线）
        rawYawList.push_back(observed_armor_yaw); 
        filteredYawList.push_back(ekf_armor_yaw); 
        
        // 限制列表长度（最多400个数据点）
        if (rawYawList.size() > 400) rawYawList.pop_front();
        if (filteredYawList.size() > 400) filteredYawList.pop_front();

    } else { 
        // 无有效装甲板时的处理逻辑
        lost_count_++;  // 增加丢失计数
        // 丢失超过30帧，重置EKF和状态
        if (lost_count_ > 30) { 
            found = false;
            ekf_initialized = false;
            center_fits.clear(); 
        } else if (ekf_initialized) {
            // 未完全丢失时，仅执行EKF预测
            double current_timestamp = timestamp.seconds();
            double dt = current_timestamp - prev_timestamp_;
            if (dt > 0.1 || dt <= 0.0) dt = 0.033;
            prev_timestamp_ = current_timestamp;

            // EKF预测步骤
            ekf.predict(dt); 

            // 提取EKF预测的装甲板位置
            Eigen::Vector3d pred_armor = ekf.getArmorCenter(); 
            double Xw = pred_armor(0);
            double Yw = pred_armor(1);
            double Zw = pred_armor(2);

            // 计算目标偏航/俯仰角
            double yaw_rad = std::atan2(Yw, Xw);
            double horizontal_dist = std::sqrt(Xw*Xw + Yw*Yw);
            double pitch_rad = std::atan2(Zw, horizontal_dist);

            // 角度转换为度，并添加偏移补偿
            target_yaw_ = yaw_rad * 180.0 / CV_PI + YAW_OFFSET;
            target_yaw_ = -target_yaw_;
            target_pitch_ = pitch_rad * 180.0 / CV_PI + PITCH_OFFSET;

            // 偏航角归一化
            while (target_yaw_ > 180.0f)  target_yaw_ -= 360.0f;
            while (target_yaw_ < -180.0f) target_yaw_ += 360.0f;

            // 填充检测结果
            target_msg.is_detected = true; 
            target_msg.filtered_yaw = static_cast<float>(yaw_rad * 180.0 / CV_PI);
            found = true;
        }
    }
    // 绘制偏航角曲线
    drawYawPlot();
    return target_msg;
}
