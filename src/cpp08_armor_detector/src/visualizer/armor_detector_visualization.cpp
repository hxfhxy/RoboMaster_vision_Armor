#include "cpp08_armor_detector/detector/armor_detector.hpp"

/**
 * @brief 从世界坐标系参数绘制机器人的四个装甲板到图像上
 * @param img 待绘制的图像
 * @param xc_w 机器人中心世界X坐标(米)
 * @param yc_w 机器人中心世界Y坐标(米)
 * @param zc_w 机器人中心世界Z坐标(米)
 * @param yaw_w 机器人世界偏航角(弧度)
 * @param r1 前后装甲板半径(米)
 * @param r2 左右装甲板半径(米)
 * @param dz 左右装甲板高度偏移(米)
 * @param timestamp 时间戳，用于TF坐标转换
 */
void ArmorDetector::drawFourArmorsFromWorld(cv::Mat& img, double xc_w, double yc_w, double zc_w, double yaw_w,
                                            double r1, double r2, double dz, rclcpp::Time timestamp) {
    // 装甲板半宽和半高(转换为米)
    const double W = robot_geom_.armor_width / 1000.0 / 2.0;
    const double H = robot_geom_.armor_height / 1000.0 / 2.0;

    // 旋转和平移向量为零，因为我们已经将点转换到相机坐标系
    cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);

    // 遍历四个装甲板(ID:0-前,1-右,2-后,3-左)
    for (int i = 0; i < 4; ++i) {
        // 计算当前装甲板的世界偏航角
        double a = yaw_w + i * (CV_PI / 2.0);
        // 判断是否为左右侧板装甲板
        bool is_side = (i == 1 || i == 3);
        // 根据装甲板类型选择对应的半径和高度
        double R = is_side ? r2 : r1;
        double Z = is_side ? zc_w + dz : zc_w;

        // 计算装甲板中心的世界坐标
        // 装甲板在机器人中心的反方向，所以减去半径乘以方向向量
        double cx = xc_w - R * std::cos(a);
        double cy = yc_w - R * std::sin(a);
        double cz = Z;

        // 装甲板向后倾斜15度(标准RM机器人装甲板倾角)
        double tilt_angle = 15.0 * CV_PI / 180.0;
        // 装甲板高度方向的单位向量(考虑倾斜角)
        cv::Point3d h_vec(
            -std::cos(a) * std::sin(tilt_angle) * H,
            -std::sin(a) * std::sin(tilt_angle) * H,
            -std::cos(tilt_angle) * H
        );
        // 装甲板宽度方向的单位向量(垂直于装甲板法线)
        cv::Point3d w_vec(-std::sin(a) * W, std::cos(a) * W, 0);

        // 计算装甲板四个角点的世界坐标(左上→左下→右下→右上)
        std::vector<cv::Point3d> corners_w = {
            cv::Point3d(cx, cy, cz) + w_vec + h_vec,
            cv::Point3d(cx, cy, cz) - w_vec + h_vec,
            cv::Point3d(cx, cy, cz) - w_vec - h_vec,
            cv::Point3d(cx, cy, cz) + w_vec - h_vec
        };

        // 将世界坐标系角点转换为相机坐标系角点
        std::vector<cv::Point3f> corners_cv;
        for (int j = 0; j < 4; ++j) {
            geometry_msgs::msg::PointStamped pt_w, pt_c;
            pt_w.header.frame_id = "world_frame";
            pt_w.header.stamp = timestamp;
            pt_w.point.x = corners_w[j].x;
            pt_w.point.y = corners_w[j].y;
            pt_w.point.z = corners_w[j].z;

            try {
                // TF转换：世界坐标系→相机坐标系
                pt_c = tf_buffer_->transform(pt_w, "camera_frame");
            } catch (...) {
                // 转换失败时使用世界坐标(仅调试用)
                pt_c = pt_w;
            }

            // ROS相机坐标系(FLU)转换为OpenCV相机坐标系(RDF)
            // ROS: x-前, y-左, z-上 | OpenCV: x-右, y-下, z-前
            corners_cv.push_back(cv::Point3f(
                -pt_c.point.y * 1000.0,  // OpenCV x = -ROS y
                -pt_c.point.z * 1000.0,  // OpenCV y = -ROS z
                 pt_c.point.x * 1000.0   // OpenCV z = ROS x
            ));
        }

        // 过滤掉在相机后面的装甲板(z<10mm)
        bool is_behind = false;
        for (auto& pt : corners_cv) {
            if (pt.z < 10.0) is_behind = true;
        }
        if (is_behind) continue;

        // 将相机坐标系3D点投影到图像平面
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(corners_cv, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, img_pts);

        // 绘制装甲板边框和ID
        if (img_pts.size() == 4) {
            // 绘制装甲板四条边(蓝色)
            for (int j = 0; j < 4; ++j) {
                cv::line(img, img_pts[j], img_pts[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
            }
            // 在装甲板左下角绘制ID号
            cv::putText(img, std::to_string(i), img_pts[1] + cv::Point2f(0, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }
    }
}

/**
 * @brief 绘制预测的装甲板中心和机器人中心
 * @param img 待绘制的图像
 * @param pred_cam_pt 预测的装甲板中心(相机坐标系)
 * @param center_cam_pt 机器人中心(相机坐标系)
 */
void ArmorDetector::drawProjectedCenters(cv::Mat& img, const geometry_msgs::msg::PointStamped& pred_cam_pt,
                                         const geometry_msgs::msg::PointStamped& center_cam_pt) {
    // 旋转和平移向量为零
    cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);

    // 转换预测装甲板中心为OpenCV相机坐标系并投影
    std::vector<cv::Point3f> pred_armor_3d = {
        cv::Point3f(
            -pred_cam_pt.point.y * 1000.0,
            -pred_cam_pt.point.z * 1000.0,
             pred_cam_pt.point.x * 1000.0)
    };
    std::vector<cv::Point2f> pred_armor_2d;
    cv::projectPoints(pred_armor_3d, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, pred_armor_2d);
    // 绘制预测装甲板中心(黄色实心圆)
    if (!pred_armor_2d.empty()) {
        cv::circle(img, pred_armor_2d[0], 6, cv::Scalar(0, 255, 255), -1);
    }

    // 转换机器人中心为OpenCV相机坐标系并投影
    std::vector<cv::Point3f> robot_center_3d = {
        cv::Point3f(
            -center_cam_pt.point.y * 1000.0,
            -center_cam_pt.point.z * 1000.0,
             center_cam_pt.point.x * 1000.0)
    };
    std::vector<cv::Point2f> robot_center_2d;
    cv::projectPoints(robot_center_3d, rvec_zero, tvec_zero, cameraMatrix, distCoeffs, robot_center_2d);
    // 绘制机器人中心(白色实心圆+黑色边框)
    if (!robot_center_2d.empty()) {
        cv::circle(img, robot_center_2d[0], 8, cv::Scalar(255, 255, 255), -1);
        cv::circle(img, robot_center_2d[0], 8, cv::Scalar(0, 0, 0), 2);
    }
}

/**
 * @brief 绘制YOLO检测到的所有装甲板
 * @param img 待绘制的图像
 * @param armors 检测到的装甲板列表
 * @param best_pnp_corners 最佳装甲板的PnP角点
 * @param bestArmorRect 最佳装甲板的最小外接旋转矩形
 */
void ArmorDetector::drawDetectedArmors(cv::Mat& img, const std::vector<DetectedArmor>& armors,
                                      const std::vector<cv::Point2f>& best_pnp_corners,
                                      const cv::RotatedRect& bestArmorRect) {
    // 遍历所有检测到的装甲板
    for (const auto& armor : armors) {
        // 绘制装甲板最小外接矩形(绿色)
        cv::Point2f armor_corners[4];
        armor.rect.points(armor_corners);
        for (int k = 0; k < 4; k++) {
            cv::line(img, armor_corners[k], armor_corners[(k + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        // 绘制PnP解算用的四个角点(红色实心圆)
        for (const auto& pt : armor.pnp_corners) {
            cv::circle(img, pt, 4, cv::Scalar(0, 0, 255), -1);
        }

        // 绘制最佳装甲板角点的像素坐标(绿色文字)
        for (const auto& pt : best_pnp_corners) {
            cv::putText(img, "(" + std::to_string(static_cast<int>(pt.x)) + "," + std::to_string(static_cast<int>(pt.y)) + ")",
                        pt + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }

        // 绘制装甲板数字标签(黄色文字)
        if (armor.number != -1) {
            std::string text = "Num:" + std::to_string(armor.number);
            cv::putText(img, text, armor.pnp_corners[0] - cv::Point2f(0, 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
        }
    }

    // 用红色细线突出显示最佳装甲板的最小外接矩形
    cv::Point2f final_armor_corners[4];
    bestArmorRect.points(final_armor_corners);
    for (int k = 0; k < 4; k++) {
        cv::line(img, final_armor_corners[k], final_armor_corners[(k + 1) % 4], cv::Scalar(0, 0, 255), 1);
    }
}

/**
 * @brief 绘制原始yaw和滤波后yaw的对比曲线(调试用)
 */
void ArmorDetector::drawYawPlot()
{
    // 创建400x800的黑色画布
    cv::Mat plot = cv::Mat::zeros(400, 800, CV_8UC3);
    // 绘制中间的零轴线(灰色)
    cv::line(plot, cv::Point(0, 200), cv::Point(800, 200), cv::Scalar(100, 100, 100), 1);
    
    // 角度缩放因子(1像素对应1弧度)
    float scale = 1;
    // 遍历历史yaw数据绘制曲线
    for (size_t i = 1; i < filteredYawList.size(); i++) {
        int x = i * 2;
        // 超出画布宽度时停止绘制
        if (x >= 800)
            break;
        
        // 原始yaw用红色点表示
        cv::circle(plot, cv::Point(x, 200 - rawYawList[i] * scale), 1, cv::Scalar(0, 0, 255), -1);
        // 滤波后yaw用蓝色线表示
        cv::line(plot, cv::Point((i - 1) * 2, 200 - filteredYawList[i - 1] * scale),
                cv::Point(x, 200 - filteredYawList[i] * scale), cv::Scalar(255, 0, 0), 2);
    }
    
    // 显示yaw曲线窗口
    cv::imshow("Yaw Filter Plot", plot);
}
