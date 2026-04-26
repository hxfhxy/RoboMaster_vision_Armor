#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <opencv2/dnn.hpp>
#include "kalman_filter.hpp"
#include "armor_detector_lightbar.hpp"
#include "armor_detector_matching.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include "cpp08_armor_detector/msg/armor_target.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArmorDetector {
public:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_armor_broadcaster_;
    void setArmorTFBroadcaster(std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster) {
        tf_armor_broadcaster_ = broadcaster;
    }
    // 接收 TF Buffer
    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> buffer) {
        tf_buffer_ = buffer;
    }
    // 新增：加载 ONNX 模型
    void loadModel(const std::string& model_path);

    void setGimbalCurrent(float yaw_current, float pitch_current);
    float getTargetYaw() { return target_yaw_; }
    float getTargetPitch() { return target_pitch_; }

    bool initUART(const char* device = "/dev/ttyUSB0", int baud = 115200);
    
    // 检测参数常量
    static constexpr float HEIGHT_RATIO_THRESH = 0.2f;
    static constexpr float Y_OFFSET_RATIO_THRESH = 0.3f;
    static constexpr float X_DIST_RATIO_THRESH = 5.0f;
    static constexpr int MIN_LIGHTBAR_AREA = 350;
    static constexpr int COLOR_DIFF_THRESH = 15;
    static constexpr int BRIGHTNESS_THRESH = 30;

    // 在 ArmorDetector 类中添加
    struct RobotGeometry {
        double armor_plate_distance; // 装甲板表面到机器人旋转中心(Yaw轴)的半径距离 (mm)
        double armor_width;           // 单块装甲板的宽度 (mm)，你已经有了：135mm
        double armor_height;          // 单块装甲板的高度 (mm)，你已经有了：55mm
    };

    RobotGeometry robot_geom_ = {
        260.0, // 假设机器人中心到装甲板中心的距离是300mm (根据实际调整)
        135.0,
        55.0
    };


    // 相机参数
    cv::Mat cameraMatrix, distCoeffs;   
    // 装甲板物理坐标
    std::vector<cv::Point3f> objectPoints;
    // 卡尔曼滤波器
    ArmorEKF ekf;
    //标记是否初始化过 EKF
    bool ekf_initialized = false;
    // 滤波数据缓存
    std::deque<float> rawYawList, filteredYawList;

    void drawFourArmorsFromOne(cv::Mat& img, const cv::Mat& rvec, const cv::Mat& tvec);


    // 目标颜色设定：0 代表蓝色，1 代表红色
    int enemy_color = 0;   

    //find_robot_center相关成员     
    struct CenterFit {
        cv::Point3f position;
        cv::Point3f normalvector;
    };
    std::vector<CenterFit> center_fits;
    std::vector<cv::Point3f> center_3d;
    std::vector<cv::Point2f> center_2d;
    bool found = false;
    bool find_center = false;
    bool first_frame = true;
    cv::Point2f last_center_2d;

    // 构造函数
    ArmorDetector();

    double YAW_OFFSET = 0.0;
    double PITCH_OFFSET = 0.0;
    bool IS_YAW_REVERSED = false;
    bool IS_PITCH_REVERSED = false;

    // 图像预处理
    cv::Mat preprocess(cv::Mat img);

    // 绘制Yaw滤波图
    void drawYawPlot();

    // 核心识别逻辑
    cpp08_armor_detector::msg::ArmorTarget detect(cv::Mat img,rclcpp::Time timestamp);

    // find_robot_center成员函数声明
    void find_robot_center();

    

private:
    //绝对角度计算成员变量
    float gimbal_yaw_current_ = 0.0f;
    float gimbal_pitch_current_ = 0.0f;
    float target_yaw_ = 0.0f;
    float target_pitch_ = 0.0f;
    const float BULLET_SPEED = 15.0f;

    cv::dnn::Net net_; // 神经网络对象
    
    // 新增：数字分类逻辑
    void classifyArmors(const cv::Mat& src, std::vector<DetectedArmor>& armors);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif // ARMOR_DETECTOR_HPP
