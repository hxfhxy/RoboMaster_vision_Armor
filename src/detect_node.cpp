/**
 * @file detect_node.cpp
 * @brief ROS2 装甲检测节点
 * 该节点订阅图像话题，进行装甲板检测，并通过串口与电控通信。
 */

#include <termios.h>
#include "cpp08_armor_detector/uart_work.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cpp08_armor_detector/msg/armor_target.hpp"
#include "cpp08_armor_detector/armor_detector.hpp"
#include "cpp08_armor_detector/uart_protocol.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @class DetectNode
 * @brief 装甲检测节点类
 * 继承自 rclcpp::Node，负责图像处理、目标检测和串口通信。
 */
class DetectNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * 初始化订阅者、发布者、检测器和串口。
     */
    DetectNode() : Node("detect_node") {

        // 1. 初始化 TF2 组件
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        tf_armor_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 3. 初始化检测器（仅一次！）
        detector_ = std::make_shared<ArmorDetector>();
        detector_->setTFBuffer(tf_buffer_); // 注入 tf_buffer
        detector_->setArmorTFBroadcaster(tf_armor_broadcaster_); // 注入装甲板TF广播器

        
        // 加载模型
        std::string model_path = "/home/hzy/ArmorDetector1/src/cpp08_armor_detector/model/Zenet-已训练好.onnx";
        detector_->loadModel(model_path);

        // 4. 其他初始化...
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/armor/image_raw", 1,
            std::bind(&DetectNode::img_callback, this, std::placeholders::_1));
        target_pub_ = this->create_publisher<cpp08_armor_detector::msg::ArmorTarget>("/armor/target", 10);

        serial_opened_ = serial_.init("/dev/ttyACM0", B115200);
        if (serial_opened_) {
            RCLCPP_INFO(this->get_logger(), "串口初始化成功");
        } else {
            RCLCPP_WARN(this->get_logger(), "串口未连接，仅运行识别");
        }

        RCLCPP_INFO(this->get_logger(), "识别节点已启动");
        cv::namedWindow("Armor Detect", cv::WINDOW_NORMAL);
    }
    
private:
    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    // 目标发布者
    rclcpp::Publisher<cpp08_armor_detector::msg::ArmorTarget>::SharedPtr target_pub_;
    // 装甲检测器
    std::shared_ptr<ArmorDetector> detector_;
    // 串口对象
    RoboMasterUART serial_;
    // 串口是否打开标志
    bool serial_opened_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;//TF2缓冲区，用于坐标转换
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;//监听器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;//动态变换广播器
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;//静态变换广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_armor_broadcaster_;// 用于广播装甲板位置的动态变换

    /**
     * @brief 图像回调函数
     */
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv::Mat img_rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::Mat img;
            cv::cvtColor(img_rgb, img, cv::COLOR_RGB2BGR);

            // 1. 接收电控角度
            if (serial_opened_) {
                GimbalToVision_Data rx_data{}; 
                bool got_new_data = false;
                // 单线程清空缓存法，确保拿到的永远是最新的一帧
                while (serial_.receiveGimbal(rx_data)) {
                    got_new_data = true;
                }

                if (got_new_data) {
                    // ==================== 新增：打印接收到的串口数据 ====================
                    RCLCPP_INFO(this->get_logger(), ">>> [串口接收] 当前云台角度: Yaw=%.2f, Pitch=%.2f", 
                                rx_data.Gimbal_Yaw_Current, rx_data.Gimbal_Pitch_Current);

                    // ==================== 发布【世界 -> camera】的动态变换 ====================
                    geometry_msgs::msg::TransformStamped dynamic_tf;// 定义从 "world_frame" 到 "camera_frame" 的动态变换，表示云台当前的姿态
                    dynamic_tf.header.stamp = msg->header.stamp; // 使用图像消息的时间戳，保持同步
                    dynamic_tf.header.frame_id = "world_frame";// 父坐标系
                    dynamic_tf.child_frame_id = "camera_frame";// 子坐标系
                    
                    dynamic_tf.transform.translation.x = 0.0;
                    dynamic_tf.transform.translation.y = 0.0;
                    dynamic_tf.transform.translation.z = 0.0;

                    tf2::Quaternion q;// 定义四元数，表示云台的旋转
                    // 将接收到的云台角度转换为弧度，并设置四元数（注意RPY顺序和符号）
                    double gimbal_yaw_rad = rx_data.Gimbal_Yaw_Current * M_PI / 180.0;   // 绕Y轴
                    double gimbal_pitch_rad = -rx_data.Gimbal_Pitch_Current * M_PI / 180.0; // 绕X轴

                    q.setRPY(0.0, gimbal_pitch_rad, gimbal_yaw_rad);

                    dynamic_tf.transform.rotation.x = q.x();
                    dynamic_tf.transform.rotation.y = q.y();
                    dynamic_tf.transform.rotation.z = q.z();
                    dynamic_tf.transform.rotation.w = q.w();

                    tf_broadcaster_->sendTransform(dynamic_tf);// 广播动态变换

                    // 更新给检测器里的变量留作备用
                    detector_->setGimbalCurrent(rx_data.Gimbal_Yaw_Current, rx_data.Gimbal_Pitch_Current);
                }
            }

            // 2. 运行识别
            auto target_msg = detector_->detect(img, msg->header.stamp);
            target_msg.header = msg->header;
            target_pub_->publish(target_msg);

            // 3. 准备发送数据
            Manifold_UART_Rx_Data tx_data{}; 

            if (target_msg.is_detected) {
                tx_data.Gimbal_Yaw_Angle = -detector_->getTargetYaw();
                tx_data.Gimbal_Pitch_Angle = detector_->getTargetPitch();
                tx_data.target_valid = 1; 
                RCLCPP_INFO(this->get_logger(), "<<< [串口发送] 目标角度: Y=%.2f P=%.2f",
                            tx_data.Gimbal_Yaw_Angle, tx_data.Gimbal_Pitch_Angle);
            } else {
                tx_data.Gimbal_Yaw_Angle = 0.0f;
                tx_data.Gimbal_Pitch_Angle = 0.0f;
                tx_data.target_valid = 0; 
                // 没识别到时可以选择保持静默，或打印提示
                RCLCPP_INFO(this->get_logger(), "未识别到，发送 0, 0");
            }

            // 仅在串口打开时才发送
            if (serial_opened_) {
                if (!serial_.send(tx_data)) {
                    RCLCPP_DEBUG(this->get_logger(), "发送失败");
                }
            }

            // 显示检测结果图像
            cv::imshow("Armor Detect", img);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "图像错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    rclcpp::shutdown();
    return 0;
}