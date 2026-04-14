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
        // 订阅图像话题
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/armor/image_raw", 1,
            std::bind(&DetectNode::img_callback, this, std::placeholders::_1));

        // 发布目标检测结果
        target_pub_ = this->create_publisher<cpp08_armor_detector::msg::ArmorTarget>("/armor/target", 10);

        // 初始化装甲检测器
        detector_ = std::make_shared<ArmorDetector>();
        std::string model_path = "/home/hzy/ArmorDetector1/src/cpp08_armor_detector/model/Zenet-已训练好.onnx";
        detector_->loadModel(model_path);

        // 初始化串口并更新标志位
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

    /**
     * @brief 图像回调函数
     * 处理接收到的图像，进行检测并发布结果。
     * @param msg 接收到的图像消息
     */
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv::Mat img_rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::Mat img;
            cv::cvtColor(img_rgb, img, cv::COLOR_RGB2BGR);

            // 1. 接收电控角度 (仅在串口打开时)
            if (serial_opened_) {
                GimbalToVision_Data rx_data{}; // 初始化为 0
                if (serial_.receiveGimbal(rx_data)) {
                    detector_->setGimbalCurrent(rx_data.Gimbal_Yaw_Current, rx_data.Gimbal_Pitch_Current);
                    RCLCPP_INFO(this->get_logger(), "电控 Y=%.2f P=%.2f",
                                rx_data.Gimbal_Yaw_Current, rx_data.Gimbal_Pitch_Current);
                }
            }

            // 2. 运行识别
            auto target_msg = detector_->detect(img);
            target_msg.header = msg->header;
            target_pub_->publish(target_msg);

            // 3. 准备发送数据
            Manifold_UART_Rx_Data tx_data{}; //  全结构体初始化为 0

            if (target_msg.is_detected) {
                // 识别到: 发真实角度
                tx_data.Gimbal_Yaw_Angle = -detector_->getTargetYaw();
                tx_data.Gimbal_Pitch_Angle = detector_->getTargetPitch();
                RCLCPP_INFO(this->get_logger(), "发送 Y=%.2f P=%.2f",
                            -tx_data.Gimbal_Yaw_Angle, tx_data.Gimbal_Pitch_Angle);
            } else {
                // 没识别到: 发 0, 0
                tx_data.Gimbal_Yaw_Angle = 0.0f;
                tx_data.Gimbal_Pitch_Angle = 0.0f;
                RCLCPP_INFO(this->get_logger(), "未识别到，发送 0, 0");
            }

            // 仅在串口打开时才发送
            if (serial_opened_) {
                if (!serial_.send(tx_data)) {
                    RCLCPP_DEBUG(this->get_logger(), "发送失败"); // 避免刷屏
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

/**
 * @brief 主函数
 * 初始化ROS2并运行检测节点。
 * @param argc 参数数量
 * @param argv 参数数组
 * @return 程序退出码
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    rclcpp::shutdown();
    return 0;
}
