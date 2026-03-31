#include <termios.h>  
#include "cpp08_armor_detector/uart_work.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cpp08_armor_detector/msg/armor_target.hpp" // 引用自定义消息
#include "cpp08_armor_detector/armor_detector.hpp"

// 新增：引入你的串口协议和串口操作类
#include "cpp08_armor_detector/uart_protocol.hpp" 
// 假设你把 RoboMasterUART 类的声明放在了这个头文件里
// #include "cpp08_armor_detector/uart_worker.hpp" 

class DetectNode : public rclcpp::Node {
public:
    DetectNode() : Node("detect_node") {
        // 订阅话题名必须与 CameraNode 发布的一致
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/armor/image_raw", 1,
            std::bind(&DetectNode::img_callback, this, std::placeholders::_1));

        // 发布识别后的目标信息
        target_pub_ = this->create_publisher<cpp08_armor_detector::msg::ArmorTarget>("/armor/target", 10);

        detector_ = std::make_shared<ArmorDetector>();

        // --- 新增：初始化串口 ---
        // 假设你的 RoboMasterUART 类实例叫 serial_
        if (serial_.init("/dev/ttyUSB0", B115200)) {
            RCLCPP_INFO(this->get_logger(), "串口初始化成功，准备与电控通信...");
        } else {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败！请检查权限或连线。");
        }

        RCLCPP_INFO(this->get_logger(), "识别节点已启动，等待图像...");
        //cv::namedWindow("Armor Detect", cv::WINDOW_NORMAL);
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 运行识别算法
        auto target_msg = detector_->detect(img);
        target_msg.header = msg->header;
        
        // 发布 ROS2 话题供调试/记录
        target_pub_->publish(target_msg);

        // --- 串口发送逻辑 ---
        Manifold_UART_Rx_Data tx_data;
        
        // 强制填充协议定义的起始/结束标志位
        tx_data.Frame_Header = 0xFF; 
        tx_data.Frame_Tail = 0xFE;

        if (target_msg.is_detected) {
            // 映射你的 .msg 文件字段
            tx_data.Gimbal_Pitch_Angle = target_msg.pitch;
            tx_data.Gimbal_Yaw_Angle_Increment = target_msg.yaw;
            // 假设你从算法中能获取到目标 ID，这里先硬编码示例
            tx_data.Enemy_ID = ENEMY_ID_INFANTRY_3;
        } else {
            tx_data.Gimbal_Pitch_Angle = 0.0f;
            tx_data.Gimbal_Yaw_Angle_Increment = 0.0f;
            tx_data.Enemy_ID = ENEMY_ID_UNKNOWN;
        }

        // 发送给电控
        serial_.send(tx_data); 

        // 调试用
        if (target_msg.is_detected) {
            RCLCPP_INFO(this->get_logger(), "发送目标: P:%.2f, Y:%.2f", tx_data.Gimbal_Pitch_Angle, tx_data.Gimbal_Yaw_Angle_Increment);
        }

        cv::imshow("Armor Detect", img);
        // cv::waitKey(1); // 必须有这一行才能看到窗口

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 错误: %s", e.what());
    }
}

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<cpp08_armor_detector::msg::ArmorTarget>::SharedPtr target_pub_;
    std::shared_ptr<ArmorDetector> detector_;
    
    // 新增：串口操作类的实例
    RoboMasterUART serial_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    rclcpp::shutdown();
    //cv::destroyAllWindows();
    return 0;
}