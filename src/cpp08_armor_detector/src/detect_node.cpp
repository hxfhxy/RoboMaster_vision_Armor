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
        if (serial_.init("/dev/ttyACM0", B115200)) {
            RCLCPP_INFO(this->get_logger(), "串口初始化成功，准备与电控通信...");
        } else {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败！请检查权限或连线。");
        }

        RCLCPP_INFO(this->get_logger(), "识别节点已启动，等待图像...");
        cv::namedWindow("Armor Detect", cv::WINDOW_NORMAL);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<cpp08_armor_detector::msg::ArmorTarget>::SharedPtr target_pub_;
    std::shared_ptr<ArmorDetector> detector_;
    RoboMasterUART serial_;

    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 先按rgb8格式读取相机的原始图像
        cv::Mat img_rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
        // RGB转BGR，解决红蓝蓝红反转，匹配OpenCV的默认格式
        cv::Mat img;
        cv::cvtColor(img_rgb, img, cv::COLOR_RGB2BGR);

        // ================== 【新增】先收电控的当前角度 ==================
        GimbalToVision_Data rx_data;
        if (serial_.receiveGimbal(rx_data)) {
            detector_->setGimbalCurrent(rx_data.Gimbal_Yaw_Current, rx_data.Gimbal_Pitch_Current);
        }
        std::cout<<"当前电控角度: Y="<<rx_data.Gimbal_Yaw_Current<<", P="<<rx_data.Gimbal_Pitch_Current<<std::endl;
        // ==============================================================

        // 运行识别（你原有的代码）
        auto target_msg = detector_->detect(img);
        target_msg.header = msg->header;
        target_pub_->publish(target_msg);

        // ================== 【修改】发绝对角度给电控 ==================
        Manifold_UART_Rx_Data tx_data;
        // 注释/删除错误的帧头帧尾赋值（协议层会自动填充）
        // tx_data.Frame_Header = 0xFF; 
        // tx_data.Frame_Tail = 0xFE;

        if (target_msg.is_detected) {
            // 仅赋值业务数据，发送顺序由结构体内存布局保证
            tx_data.Gimbal_Yaw_Angle = detector_->getTargetYaw();
            tx_data.Gimbal_Pitch_Angle = detector_->getTargetPitch();
            RCLCPP_INFO(this->get_logger(), "发绝对角度: Y=%.2f, P=%.2f", 
                        tx_data.Gimbal_Yaw_Angle, tx_data.Gimbal_Pitch_Angle);
        } else {
            tx_data.Gimbal_Yaw_Angle = 0.0f;
            tx_data.Gimbal_Pitch_Angle = 0.0f;
        }

        if (!serial_.send(tx_data)) {
            RCLCPP_ERROR(this->get_logger(), "串口发送数据失败！");
        } else {
            RCLCPP_INFO(this->get_logger(), "数据发送成功:Y=%.2f, P=%.2f", 
                        tx_data.Gimbal_Yaw_Angle, tx_data.Gimbal_Pitch_Angle);
        }


        // 注释掉imshow，因为在无GUI环境下会出错
        cv::imshow("Armor Detect", img);
        cv::waitKey(1); 
        
        RCLCPP_INFO(this->get_logger(), "图像处理完成，尺寸: %dx%d", img.cols, img.rows);
    }   catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 错误: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    rclcpp::shutdown();
    //cv::destroyAllWindows();
    return 0;
}