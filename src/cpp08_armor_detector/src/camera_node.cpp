#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

//ROS2 头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

// OpenCV 
#include <opencv2/opencv.hpp>

//相机驱动
#include "cpp08_armor_detector/armor_camera_capture.hpp"

using namespace std::chrono_literals;

// 相机节点类：负责从相机采集图像并发布到ROS2话题
class CameraNode : public rclcpp::Node {
public:
    // 构造函数：初始化节点、相机、发布者和定时器
    CameraNode() : Node("camera_node"), 
      cam_(armor_camera::CameraConfig()) // 初始化相机对象（使用默认配置）
    {
        // 初始化图像发布者：话题名为"/armor/image_raw"，队列长度为10
        img_pub_ = create_publisher<sensor_msgs::msg::Image>("/armor/image_raw", 10);

        // 打开相机
        if (!cam_.open()) {
            RCLCPP_ERROR(get_logger(), "无法初始化图像源！"); // 相机打开失败，打印错误日志
            return;
        }


        // 创建定时器：每30ms触发一次publish_img回调函数（约33fps）
        timer_ = create_wall_timer(30ms, std::bind(&CameraNode::publish_img, this));
    }

private:
    // 定时器回调函数：采集图像并发布
    void publish_img() {
        cv::Mat frame;
        if (cam_.read(frame)) { // 从相机读取一帧图像
            // 发布图像
            auto header = std_msgs::msg::Header(); // 创建消息头
            header.stamp = get_clock()->now(); // 填充时间戳
            header.frame_id = "camera_frame"; // 填充坐标系ID
            
            // 将OpenCV的Mat图像转换为ROS2的Image消息（编码格式为bgr8）
            auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            
            // 发布图像消息
            img_pub_->publish(*img_msg);
        }

    }

    // 成员变量
    armor_camera::ArmorCameraCapture cam_; // 相机捕获对象
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_; // 图像发布者
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // 初始化ROS2
    rclcpp::spin(std::make_shared<CameraNode>()); // 运行相机节点，进入事件循环
    rclcpp::shutdown(); // 关闭ROS2
    return 0;
}
