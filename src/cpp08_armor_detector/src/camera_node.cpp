#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

// ========== ROS2 头文件（放在最前面，修复编译错误）==========
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

// ========== OpenCV ==========
#include <opencv2/opencv.hpp>

// ========== 相机驱动 ==========
#include "cpp08_armor_detector/armor_camera_capture.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), 
      cam_(armor_camera::CameraConfig()) 
    {
        // 初始化图像发布者（修复版）
        img_pub_ = create_publisher<sensor_msgs::msg::Image>("/armor/image_raw", 10);

        // 打开相机
        if (!cam_.open()) {
            RCLCPP_ERROR(get_logger(), "无法初始化图像源！");
            return;
        }


        // 定时器
        timer_ = create_wall_timer(30ms, std::bind(&CameraNode::publish_img, this));
    }

private:
    void publish_img() {
        cv::Mat frame;
        if (cam_.read(frame)) {
            // 发布图像
            auto header = std_msgs::msg::Header();
            header.stamp = get_clock()->now();
            header.frame_id = "camera_frame";
            auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            img_pub_->publish(*img_msg);
        }

    }

    // 成员变量
    armor_camera::ArmorCameraCapture cam_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
