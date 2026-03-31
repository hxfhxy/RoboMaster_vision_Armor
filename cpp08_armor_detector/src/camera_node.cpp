#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "cpp08_armor_detector/armor_camera_capture.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), 
      // 2. 关键修复：显式调用命名空间，并传入初始配置参数
      cam_(armor_camera::CameraConfig()) 
    {
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/armor/image_raw", 10);

        // 3. 打开相机
        if (!cam_.open()) {
            RCLCPP_ERROR(this->get_logger(), "无法初始化图像源！");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "相机节点启动成功！");

        timer_ = this->create_wall_timer(
            30ms, std::bind(&CameraNode::publish_img, this)
        );
    }

private:
    void publish_img() {
        cv::Mat frame;
        if (cam_.read(frame)) {
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_frame";
            auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            img_pub_->publish(*img_msg);
        }
    }

    // 4. 关键修复：使用完整的命名空间路径
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