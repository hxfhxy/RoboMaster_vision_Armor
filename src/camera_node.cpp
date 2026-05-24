#include <chrono>
#include <memory>
#include <string>

// ROS2 头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

// OpenCV 
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// 相机节点类：改装为离线视频播放器
class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        // 初始化图像发布者
        img_pub_ = create_publisher<sensor_msgs::msg::Image>("/armor/image_raw", 10);

        // ==================== 🎯 核心修改：打开本地视频 ====================
        std::string video_path = "/home/hzy/下载/视觉第三轮考核25.12.27/装甲板.mp4";
        cap_.open(video_path);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "视频加载失败，请检查路径是否正确: %s", video_path.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "已成功加载离线视频进行测试！");

        // 创建定时器：33ms触发一次，约等于 30 帧/秒
        timer_ = create_wall_timer(33ms, std::bind(&CameraNode::publish_img, this));
    }

private:
    void publish_img() {
        cv::Mat frame;
        // 如果视频读到了新的一帧
        if (cap_.read(frame)) { 
            cv::resize(frame, frame, cv::Size(640, 480));
            auto header = std_msgs::msg::Header(); 
            header.stamp = get_clock()->now(); 
            header.frame_id = "camera_frame"; 
            
            auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            img_pub_->publish(*img_msg);
            //cv::imshow("Offline Video Player", frame);
            //cv::waitKey(1);
        } else {
            // 如果视频播完了，重置到第 0 帧，实现无限循环播放！
            RCLCPP_INFO(get_logger(), "视频播放完毕，重新开始循环...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        }
    }

    cv::VideoCapture cap_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_; 
    rclcpp::TimerBase::SharedPtr timer_; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<CameraNode>()); 
    rclcpp::shutdown(); 
    return 0;
}