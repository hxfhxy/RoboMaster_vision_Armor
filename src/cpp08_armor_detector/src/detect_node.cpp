/**
 * @file detect_node.cpp
 * @brief 离线视频纯净测试版：一键读取视频 -> 跑算法 -> 弹窗显示
 */

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "cpp08_armor_detector/armor_detector.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

class StandaloneDetectNode : public rclcpp::Node {
public: 
    StandaloneDetectNode() : Node("standalone_detect_node") {
        // 1. 初始化 TF (维持内部运算需要)
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_armor_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 2. 初始化核心检测器
        detector_ = std::make_shared<ArmorDetector>();
        detector_->setTFBuffer(tf_buffer_); 
        detector_->setArmorTFBroadcaster(tf_armor_broadcaster_); 
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/armor/velocity_marker", 10);
        detector_->setMarkerPublisher(marker_pub_);
        
        // 3. 加载模型
        detector_->loadModel("/home/hzy/ArmorDetector1/src/cpp08_armor_detector/model/0526.onnx"); 

        // 3. 打开本地测试视频
        std::string video_path = "/home/hzy/下载/robomaster测试视频/测试3.mp4";
        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "视频打开失败");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "成功加载视频");

        // 4. 设置定时器 (33ms 一帧)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), 
            std::bind(&StandaloneDetectNode::timer_callback, this));
        
        cv::namedWindow("Standalone Video Test", cv::WINDOW_NORMAL);
    }

private:
    void timer_callback() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "视频播放完毕，循环重置...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        cv::resize(frame, frame, cv::Size(640, 480)); 

        auto timestamp = this->now();

        // 伪造云台TF
        geometry_msgs::msg::TransformStamped dynamic_tf;
        dynamic_tf.header.stamp = timestamp;
        dynamic_tf.header.frame_id = "world_frame";
        dynamic_tf.child_frame_id = "camera_frame";
        dynamic_tf.transform.translation.x = 0.0;
        dynamic_tf.transform.translation.y = 0.0;
        dynamic_tf.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        dynamic_tf.transform.rotation.x = q.x();
        dynamic_tf.transform.rotation.y = q.y();
        dynamic_tf.transform.rotation.z = q.z();
        dynamic_tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(dynamic_tf);

        // 运行核心检测算法
        detector_->detect(frame, timestamp);

        // 弹出画面
        cv::imshow("Standalone Video Test", frame);
        
        cv::waitKey(0); 
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<ArmorDetector> detector_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_armor_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandaloneDetectNode>());
    rclcpp::shutdown();
    return 0;
}