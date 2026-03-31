#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cpp08_armor_detector/msg/armor_target.hpp" // 引用自定义消息
#include "cpp08_armor_detector/armor_detector.hpp"

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

        RCLCPP_INFO(this->get_logger(), "识别节点已启动，等待图像...");
        //cv::namedWindow("Armor Detect", cv::WINDOW_NORMAL);
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 转换图像，使用 toCvCopy 保证安全，避免修改原始缓冲区
            cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // 调用算法逻辑
            auto target_msg = detector_->detect(img);
            
            // 填充消息的时间戳（重要：保持时间戳同步）
            target_msg.header = msg->header;
            target_pub_->publish(target_msg);

            // 调试显示
            cv::imshow("Armor Detect", img);
            // cv::waitKey(1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<cpp08_armor_detector::msg::ArmorTarget>::SharedPtr target_pub_;
    std::shared_ptr<ArmorDetector> detector_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectNode>());
    rclcpp::shutdown();
    //cv::destroyAllWindows();
    return 0;
}