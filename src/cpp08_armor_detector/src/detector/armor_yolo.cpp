#include "cpp08_armor_detector/detector/armor_yolo.hpp"
#include <cmath>

/**
 * @brief 装甲板YOLO检测器构造函数
 * @note 初始化空网络，后续需调用loadModel加载模型
 */
ArmorYolo::ArmorYolo() {}

/**
 * @brief 加载YOLO ONNX模型
 * @param model_path ONNX模型文件路径
 */
void ArmorYolo::loadModel(const std::string& model_path) {
    // 使用OpenCV DNN模块读取ONNX格式的YOLO模型
    net_ = cv::dnn::readNetFromONNX(model_path);
}

/**
 * @brief 执行装甲板检测
 * @param img 输入图像（BGR格式，OpenCV默认）
 * @return std::vector<DetectedArmor> 检测到的装甲板列表
 */
std::vector<DetectedArmor> ArmorYolo::detect(const cv::Mat& img) {
    // 最终返回的有效装甲板列表
    std::vector<DetectedArmor> final_armors;
    
    // 异常处理：图像为空或网络未加载，直接返回空列表
    if (img.empty() || net_.empty()) {
        return final_armors;
    }

    // 1. 图像预处理：转换为网络输入格式（Blob）
    // blobFromImage参数说明：
    // - 1.0/255.0：像素值归一化到[0,1]
    // - cv::Size(INPUT_WIDTH, INPUT_HEIGHT)：网络输入尺寸（如640x640）
    // - cv::Scalar(0,0,0)：均值减法（YOLO通常不需要，设为0）
    // - true：交换RB通道（OpenCV是BGR，YOLO是RGB）
    // - false：不裁剪图像，保持比例缩放
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0,
                                          cv::Size(INPUT_WIDTH, INPUT_HEIGHT),
                                          cv::Scalar(0, 0, 0), true, false);
    
    // 设置网络输入
    net_.setInput(blob);

    // 2. 网络前向推理 
    // 存储网络所有输出层的结果
    std::vector<cv::Mat> net_outputs;
    // 执行前向传播，获取所有未连接输出层的结果（YOLO只有一个输出层）
    net_.forward(net_outputs, net_.getUnconnectedOutLayersNames());
    
    // 网络输出为空，直接返回
    if (net_outputs.empty()) {
        return final_armors;
    }

    // 提取第一个（也是唯一一个）输出层的结果
    // YOLO输出形状：[1, num_anchors, 22]，其中22是每个锚点的输出维度
    cv::Mat output = net_outputs[0];
    // 将输出转换为二维Mat方便访问：[num_anchors, 22]
    cv::Mat output_buffer(output.size[1], output.size[2], CV_32F, output.ptr<float>());

    // 临时存储：用于NMS的包围盒、置信度，以及所有检测到的装甲板
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<DetectedArmor> temp_armors;

    // 坐标缩放因子：将网络输出的相对坐标转换为原始图像的绝对坐标
    float x_factor = img.cols / static_cast<float>(INPUT_WIDTH);
    float y_factor = img.rows / static_cast<float>(INPUT_HEIGHT);

    // 3. 遍历所有检测结果，解析输出 
    for (int i = 0; i < output_buffer.rows; ++i) {
        // 第8列：目标存在置信度（logits形式，需sigmoid转换为概率）
        float confidence = output_buffer.at<float>(i, 8);
        // Sigmoid激活：将logits转换为[0,1]的概率
        confidence = 1.0f / (1.0f + std::exp(-confidence));
        
        // 置信度低于阈值，直接跳过该检测
        if (confidence < conf_threshold_) {
            continue;
        }

        // 解析分类得分：
        // 第9-12列（共4个）：颜色分类得分（0:蓝, 1:红, 2:无效, 3:无效）
        cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);
        // 第13-21列（共9个）：数字分类得分（0-8对应数字1-9）
        cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22);

        // 找到得分最高的数字ID和颜色ID
        cv::Point class_id, color_id;
        double score_num = 0.0;   // 最高数字得分
        double score_color = 0.0; // 最高颜色得分
        cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
        cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

        // 颜色过滤逻辑 
        // 跳过无效颜色（ID=2/3）
        if (color_id.x == 2 || color_id.x == 3) {
            continue;
        }
        // detect_color_=0：只检测蓝色，跳过红色（ID=1）
        if (detect_color_ == 0 && color_id.x == 1) {
            continue;
        }
        // detect_color_=1：只检测红色，跳过蓝色（ID=0）
        if (detect_color_ == 1 && color_id.x == 0) {
            continue;
        }

        // 解析装甲板四个角点坐标 
        // 第0-7列：四个角点的x,y坐标（相对于网络输入尺寸）
        cv::Point2f p1_lt(output_buffer.at<float>(i, 0) * x_factor,
                          output_buffer.at<float>(i, 1) * y_factor); // 左上角
        cv::Point2f p2_lb(output_buffer.at<float>(i, 2) * x_factor,
                          output_buffer.at<float>(i, 3) * y_factor); // 左下角
        cv::Point2f p3_rb(output_buffer.at<float>(i, 4) * x_factor,
                          output_buffer.at<float>(i, 5) * y_factor); // 右下角
        cv::Point2f p4_rt(output_buffer.at<float>(i, 6) * x_factor,
                          output_buffer.at<float>(i, 7) * y_factor); // 右上角

        // 计算四个角点的最小包围矩形（用于NMS）
        float min_x = std::min({p1_lt.x, p2_lb.x, p3_rb.x, p4_rt.x});
        float max_x = std::max({p1_lt.x, p2_lb.x, p3_rb.x, p4_rt.x});
        float min_y = std::min({p1_lt.y, p2_lb.y, p3_rb.y, p4_rt.y});
        float max_y = std::max({p1_lt.y, p2_lb.y, p3_rb.y, p4_rt.y});
        cv::Rect bounding_box(min_x, min_y, max_x - min_x, max_y - min_y);

        // 构造检测结果结构体 
        DetectedArmor armor;
        armor.pnp_corners = {p1_lt, p2_lb, p3_rb, p4_rt}; // PnP解算用的四个角点
        armor.number = class_id.x;                        // 识别出的数字ID
        armor.class_confidence = static_cast<float>(score_num); // 数字分类置信度
        armor.confidence = confidence;                    // 目标存在置信度
        armor.rect = cv::minAreaRect(armor.pnp_corners);  // 装甲板的最小外接旋转矩形

        // 保存到临时列表，等待NMS过滤
        boxes.push_back(bounding_box);
        confidences.push_back(confidence);
        temp_armors.push_back(armor);
    }

    // 4. 非极大值抑制（NMS）：去除重复检测框
    std::vector<int> indices;
    if (!boxes.empty()) {
        // NMS参数：置信度阈值、IOU阈值
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
    }

    // 5. 整理最终检测结果
    // 计算图像中心坐标（用于计算装甲板到中心的距离）
    cv::Point2f imgCenter(img.cols / 2.0f, img.rows / 2.0f);
    
    // 遍历NMS后的有效索引
    for (int valid_index : indices) {
        // 边界检查：防止索引越界
        if (valid_index < 0 || valid_index >= static_cast<int>(temp_armors.size())) {
            continue;
        }
        
        DetectedArmor armor = temp_armors[valid_index];
        // 计算装甲板中心到图像中心的距离（用于后续优先选择最近的装甲板）
        armor.dist_to_center = cv::norm(armor.rect.center - imgCenter);
        // 添加到最终结果列表
        final_armors.push_back(std::move(armor));
    }

    return final_armors;
}
