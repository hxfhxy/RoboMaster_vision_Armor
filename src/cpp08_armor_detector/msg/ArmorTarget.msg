# 装甲板识别结果消息
std_msgs/Header header
bool   is_detected      # 是否识别到装甲板
float32 yaw             # 偏航角（度）
float32 pitch           # 俯仰角（度）
float32 distance        # 距离（mm）
float32 filtered_yaw    # 卡尔曼滤波后的偏航角
float32 center_x        # 装甲板中心像素X
float32 center_y        # 装甲板中心像素Y
