# cpp08_armor_detector
ROS2 Humble 功能包：装甲板视觉识别，实现装甲板定位、PnP解算偏航角/俯仰角/距离、卡尔曼滤波优化偏航角。

# 一、功能说明
- 读取本地MP4视频，按20fps固定帧率发布图像帧；
- 识别装甲板区域，通过PnP解算实时偏航角、俯仰角、距离；
- 卡尔曼滤波优化偏航角，降低数据抖动；
- 支持视频循环播放，识别结果通过ROS2话题发布。

# 节点
节点名	         功能
/camera_node	视频读取 + 图像帧发布
/detect_node	装甲板识别 + 结果发布

# 话题
话题名	            消息类型	                       发布方	              订阅方
/armor/image_raw   sensor_msgs/msg/Image	       /camera_node	       /detect_node
/armor/target      cpp08_armor_detector/msg/ArmorTarget  /detect_node    打开第二个终端接收

# 消息类型
## 自定义消息（ArmorTarget.msg）
bool is_detected          # 是否识别到装甲板
float32 yaw               # 偏航角（度）
float32 pitch             # 俯仰角（度）
float32 distance          # 装甲板距离（mm）
float32 filtered_yaw      # 卡尔曼滤波后的偏航角
float32 center_x          # 装甲板中心像素X
float32 center_y          # 装甲板中心像素Y
## 内置消息
sensor_msgs/msg/Image
传输视频图像帧

# 节点
![节点图](<截图 2026-03-03 14-20-21.png>)

# 运行
（可选）清除上次编译信息 rm -rf build devel

编译  colcon build

加载当前工作空间环境
source install/setup.bash

运行 ros2 launch cpp08_armor_detector armor_launch.py
 
 ros2 launch cpp08_armor_detector armor_launch.py