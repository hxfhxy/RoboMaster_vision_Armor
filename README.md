致谢

本项目数字识别模块使用了由 Chengfu Zou 提供的预训练模型。
模型来源： [https://github.com/baiyeweiguang/Number-Classifier-for-RoboMaster.git]

模型架构： LeNet-5 

感谢： 感谢原作者在 RoboMaster 视觉算法上的开源贡献。

本项目的视觉识别框架参考了 Chen Jun 的开源项目 [(https://github.com/chenjunnn/rm_auto_aim.git)。](https://github.com/chenjunnn/rm_vision.git)

特别感谢陈君大佬对 RoboMaster 算法开源生态的贡献。

# ArmorDetector4

基于 ROS2 Humble 的装甲板视觉识别系统。使用 OpenCV DNN 和传统计算机视觉算法进行灯条检测和装甲板匹配，通过 PnP 算法解算姿态，并与电控系统通信。

## 依赖项

- ROS2 Humble
- OpenCV 4.5+
- cv_bridge, rclcpp, sensor_msgs 等 ROS2 包

## 使用

### 启动工业相机检测
```
ros2 launch cpp08_armor_detector armor_launch_galaxy.py
```

### 启动 USB 相机检测
```
ros2 launch cpp08_armor_detector armor_camera_launch.py
```

节点订阅图像话题，发布检测结果，并通过串口通信。

**功能：**
1. 订阅图像帧并执行识别算法
2. 从串口接收云台当前角度（用于坐标变换）
3. 发布ArmorTarget消息到ROS2网络
4. 通过串口将识别结果发送给电控系统
5. 调试窗口实时显示识别过程

**工作流程：**
```
接收图像 → 串口接收云台角度 → 运行识别 → 发布识别结果 → 串口发送结果 → 显示调试画面
```

---

## 📨 消息定义

### ArmorTarget.msg - 识别结果消息

```protobuf
std_msgs/Header header               # ROS消息头（时间戳、坐标系）
bool is_detected                     # 是否成功识别装甲板
float32 yaw                          # 偏航角 [度]
float32 pitch                        # 俯仰角 [度]
float32 distance                     # 目标距离 [毫米]
float32 filtered_yaw                 # 卡尔曼滤波后的偏航角 [度]
float32 center_x                     # 装甲板中心像素 X坐标
float32 center_y                     # 装甲板中心像素 Y坐标
```

---

## 🔧 核心算法模块

### 1️⃣ 预处理 (`armor_detector_preprocess.cpp`)
- HSV色彩空间分割（飽和度）
- 形态学操作（开运算、闭运算）
- 二值化处理

### 2️⃣ 灯条检测 (`armor_detector_lightbar.cpp`)
- 轮廓检测与灯条提取
- 灯条形状验证（长宽比、面积）
- 灯条方向计算

### 3️⃣ 灯条匹配 (`armor_detector_matching.cpp`)
- 灯条对配对（基于位置、角度、大小）
- 候选装甲板生成
- 装甲板有效性验证

### 4️⃣ 特征识别 (`armor_detector_core.cpp`)
- 数字识别或装甲板分类
- PnP算法进行3D姿态估计
- 角度和距离计算

### 5️⃣ 目标追踪 (`armor_detector_tracking.cpp`)
- 卡尔曼滤波器平滑偏航角
- 多目标追踪管理
- 目标生命周期管理

### 6️⃣ 可视化 (`armor_detector_visualization.cpp`)
- 在图像上绘制灯条、装甲板框、坐标系
- 显示PnP计算结果
- 调试信息显示

---

## 💾 相机驱动支持

系统支持多种图像源，通过编译选项切换：

| 驱动类型 | CMake选项 | 说明 |
|---------|----------|------|
| 本地视频 | 默认 | MP4视频文件循环播放 |
| USB相机 | 默认 | 系统V4L2设备 |
| 迈德威视 | `ARMOR_TRACKER_USE_MVSDK=ON` | 工业相机SDK支持 |

---

## 🔌 串口通信协议

### 传输格式

**接收数据 (云台→视觉)：**
```c
struct GimbalToVision_Data {
    float Gimbal_Yaw_Current;      // 当前偏航角 [度]
    float Gimbal_Pitch_Current;    // 当前俯仰角 [度]
};
```

**发送数据 (视觉→电控)：**
```c
struct Manifold_UART_Rx_Data {
    float Gimbal_Yaw_Angle;        // 目标偏航角 [度]
    float Gimbal_Pitch_Angle;      // 目标俯仰角 [度]
};
```

**串口配置：**
- 端口：`/dev/ttyACM0`
- 波特率：`115200`
- 数据位：8
- 停止位：1
- 校验位：无

---

## 📦 依赖项

### 系统依赖
```bash
# ROS2相关
rclcpp                  # ROS2 C++客户端库
sensor_msgs            # 传感器消息类型
cv_bridge              # OpenCV与ROS消息转换
opencv_viz             # OpenCV可视化组件
std_msgs               # 基础消息类型

# 算法库
opencv                 # 计算机视觉(>= 4.5)
eigen3                 # 线性代数库
```

### 编译配置（CMakeLists.txt）
```cmake
# 核心依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)

# 可选：迈德威视SDK
option(ARMOR_TRACKER_USE_MVSDK "Use MindVision Industrial Camera SDK" ON)
```

---

## 🚀 快速开始

### 1. 环境准备

```bash
# 切换到工作空间根目录
cd ~/ArmorDetector1

# 清除旧的编译数据（可选）
rm -rf build install log

# 安装ROS2依赖
sudo apt-get install ros-humble-sensor-msgs ros-humble-cv-bridge
```

### 2. 编译项目

```bash
cd ~/ArmorDetector1

# 方式1：仅编译此功能包
colcon build --packages-select cpp08_armor_detector

# 方式2：编译整个工作空间
colcon build
```

**编译选项：**
```bash
# 启用迈德威视SDK支持
colcon build --packages-select cpp08_armor_detector \
  -DARMOR_TRACKER_USE_MVSDK=ON

# 启用调试信息和日志
colcon build --packages-select cpp08_armor_detector \
  -DCMAKE_BUILD_TYPE=Debug
```

### 3. 加载环境变量

```bash
# 设置ROS环境
source ~/ArmorDetector1/install/setup.bash

# 验证环境
echo $AMENT_PREFIX_PATH
```

### 4. 运行节点

**标准启动（USB相机/视频文件）：**
```bash
ros2 launch cpp08_armor_detector armor_camera_launch.py
```

**大恒相机启动：**
```bash
ros2 launch cpp08_armor_detector armor_launch_galaxy.py
```

**手动启动两个终端：**

终端1（图像采集）：
```bash
ros2 run cpp08_armor_detector camera_node
```

终端2（识别处理）：
```bash
ros2 run cpp08_armor_detector detect_node
```

### 5. 查看识别结果

```bash
# 监听识别话题
ros2 topic echo /armor/target

# 实时绘制话题图形
ros2 run rqt_graph rqt_graph

# 查看图像
ros2 run rqt_image_view rqt_image_view
```

---

## 🐛 调试与故障排除

### 常见问题

| 问题 | 原因 | 解决方案 |
|-----|------|--------|
| `无法初始化图像源` | 相机未连接或权限不足 | 检查USB连接或运行`sudo chmod 666 /dev/video*` |
| `串口初始化失败` | 设备不存在或忙碌 | 检查`/dev/ttyACM0`是否存在，确保未被占用 |
| `识别效果不佳` | 光照条件差或参数未优化 | 调整HSV阈值范围或PnP参数 |
| `编译错误：找不到包` | 依赖未安装或工作空间未sourced | 运行`rosdep install --from-paths src -y --ignore-src` |

### 日志输出

```bash
# 查看节点日志
ROS_LOG_DIR=/tmp colcon log view --last

# 实时监控日志
ros2 run cpp08_armor_detector detect_node --ros-args --log-level INFO

# 详细调试日志
ros2 run cpp08_armor_detector detect_node --ros-args --log-level DEBUG
```

---

## ⚙️ 参数调优指南

### 图像预处理参数
编辑 `armor_detector.cpp` 中的：
- **HSV阈值**：根据光照条件调整红/蓝灯条的H、S、V范围
- **形态学内核**：修改`cv::getStructuringElement()`的大小和形状
- **二值化阈值**：适应不同的对比度

### 灯条识别参数
- **灯条长宽比范围**：`LIGHTBAR_ASPECT_RATIO_MIN/MAX`
- **灯条面积范围**：`LIGHTBAR_AREA_MIN/MAX`
- **灯条方向偏差**：`LIGHTBAR_ANGLE_TOLERANCE`

### 装甲板匹配参数
- **灯条间距比例**：`LIGHTBAR_DISTANCE_RATIO`
- **装甲板长宽比**：`ARMOR_ASPECT_RATIO_TOLERANCE`

### 卡尔曼滤波器参数
编辑 `kalman_filter.hpp`：
- **过程噪声**：`Q` - 越大对当前测量信任度越高
- **测量噪声**：`R` - 越大越平滑但延迟越大

---

## 📝 代码规范

### 命名约定
- 类名：PascalCase（如 `ArmorDetector`）
- 函数名：snake_case（如 `detect_armor()`）
- 成员变量：snake_case带后缀_（如 `img_sub_`、`detector_`）

### 重要接口

```cpp
// 核心检测接口
cpp08_armor_detector::msg::ArmorTarget detect(const cv::Mat& img);

// 获取识别结果
float getTargetYaw();
float getTargetPitch();

// 设置云台当前角度（用于坐标变换）
void setGimbalCurrent(float yaw, float pitch);
```

---

## 🔄 工作流程示例

```
1. camera_node 启动
   ↓ 打开相机
   ↓ 创建图像发布者 (/armor/image_raw)
   ↓ 按30ms周期读取帧并发布

2. detect_node 启动
   ↓ 打开串口 (/dev/ttyACM0@115200)
   ↓ 订阅图像话题 (/armor/image_raw)
   ↓ 创建识别结果发布者 (/armor/target)

3. 每一帧处理：
   a. 接收ROS图像消息
   b. 从串口读取云台当前角度（Yaw_current, Pitch_current）
   c. 执行识别算法：
      - 颜色分割 → 灯条提取 → 灯条匹配 → PnP解算
   d. 卡尔曼滤波平滑偏航角
   e. 发布识别结果 (bool is_detected, yaw, pitch, distance, ...)
   f. 打包并通过串口发送 (Gimbal_Yaw_Angle, Gimbal_Pitch_Angle)
   g. 在窗口显示调试信息
```

---

## 📚 相关资源

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [OpenCV计算机视觉](https://docs.opencv.org/)
- [PnP问题求解](https://en.wikipedia.org/wiki/Perspective-n-Point)
- [卡尔曼滤波原理](https://en.wikipedia.org/wiki/Kalman_filter)

---

## 📄 许可证

Apache-2.0

## 👤 维护者

hzy
 
