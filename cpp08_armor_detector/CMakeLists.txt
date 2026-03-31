cmake_minimum_required(VERSION 3.8)
project(cpp08_armor_detector)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ==================== 0. 外部库开关与宏定义 ====================
option(ARMOR_TRACKER_USE_MVSDK "Use MindVision Industrial Camera SDK" ON)

# ==================== 1. 查找所有依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==================== 2. 编译自定义消息 (ArmorTarget.msg) ====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorTarget.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# ==================== 3. 编译 camera_node (相机驱动节点) ====================
add_executable(camera_node 
  src/camera_node.cpp
  src/armor_camera_capture.cpp
)

# 链接 ROS 2 基础依赖
ament_target_dependencies(camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
)

# 针对 camera_node 的宏定义和库链接
target_link_libraries(camera_node Eigen3::Eigen)

if(ARMOR_TRACKER_USE_MVSDK)
  # 关键：PRIVATE 确保宏定义在编译 src 下的 cpp 时生效
  target_compile_definitions(camera_node PRIVATE ARMOR_TRACKER_USE_MVSDK)
  # 链接迈德威视库
  target_link_libraries(camera_node MVSDK) 
  message(STATUS ">>>> [Building with MindVision SDK Support] <<<<")
else()
  message(STATUS ">>>> [Building with Video File / USB Camera Support] <<<<")
endif()

# 核心修正：添加头文件搜索路径，确保能找到 .hpp
target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

# 确保在编译节点前先生成消息头文件
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_dependencies(camera_node ${cpp_typesupport_target})

# ==================== 4. 编译 detect_node (识别算法节点) ====================
add_executable(detect_node 
  src/detect_node.cpp 
  src/armor_detector.cpp 
  src/armor_detector_preprocess.cpp 
  src/armor_detector_core.cpp 
  src/armor_detector_lightbar.cpp
  src/armor_detector_matching.cpp
  src/armor_detector_visualization.cpp 
  src/armor_detector_tracking.cpp
  src/uart_protocol.cpp  # ✅ 仅在这里追加一行，原有代码全部保留
)

ament_target_dependencies(detect_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV 
  std_msgs
)

target_link_libraries(detect_node Eigen3::Eigen)

# 链接自定义消息类型支持
if(TARGET ${cpp_typesupport_target})
  target_link_libraries(detect_node "${cpp_typesupport_target}")
endif()

target_include_directories(detect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# ==================== 5. 安装与打包 ====================
install(TARGETS
  camera_node
  detect_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
