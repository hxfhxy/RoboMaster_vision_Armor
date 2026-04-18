cmake_minimum_required(VERSION 3.8)
project(cpp08_armor_detector)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ==================== 2. 编译自定义消息 (ArmorTarget.msg) ====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorTarget.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# ==================== 3. 编译 camera_node (【原有】迈德威视/USB相机驱动节点) ====================
# ✅ 【唯一修改】删除了 uart_protocol.cpp 和 uart_work.cpp，相机节点不需要串口！
add_executable(camera_node 
  src/camera_node.cpp
  src/armor_camera_capture.cpp
)

ament_target_dependencies(camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
)

target_link_libraries(camera_node Eigen3::Eigen)

if(ARMOR_TRACKER_USE_MVSDK)
  target_compile_definitions(camera_node PRIVATE ARMOR_TRACKER_USE_MVSDK)
  target_link_libraries(camera_node MVSDK) 
  message(STATUS ">>>> [Building with MindVision SDK Support] <<<<")
else()
  message(STATUS ">>>> [Building with Video File / USB Camera Support] <<<<")
endif()

target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_dependencies(camera_node ${cpp_typesupport_target})

# ==================== 4. 编译 detect_node (识别算法节点) ====================
# ✅ 保留串口代码，只有识别节点需要和电控通信
add_executable(detect_node 
  src/detect_node.cpp 
  src/armor_detector.cpp 
  src/armor_detector_preprocess.cpp 
  src/armor_detector_core.cpp 
  src/armor_detector_lightbar.cpp
  src/armor_detector_matching.cpp
  src/armor_detector_visualization.cpp 
  src/armor_detector_tracking.cpp
  src/uart_protocol.cpp 
  src/uart_work.cpp
  src/armor_ekf.cpp
)

ament_target_dependencies(detect_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV 
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

target_link_libraries(detect_node Eigen3::Eigen)

if(TARGET ${cpp_typesupport_target})
  target_link_libraries(detect_node "${cpp_typesupport_target}")
endif()

target_include_directories(detect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# ==================== 5. 安装与打包 ====================
install(TARGETS
  camera_node           # 原有节点
  detect_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
