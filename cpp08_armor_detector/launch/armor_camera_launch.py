import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明你自己写的相机驱动节点
    camera_node = Node(
        package='cpp08_armor_detector',
        executable='camera_node',  # 对应 CMakeLists 里的 add_executable 名称
        name='camera_driver',
        output='screen',
        parameters=[{
            'use_sensor_data_qos': False,
            # 如果你的代码里写了读取参数的逻辑，可以在这里添加
        }]
    )

    # 2. 声明装甲板识别节点
    armor_detection_node = Node(
        package='cpp08_armor_detector',
        executable='detect_node',
        name='armor_detector',
        output='screen',
        emulate_tty=True
    )

    # 3. 组装并返回
    return LaunchDescription([
        camera_node,
        armor_detection_node
    ])