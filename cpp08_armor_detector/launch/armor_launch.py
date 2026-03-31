from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 图像发布节点
        Node(
            package="cpp08_armor_detector",
            executable="camera_node",
            name="camera_node",
            output="screen"
        ),
        # 装甲板识别节点
        Node(   
            package="cpp08_armor_detector",
            executable="detect_node",
            name="detect_node",
            output="screen"
        )
    ])
