from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动现成的大恒相机节点（来自你已经编译好的galaxy_camera包）
        Node(
            package="galaxy_camera",
            executable="galaxy_camera_node",
            name="galaxy_camera",
            output="screen",
            # 这里可以直接调相机参数，装甲板识别建议曝光3000-8000us
            parameters=[{
                "exposure_time": 5000,
                "gain": 1.2,
                "use_sensor_data_qos": False
            }],
            # 【核心】话题重映射：把大恒的/image_raw 改成你识别节点需要的/armor/image_raw
            remappings=[
                ("image_raw", "/armor/image_raw")
            ]
        ),

        # 2. 启动你原来的识别节点，**一行代码都不用改**
        Node(
            package="cpp08_armor_detector",
            executable="detect_node",
            name="detect_node",
            output="screen",
            emulate_tty=True
        )
    ])
