from launch import LaunchDescription
from launch_ros.actions import Node

# ROS2 Launch文件的入口函数：生成并返回启动描述
def generate_launch_description():
    return LaunchDescription([
        # 1. 启动大恒相机驱动节点
        Node(
            package="galaxy_camera",          # 相机驱动的功能包名
            executable="galaxy_camera_node",   # 可执行文件名
            name="galaxy_camera",              # 节点运行时的名称
            output="screen",                    # 将日志输出到屏幕
            # 相机参数配置（可直接在此调整）
            parameters=[{
                "exposure_time": 15000,        # 曝光时间（单位：微秒）
                "gain": 1.2,                    # 相机增益
                "use_sensor_data_qos": False    # 是否使用传感器数据QoS策略
            }],
            # 话题重映射：将相机默认的"image_raw"话题改为"/armor/image_raw"
            remappings=[
                ("image_raw", "/armor/image_raw")
            ]
        ),

        # 2. 启动装甲板识别节点
        Node(
            package="cpp08_armor_detector",    # 识别算法的功能包名
            executable="detect_node",           # 可执行文件名
            name="detect_node",                 # 节点运行时的名称
            output="screen",                    # 将日志输出到屏幕
            emulate_tty=True                    # 模拟终端（保证彩色日志正常显示）
        )
    ])
