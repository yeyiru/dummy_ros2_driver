import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dummy_dir = get_package_share_directory('dummy_controller')
    fingerprinter_cam_config = os.path.join(dummy_dir, 'config', 'fingerprinter_camera.yaml')
    # rviz_config = os.path.join(dummy_dir, 'rviz_cfg', 'calib.rviz')

    return LaunchDescription([
        # 启动 LiDAR-Camera 校准节点
        Node(
            package='dummy_controller',
            executable='dummy_controller',
            name='dummy_controller',
            output='screen',
            # parameters=[dummy_config]  # 直接传递参数文件
        ),
        # 启动 USB Camera 节点
        Node(
            package='dummy_controller',
            executable='fingerprint_camera',
            name='fingerprinter_cam_publisher',
            output='screen',
            parameters=[fingerprinter_cam_config],  # 直接传递参数文件
        ),
        # # 启动 RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     output="screen",
        #     arguments=['-d', rviz_config]
        # )
    ])