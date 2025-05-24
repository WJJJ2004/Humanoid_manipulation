from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('arm_ctrl'),
        'config',
        'arm_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='arm_ctrl',
            executable='main_node',
            name='main_node',
            parameters=[config_path],
            output='screen'
        )
    ])
