from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        os.getenv('HOME'), 'colcon_ws', 'src', 'arm_ctrl', 'config', 'arm_params.yaml'
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

