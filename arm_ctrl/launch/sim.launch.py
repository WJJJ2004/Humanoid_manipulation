from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 패키지 경로 설정
    pkg_desc = get_package_share_directory('robit_humanoid_offset_22DOF_description')
    pkg_ctrl = get_package_share_directory('arm_ctrl')

    # xacro 파일 경로
    xacro_path = os.path.join(pkg_desc, 'urdf', 'robit_humanoid_offset_22DOF.xacro')
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        # Gazebo 환경 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_desc, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'world': os.path.join(pkg_desc, 'worlds', 'empty.world')
            }.items()
        ),

        # Robot State Publisher 노드 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),

        # Controller Manager 실행
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                os.path.join(pkg_ctrl, 'config', 'controller.yaml')
            ],
            output='screen'
        ),

        # 컨트롤러 스포너
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
    ])
