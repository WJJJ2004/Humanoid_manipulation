from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    desc_pkg = FindPackageShare("robit_humanoid_offset_22DOF_description")
    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "robit_humanoid_offset_22DOF.xacro"])
    rviz_config_file = PathJoinSubstitution([desc_pkg, "config", "display.rviz"])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file
        ])
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        )
    ])


# from launch.substitutions import Command
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.actions import Node
# from launch import LaunchDescription
# from launch_ros.substitutions import PathJoinSubstitution

# def generate_launch_description():
#     robot_description_content = Command([
#         'xacro ',
#         PathJoinSubstitution([
#             FindPackageShare('robit_humanoid_offset_22DOF_description'),
#             'urdf',
#             'robit_humanoid_offset_22DOF.xacro'
#         ])
#     ])
#     robot_description = {'robot_description': robot_description_content}

#     return LaunchDescription([
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             parameters=[robot_description],
#             output='screen'
#         ),
#         Node(
#             package='joint_state_publisher_gui',
#             executable='joint_state_publisher_gui',
#             output='screen'
#         ),
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             arguments=['-d', PathJoinSubstitution([
#                 FindPackageShare('robit_humanoid_offset_22DOF_description'),
#                 'config',
#                 'display.rviz'
#             ])],
#             output='screen'
#         )
#     ])
