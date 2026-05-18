import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    # pkg_path = get_package_share_directory('ur5e_robot_bringup')
    # urdf_file = os.path.join(pkg_path, 'urdf', 'ur5e.urdf.xacro')

    # robot_description = Command(['xacro ', urdf_file])

    pkg_path = get_package_share_directory('ur5e_robot_bringup')
    urdf_file = os.path.join(pkg_path, 'urdf', 'ur5e.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()  

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0,      # ← add this
                'use_sim_time': False            # ← add this
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])