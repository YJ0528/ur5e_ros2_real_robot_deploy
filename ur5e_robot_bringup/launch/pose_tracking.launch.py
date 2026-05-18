from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bringup_package_name =                      "ur5e_robot_bringup"
    moveit_config_package_name =                "ur5e_robot_moveit_config"
    moveit_control_package_name =               "moveit_control" 

    bringup_package_dir=                        get_package_share_directory(bringup_package_name) 
    moveit_config_package_dir=                  get_package_share_directory(moveit_config_package_name)
    moveit_control_package_dir=                 get_package_share_directory(moveit_control_package_name)

    pose_tracking_settings_config = os.path.join(
        moveit_control_package_dir, 
        "config", 
        "pose_tracking_settings.yaml"
    )
    ur_servo_config = os.path.join(
        moveit_control_package_dir, 
        "config", 
        "ur_servo.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5e_robot")
        .to_moveit_configs()
    )

    pose_tracking_node = Node(
        package=moveit_control_package_name,
        executable="realtime_pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            pose_tracking_settings_config,
            ur_servo_config,
        ],
    )

    return LaunchDescription([
        pose_tracking_node,
    ])