from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    # --------------------------------------------------------------------------
    # Declare launch arguments
    # --------------------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of the UR robot.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="127.0.0.1",
            description="IP address of the UR robot.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware for simulation.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the robot driver.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value="",        
            description="Path to kinematics calibration YAML. Leave blank to skip.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="ur5e_robot_bringup",
            description="Package containing the robot URDF/xacro description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="ur5e.urdf.xacro",
            description="URDF/xacro file for the robot description.",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur5e_robot_moveit_config",
            description="MoveIt 2 configuration package.",
        ),
        DeclareLaunchArgument(
            "launch_moveit_rviz",
            default_value="true",
            description="Launch MoveIt RViz.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller:",
            default_value="scaled_joint_trajectory_controller",
            description="Initial joint controller.",
        ),
    ]

    # --------------------------------------------------------------------------
    # Launch configurations
    # --------------------------------------------------------------------------
    ur_type                     = LaunchConfiguration("ur_type")
    robot_ip                    = LaunchConfiguration("robot_ip")
    use_fake_hardware           = LaunchConfiguration("use_fake_hardware")
    launch_rviz                 = LaunchConfiguration("launch_rviz")
    kinematics_params_file      = LaunchConfiguration("kinematics_params_file")
    description_package         = LaunchConfiguration("description_package")
    description_file            = LaunchConfiguration("description_file")
    moveit_config_package       = LaunchConfiguration("moveit_config_package")
    launch_moveit_rviz          = LaunchConfiguration("launch_moveit_rviz")
    initial_joint_controller    = LaunchConfiguration("initial_joint_controller")

    # --------------------------------------------------------------------------
    # 1. UR Robot Driver
    # --------------------------------------------------------------------------
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type":                      ur_type,
            "robot_ip":                     robot_ip,
            "use_fake_hardware":            use_fake_hardware,
            "launch_rviz":                  launch_rviz,
            "kinematics_params_file":       kinematics_params_file,
            "description_package":          description_package,
            "description_file":             description_file,
            "initial_joint_controller":     initial_joint_controller,
        }.items(),
    )

    # --------------------------------------------------------------------------
    # 2. MoveIt 2 — move_group
    # --------------------------------------------------------------------------
    
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                "launch",
                "move_group.launch.py",
            ])
        ),
    )

    # --------------------------------------------------------------------------
    # 3. MoveIt 2 — RViz
    # --------------------------------------------------------------------------
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                "launch",
                "moveit_rviz.launch.py",
            ])
        ),
        condition=IfCondition(launch_moveit_rviz),
    )
    
    return LaunchDescription(
        declared_arguments + [
            ur_control_launch,
            move_group_launch,
            moveit_rviz_launch,
        ]
    )