"""
MoveIt demo launch

controller_list 等のネストした dict は launch の parameters で扱えないため、
MoveItConfigsBuilder.to_dict() ではなく move_group.launch.py（ファイルパス渡し）を利用
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = PathJoinSubstitution([
        FindPackageShare("robot_description"), "urdf", "my_robot.urdf.xacro",
    ])

    # Robot State Publisher（URDF から robot_description を発行）
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_file]),
                    value_type=str,
                ),
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    # move_group はファイルパス渡しの launch を使用（controller_list の dict を避ける）
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(Path(get_package_share_directory("moveit_config")) / "launch" / "move_group.launch.py"),
        ]),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            move_group_launch,
        ]
    )
