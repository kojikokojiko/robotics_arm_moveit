from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch引数
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher', default='false')
    
    # パッケージのパスを取得
    robot_description_pkg = FindPackageShare('robot_description')
    
    # URDFファイルのパス
    urdf_file = PathJoinSubstitution([
        robot_description_pkg,
        'urdf',
        'my_robot.urdf.xacro'
    ])

    # Robot State Publisher
    # ParameterValueを使って文字列として明示的に指定
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            )
        }]
    )

    # Joint State Publisher（オプション、デフォルトでは起動しない）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_state_publisher)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_joint_state_publisher', default_value='false'),
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
