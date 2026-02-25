from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml


def get_robot_description_planning():
    """Load joint_limits and pass as robot_description_planning so RobotModel gets acceleration limits."""
    def load_joint_limits(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        if "move_group" in data and "ros__parameters" in data["move_group"]:
            return data["move_group"]["ros__parameters"]["joint_limits"]
        return data.get("joint_limits", data)

    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory("moveit_config")
        path = os.path.join(pkg_dir, "config", "joint_limits.yaml")
        joint_limits = load_joint_limits(path)
        return {"robot_description_planning": {"joint_limits": joint_limits}}
    except Exception:
        try:
            # ソース空間で実行時（install 前）
            path = os.path.join(os.path.dirname(__file__), "..", "config", "joint_limits.yaml")
            path = os.path.abspath(path)
            if os.path.isfile(path):
                joint_limits = load_joint_limits(path)
                return {"robot_description_planning": {"joint_limits": joint_limits}}
        except Exception:
            pass
    return {}


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Get launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get package paths
    robot_description_pkg = FindPackageShare("robot_description")
    moveit_config_pkg = FindPackageShare("moveit_config")

    # URDF file path
    urdf_file = PathJoinSubstitution([
        robot_description_pkg,
        "urdf",
        "my_robot.urdf.xacro",
    ])

    # SRDF file path
    srdf_file = PathJoinSubstitution([
        moveit_config_pkg,
        "config",
        "2dof_arm.srdf",
    ])

    # Robot description
    robot_description_content = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_file]),
            value_type=str
        ),
    }

    # Robot description semantic
    robot_description_semantic_content = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str
        ),
    }

    # Joint limits
    joint_limits_file = PathJoinSubstitution([
        moveit_config_pkg,
        "config",
        "joint_limits.yaml",
    ])
    kinematics_file = PathJoinSubstitution([
        moveit_config_pkg,
        "config",
        "kinematics.yaml",
    ])

    # Planning pipeline（ファイルパスで渡す。planning_pipelines.yaml に response_adapters は ValidateSolution のみ記載済み）
    planning_pipelines_file = PathJoinSubstitution([
        moveit_config_pkg,
        "config",
        "planning_pipelines.yaml",
    ])

    # 軌道実行は無効。デモは plan_only で計画を取得し、/joint_states で可視化するため実機コントローラ不要
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    # move_action を /move_group/move_action にリマップ → クライアントが一意に接続でき joint2 が正しく動く
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        remappings=[("move_action", "/move_group/move_action")],
        parameters=[
            robot_description_content,
            robot_description_semantic_content,
            get_robot_description_planning(),
            joint_limits_file,
            kinematics_file,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            planning_pipelines_file,
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node])
