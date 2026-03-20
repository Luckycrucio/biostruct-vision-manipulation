import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# TODO review launch configuration (check git history)

ROBOT_NAME = "abb_irb6700_205_280"
MOVEIT_CONFIG_PACKAGE = "abb_irb6700_205_280_moveit_config"


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    support_package = LaunchConfiguration("support_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # MoveIt configuration
    # - retrieve paths
    support_package_path = get_package_share_directory(f"{support_package.perform(context)}")

    # - build config
    mcb = MoveItConfigsBuilder(robot_name=ROBOT_NAME,
                               package_name=MOVEIT_CONFIG_PACKAGE)
    
    mcb.robot_description() # default path
    mcb.robot_description_semantic() # default path
    mcb.robot_description_kinematics() # default path

    
    mcb.planning_pipelines()

    # MoveIt does not handle controller switching automatically
    mcb.trajectory_execution(
        file_path=os.path.join(
            get_package_share_directory(
                f"{moveit_config_package.perform(context)}"
            ),
            "config",
            "moveit_controllers.yaml",
        ),
        moveit_manage_controllers=False,
    )

    mcb.planning_scene_monitor(publish_planning_scene=True,
                               publish_geometry_updates=True,
                               publish_state_updates=True,
                               publish_transforms_updates=True,
                               publish_robot_description=True,
                               publish_robot_description_semantic=True,
    )

    mcb.joint_limits(
        file_path=os.path.join(
            get_package_share_directory(
                f"{moveit_config_package.perform(context)}"
            ),
            "config",
            "joint_limits.yaml",
        )
    )
    
    moveit_config = mcb.to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory(rviz_config_package.perform(context)), "config"
    )
    rviz_config = os.path.join(rviz_base, rviz_config_file.perform(context))
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base"]
    )

    nodes_to_start = [move_group_node, rviz_node, static_tf_node]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # TODO(andyz): add other options
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="abb_irb6700_205_280.xacro",
            description="Xacro describing the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="abb_irb6700_205_280_support",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="abb_irb6700_205_280_moveit_config",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="abb_irb6700_205_280.srdf",
            description="Name of the SRDF file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="abb_irb6700_205_280_moveit_config",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="moveit.rviz",
            description="Name of the RVIZ config file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
