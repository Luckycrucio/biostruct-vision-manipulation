"""
Luca Grigolin at PROFACTOR GmbH

A launch file for moveIt and rviz-visualization for the 
Planner and Executor nodes.
"""

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
from moveit_configs_utils import MoveItConfigsBuilder, MoveItConfigs


#TODO replace with proper reflection
PACKAGE_NAME = "biostruct_amura"

ROBOT_NAME = "abb_irb6700_205_280"
MOVEIT_CONFIG_PACKAGE = "abb_irb6700_205_280_moveit_config"
package_path = get_package_share_directory(PACKAGE_NAME)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_moveit_config(moveit_config_package: str) -> MoveItConfigs:
    # MoveIt configuration
    # - retrieve paths
    moveit_config_package_path = get_package_share_directory(moveit_config_package)
    trajectory_execution_config_path = os.path.join("config", "moveit_controllers.yaml")
    #joint_limits_config_path = os.path.join("config", "joint_limits.yaml") # default
    moveit_cpp_config_path = os.path.join(package_path, "config", "moveit_cpp.yaml")

    # - build config
    mcb = MoveItConfigsBuilder(robot_name=ROBOT_NAME,
                               package_name=moveit_config_package)
    
    mcb.robot_description() # default path
    mcb.robot_description_semantic() # default path
    mcb.robot_description_kinematics() # default path
    # TODO review - required?
    mcb.joint_limits() # default path

    mcb.planning_scene_monitor(publish_planning_scene=True,
                               publish_geometry_updates=True,
                               publish_state_updates=True,
                               publish_transforms_updates=True,
                               publish_robot_description=True,
                               publish_robot_description_semantic=True,
    )

    mcb.trajectory_execution(file_path=trajectory_execution_config_path,
                             moveit_manage_controllers=False)
    
    mcb.planning_pipelines(pipelines=["pilz_industrial_motion_planner"])          
    #mcb.moveit_cpp(file_path=moveit_cpp_config_path)
    return mcb.to_moveit_configs()


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    arg_moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_package = f"{arg_moveit_config_package.perform(context)}"
    moveit_config = load_moveit_config(moveit_config_package)                         
    
    rviz_config = os.path.join(package_path, "config", "moveit.rviz")
    
    # NODES
    # - move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # - rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value=MOVEIT_CONFIG_PACKAGE,
            description="Name of the moveit config package",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
