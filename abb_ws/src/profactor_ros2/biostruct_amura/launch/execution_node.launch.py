"""
Luca Grigolin at PROFACTOR GmbH

A launch file for the Executor node.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder, MoveItConfigs

PACKAGE_NAME = "biostruct_amura"

ROBOT_NAME = "abb_irb6700_205_280"
MOVEIT_CONFIG_PACKAGE = "abb_irb6700_205_280_moveit_config"
package_path = get_package_share_directory(PACKAGE_NAME)

# load robot configs for time parametrization
def load_moveit_config(moveit_config_package: str) -> MoveItConfigs:
    # - build config
    mcb = MoveItConfigsBuilder(robot_name=ROBOT_NAME,
                               package_name=moveit_config_package)
    
    mcb.robot_description() # default path  # URDF
    mcb.robot_description_semantic() # default path  # SRDF
    mcb.robot_description_kinematics() # default path # kinematics.yaml
    mcb.joint_limits() # default path  # joint_limits.yaml  needed for collision checking
    
    return mcb.to_moveit_configs()


def generate_launch_description():
    moveit_config = load_moveit_config(MOVEIT_CONFIG_PACKAGE)

    

    # MoveItCpp node executable
    execution_node = Node(
        package=PACKAGE_NAME,
        executable="execution_node",
        parameters=[moveit_config.to_dict()]
    )

    nodes = [
        execution_node
    ]

    return LaunchDescription(nodes)