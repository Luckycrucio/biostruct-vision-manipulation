# View and view the URDF in RViz

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    robot_description_path = get_package_share_directory("abb_irb6700_205_280_support")

    robot_description_config = xacro.process_file(
        os.path.join(
            robot_description_path,
            "urdf",
            "abb_irb6700_205_280.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_sliders = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(robot_description_path, "rviz", "urdf_description.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription([robot_state_publisher_node,
                              joint_state_sliders,
                              rviz])