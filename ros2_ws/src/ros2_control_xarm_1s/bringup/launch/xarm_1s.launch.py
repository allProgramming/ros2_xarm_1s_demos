
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    urdf_xacro_path = os.path.join(
        get_package_share_directory("xarm_1s_description"),
        "urdf",
        "xarm_1s.urdf.xacro")
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([urdf_xacro_path])])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ros2_control_xarm_1s"),
        "config",
        "xarm_1s_controllers.yaml"])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("xarm_1s_description"), "rviz", "xarm_1s.rviz"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, robot_controllers],
            output="both"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                    arguments=["-d", rviz_config_file])])),
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
                )]))])
