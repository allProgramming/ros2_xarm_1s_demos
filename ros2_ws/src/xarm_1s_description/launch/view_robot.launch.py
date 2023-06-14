from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
        PathJoinSubstitution([urdf_xacro_path]),
    ])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("xarm_1s_description"), "rviz", "xarm_1s.rviz"]
    )

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file]),
    ])
