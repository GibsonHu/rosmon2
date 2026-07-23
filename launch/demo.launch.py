"""Small installed launch file useful for validating rosmon2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        Node(
            package='demo_nodes_cpp', executable='talker', name='talker',
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='demo_nodes_cpp', executable='listener', name='listener',
            namespace=LaunchConfiguration('namespace')),
    ])
