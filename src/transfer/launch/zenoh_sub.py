from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='comparison',
            executable='zenoh_sub',
            output='screen',
            emulate_tty=True,
        ),
    ])
