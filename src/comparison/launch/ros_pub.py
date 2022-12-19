from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('comparison'),
        'config/ros_pub.yaml'
    )
    return LaunchDescription([
        Node(
            package='comparison',
            executable='ros_pub',
            output='screen',
            emulate_tty=True,
            parameters=[param_file]
        ),
    ])
