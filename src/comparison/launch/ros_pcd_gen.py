from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ros_pcd_gen_node
    param_file = os.path.join(
        get_package_share_directory('comparison'),
        'config/ros_pcd_gen.yaml'
    )
    ros_pcd_gen_node = Node(
        package='comparison',
        executable='ros_pcd_gen',
        output='screen',
        emulate_tty=True,
        parameters=[param_file]
    )

    return LaunchDescription([
        ros_pcd_gen_node,
    ])
