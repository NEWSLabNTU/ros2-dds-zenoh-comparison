from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():

    # zenoh_pub_node
    param_file = os.path.join(
        get_package_share_directory('comparison'),
        'config/ros_pub.yaml'
    )
    ros_pub_node = Node(
        package='comparison',
        executable='pure_ros_pub',
        output='screen',
        emulate_tty=True,
        parameters=[param_file]
    )

    # velodyne_transform_node
    share_dir = get_package_share_directory('velodyne_pointcloud')
    params_file = os.path.join(
        share_dir,
        'config',
        'VLP32C-velodyne_transform_node-params.yaml'
    )
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    params['calibration'] = os.path.join(
        share_dir,
        'params',
        'VeloView-VLP-32C.yaml'
    )
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[params]
    )

    return LaunchDescription([
        ros_pub_node,
        # velodyne_transform_node
    ])
