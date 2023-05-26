#!/usr/bin/env bash
set -e

name="$1"
shift || {
    echo "Usage: $0 NAME"
    exit 1
}

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "${script_dir}/.."

source install/setup.bash
export CYCLONEDDS_URI="file://${script_dir}/../cyclonedds.pub.xml"
mkdir "$name"

parallel -j0 --lb --timeout 20 <<EOF
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
ros2 launch velodyne_pointcloud velodyne_transform_node-VLP32C-launch.py
ros2 run comparison ros_pub
cd $name && ros2 bag record --all
cd $name && tshark -i wlp4s0 -w packets.pcap
EOF
