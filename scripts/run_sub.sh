#!/usr/bin/env bash
set -e

name="$1"
shift || {
    echo "Usage: $0 NAME"
    exit 1
}

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "${script_dir}/.."

export CYCLONEDDS_URI="file://$script_dir/../cyclonedds.sub.xml"

source install/setup.bash
mkdir -p data
mkdir "data/$name"

parallel -j0 --lb --halt-on-error now,fail=1 --timeout 20 <<EOF
ros2 run comparison ros_sub
sleep 2 && cd data/$name && ros2 bag record --qos-profile-overrides-path ../../qos_override.yaml /transfer_topic
cd data/$name && tshark -i wlp4s0 -w packets.pcap
EOF
