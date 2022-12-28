#!/usr/bin/env bash

workspace_dir="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source $workspace_dir/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# NUM_JOBS=7
SPEED_RATE=1
# PROTOCOL="ros"
# PROTOCOL="zenoh"
TIMEOUT=60

# ROS_BAG_CPUS="0,1"
# TRANSFER_CPUS="2,3"
# FPS_COUNTER_CPUS="4,5"

ROS_BAG_CPUS="6,7"
FPS_COUNTER_CPUS="8,9"
TRANSFER_CPUS="10-13"


LOG_DIR="./logs"
rm -rf $LOG_DIR
mkdir -p $LOG_DIR

for NUM_JOBS in {1..9}; do
    for PROTOCOL in "zenoh" "ros"; do
        log_file=$LOG_DIR/${PROTOCOL}-${NUM_JOBS}.log
        rm -f $log_file &> /dev/null
        pkill -9 $PROTOCOL
        pkill -9 velodyne
        parallel --lb --halt now,done=1 << EOF
taskset -a -c $ROS_BAG_CPUS ros2 bag play sample-data/rosbag2_2022_12_09-21_10_35_0.db3 --loop -r $SPEED_RATE &> /dev/null
seq $NUM_JOBS | parallel -j $NUM_JOBS "taskset -a -c $TRANSFER_CPUS ros2 launch comparison ${PROTOCOL}_pub.py; echo "
timeout $TIMEOUT taskset -a -c $FPS_COUNTER_CPUS ros2 launch comparison ${PROTOCOL}_sub.py | tee -a $log_file
EOF
    done
done

pkill -9 ros
pkill -9 zenoh
pkill -9 velodyne
