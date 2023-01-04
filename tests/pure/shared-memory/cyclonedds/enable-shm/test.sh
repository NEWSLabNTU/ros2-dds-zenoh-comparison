#!/usr/bin/env bash

workspace_dir="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source $workspace_dir/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


MAX_NUM_JOBS=25
SPEED_RATE=1
# PROTOCOL="ros"
# PROTOCOL="zenoh"
TIMEOUT=30

# ROS_BAG_CPUS="0,1"
# TRANSFER_CPUS="2,3"
# FPS_COUNTER_CPUS="4,5"

ROS_BAG_CPUS="6,7"
FPS_COUNTER_CPUS="8,9"
TRANSFER_CPUS="10-13"


CURR_DIR=$(exec pwd)
LOG_DIR=$CURR_DIR/logs
rm -rf $LOG_DIR
mkdir -p $LOG_DIR

# echo $0 >> $LOG_DIR

for ((NUM_JOBS=1;NUM_JOBS<=MAX_NUM_JOBS;NUM_JOBS+=2)); do
    for PROTOCOL in "zenoh" "ros"; do
        log_file=$LOG_DIR/${PROTOCOL}-${NUM_JOBS}.log
        rm -f $log_file &> /dev/null
        pkill -9 $PROTOCOL
        pkill -9 velodyne
        parallel --lb --halt now,success=1 << EOF
seq $NUM_JOBS | parallel -j $NUM_JOBS "taskset -a -c $TRANSFER_CPUS ros2 launch comparison pure_${PROTOCOL}_pub.py; echo "
timeout $TIMEOUT taskset -a -c $FPS_COUNTER_CPUS ros2 launch comparison ${PROTOCOL}_sub.py | tee -a $log_file
EOF
    done
done

pkill -9 ros
pkill -9 zenoh
pkill -9 velodyne

