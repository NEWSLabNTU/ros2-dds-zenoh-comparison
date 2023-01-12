#!/usr/bin/env bash



workspace_dir="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source $workspace_dir/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

trap ctrl_c INT

function ctrl_c() {
    pkill -9 ros
    pkill -9 zenoh
    echo "Kill all!"
    exit
}

# NUM_JOBS=7
# PROTOCOL="ros"
# PROTOCOL="zenoh"
TIMEOUT=30

# ROS_BAG_CPUS="0,1"
# TRANSFER_CPUS="2,3"
# FPS_COUNTER_CPUS="4,5"

FPS_COUNTER_CPUS="8,9"
TRANSFER_CPUS="10-13"



payload_size_list=(
    1024
    4096
    16384
    65536
    262144
    1048576
    # 4194304
    # 16777216
)
BASE_LOG_DIR=./logs/$(date +"%Y%m%d.%H%M%S")

for PAYLOAD_SIZE in ${payload_size_list[@]}; do
    LOG_DIR="${BASE_LOG_DIR}/${PAYLOAD_SIZE}"
    mkdir -p $LOG_DIR
    for NUM_JOBS in {1..20..2}; do
    # for NUM_JOBS in 13; do
        for PROTOCOL in "zenoh" "ros"; do
        # for PROTOCOL in "ros"; do
            log_file=$LOG_DIR/${PROTOCOL}-${NUM_JOBS}.log
            rm -f $log_file &> /dev/null
            pkill -9 $PROTOCOL
            parallel --lb --halt now,done=1 << EOF
seq $NUM_JOBS | parallel -j $NUM_JOBS "taskset -a -c $TRANSFER_CPUS ros2 run comparison ${PROTOCOL}_pcd_gen --ros-args -p payload_size:=$PAYLOAD_SIZE; echo "
timeout $TIMEOUT taskset -a -c $FPS_COUNTER_CPUS ros2 launch comparison ${PROTOCOL}_sub.py | tee -a $log_file
EOF
        done
    done
done

pkill -9 ros
pkill -9 zenoh
