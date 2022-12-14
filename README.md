## This is a project to compare the performance of ROS2 point cloud message passing between native DDS and Zenoh.

* ROS Velodyne driver: https://github.com/ros-drivers/velodyne/tree/ros2


```mermaid
flowchart TB
subgraph SR[LiDAR Source]
    direction LR
    VL(Velodyne LiDAR)
    VD(velodyne_driver)
    VP(velodyne_pointcloud)
    VL --> VD --> VP
end
VP --> TF
subgraph TF[Transfer]
    direction LR
    RP(ROS2 publisher)
    ZP(Zenoh publisher)
end
subgraph FRC[Frame Rate Counter]
    direction TB
    RS(ROS2 subscriber)
    ZS(Zenoh subscriber)
end
RP -- P1 --> RS
ZP -- P4 --> ZS
ZBD1(Zenoh Bridge DDS)
ZBD2(Zenoh Bridge DDS)
RP --> ZBD1 --> ZBD2 -- P2 --> RS
ZBD1 -- P3 --> ZS
```

Here's a simple visualizaton of the mechanism of [zenoh-bridge-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds).

```mermaid
flowchart LR
A{{ROS2 msg}} -- DDS --> B(Zenoh Bridge DDS) -- Zenoh --> C{{Zenoh msg}}
```

There're four pathways to transfer the raw LiDAR packets to the endpoint Frame rate counter.

* P1: the default way used in ROS2, passing messages with DDS
* P2: using two Zenoh/DDS bridges to cross the local network
* P3: directly using the zenoh message after Zenoh/DDS bridge
* P4: replacing the last message passing by pure zenoh protocol


## Usage

### Prerequisites

Make sure you have [ROS2 humble](https://docs.ros.org/en/humble/Installation.html) installed on your system.

### Download ROS dependencies

```bash
vcs import --input src/ros2.repos src
```

### Build

```bash
source /opt/ros/humble/setup.zsh
colcon build
```

### Run

```bash
source ./install/setup.zsh
ros2 launch transfer ros_pub.py
```

## Sample Data

![NAME](./pic/demo.gif)

```bash
wget https://github.com/YuanYuYuan/ros2-dds-zenoh-comparison/releases/download/2022-12-15/sample-data.tar.xz
tar xvf sample-data.tar.xz
ros2 bag play ./sample-data/rosbag2_2022_12_09-21_10_35_0.db3
ros2 launch velodyne_pointcloud velodyne_transform_node-VLP32C-launch.py
rviz2 -f velodyne
```


## Tips

Setup ccls via cmake _compile_commands.json_.

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -s ./build/compile_commands.json .
```
