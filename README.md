# ROS_P128

This repository includes the ROS Driver for Pandar128 LiDAR sensor manufactured by Hesai Technology.


## Build

### Install dependency library

```
$ sudo apt-get update
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev python-catkin-tools
```

### Compile

1. Create ROS Workspace. i.e. `rosworkspace`
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
```

2. Clone recursively this repository.
3. Install required dependencies with the help of `rosdep`
```
$ cd ..
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

4. Set udp buffer default size
```
$sudo sysctl -w net.core.rmem_default=12582912
enter password
```
5. Enter super user mode
```
$sudo -s
```

5. Compile
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```


## Config
```
 $ cd src/ROS_P128/pandar_pointcloud/launch
```
open Pandar128_points.launch to set configuration parameters
### Reciving data from connected Lidar: config lidar ip&port, leave the pcap_file empty
|Parameter | Default Value|
|---------|---------------|
|device_ip |192.168.1.201|
|port |2368|
|pcap_file ||

Data source will be from connected Lidar when "pcap_file" set to empty

### Reciving data from pcap file: config pcap_file and correction file path
|Parameter | Value|
|---------|---------------|
|pcap |pcap file path|

Data source will be from pcap file once "pcap_file" not empty


## Run
###View the point cloud from connected Lidar
1. While in the `rosworkspace` directory.
```
$ source devel/setup.bash
$ roslaunch pandar_pointcloud Pandar128_points.launch
```
2. The driver will publish a PointCloud2 message in the topic.
```
/pandar_points
```
3. Open Rviz and add display by topic.
4. Change fixed frame to "Pandar128" to view published point clouds.

###View the point cloud from rosbag
1. While in the `rosworkspace` directory.
```
$ source devel/setup.bash
$ roslaunch pandar_pointcloud Pandar128_points.launch
```
2. The driver will publish a raw data packet message in the topic.
```
/pandar_packets
```
3. record raw data rosbag
```
$rosbag record -b 4096 /pandar_packets
```

4. stop roslaunch and rosbag record by "Ctrl + C"

4. play raw data rosbag
```
$rosbag play <rosbagfiel>
```

6. launch transform_nodelet.launch
```
$ roslaunch pandar_pointcloud transform_nodelet.launch data_type:=rosbag
```
7. Open Rviz and add display by topic.
8. Change fixed frame to "Pandar128" to view published point clouds.