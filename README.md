# HesaiLidar_Swift_ROS

This repository includes the ROS Driver for Pandar128 LiDAR sensor manufactured by Hesai Technology. Branches are included for different systems and UDP protocol versions:
* master: Pandar128 ROS driver for Ubuntu 18.04 supports the latest UDP protocol v1.4
* UDP1.4_ubuntu16.04: Pandar128 ROS driver for Ubuntu 16.04 supports the latest UDP protocol v1.4
* UDP1.3: Pandar128 ROS driver for ubuntu16.04 and ubuntu 18.04 supports UDP protocol v1.3    

To get the UDP protocol version number of your device,  check the UDP package header field.

## About the project
ROS_Pandar128 project includes the ROS Driver for：  
**Pandar64S/Pandar40S/Pandar80S/Pandar128 with UDP protocol version 1.4**  
LiDAR sensor manufactured by Hesai Technology.  

## Build

### Install dependency libraries
```
$ sudo apt-get update
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev python-catkin-tools
```

### Install ROS
http://wiki.ros.org/ROS/Installation

### Compile

1. Create ROS Workspace. i.e. `rosworkspace`. Please note that "Workspace" is defined by your own, but subfolder "src" is required and cannot be renamed 
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
```

2. Clone recursively this repository in the current path
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_Pandar128_ROS.git
```
3. Install required dependencies with the help of `rosdep`
```
$ cd ..
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

4. Compile
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```


## Config
```
 $ cd src/ROS_P128/pandar_pointcloud/launch
```
open Pandar128_points.launch to set configuration parameters

### Reciving data from connected LiDAR: config LiDAR IP address & UDP port, and leave the pcap_file empty

|Parameter | Default Value|
|---------|---------------|
|device_ip |192.168.1.201|
|port |2368|
|pcap_file ||

Data source will be read from connected LiDAR when "pcap_file" is set to empty

### Reciving data from pcap file: config pcap_file and correction file path

|Parameter | Value|
|---------|---------------|
|pcap |pcap file path|

Data source will be read from pcap file instead of LiDAR once "pcap_file" not empty


## Run

Set UDP buffer default size
```
$sudo sysctl -w net.core.rmem_default=26214400
```
Enter super user mode
```
$sudo -s
```

### View the point cloud from connected LiDAR

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

### View the point cloud from rosbag file

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