# HesaiLidar_Swift_ROS 

## About the project 
This repository includes the ROS Driver for Pandar LiDAR sensor manufactured by Hesai Technology. Branches are included for different systems and UDP protocol versions.

Developed based on [HesaiLidar_Swift_SDK](https://github.com/HesaiTechnology/HesaiLidar_Swift_SDK), After launched, the project will monitor UDP packets from Lidar,parse data and publish point clouds frames into ROS under topic: ```/pandar_points```. It can also be used as an official demo showing how to work with [HesaiLidar_Swift_SDK](https://github.com/HesaiTechnology/HesaiLidar_Swift_SDK).

## Branches 
```
* master: Pandar LiDAR ROS driver for Ubuntu 18.04 and Ubuntu20.04 supports the latest UDP protocol v1.4 v1.3 and v3.2 
* UDP1.4_ubuntu16.04: Pandar LiDAR ROS driver for Ubuntu 16.04 supports the latest UDP protocol v1.4
* UDP4.3: Pandar LiDAR ROS driver for  Ubuntu 16.04/Ubuntu 18.04/Ubuntu 20.04 supports the latest UDP protocol v4.3 

To get the UDP protocol version number of your device,  check the UDP package header field.
```
## Environment and Dependencies 
**System environment requirement: Linux + ROS**  
```
　Recommanded:  
　Ubuntu 16.04 - with ROS kinetic desktop-full installed or  
　Ubuntu 18.04 - with ROS melodic desktop-full installed or 
  Ubuntu 20.04 - with ROS noetic desktop-full installed
　Check resources on http://ros.org for installation guide 
```
**Compiler version requirement**
```
 Cmake version requirement: Cmake 3.8.0 or above
 G++ version requirement: G++ 7.5.0 or above
 ```

**Library Dependencies: libpcl-dev + libpcap-dev + libyaml-cpp-dev + libboost-dev**  
```
$ sudo apt-get update
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev  libboost-dev
```
## Download and Build 

**Install `catkin_tools`**
```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```
**Download code**  
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
$ git clone https://github.com/HesaiTechnology/HesaiLidar_Swift_ROS.git --recursive
```
**Install required dependencies with the help of `rosdep`**
```
$ cd ..
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
**Build**
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```


## Configuration 
```
 $ cd src/HesaiLidar_Swift_ROS/pandar_pointcloud/launch
```
open PandarSwift_points.launch to set configuration parameters

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
|calibration|lidar calibration file path|　

Data source will be read from pcap file instead of LiDAR once "pcap_file" not empty


## Run

### View the point clouds from connected LiDAR

1. Make sure current path in the `rosworkspace` directory
```
$ source devel/setup.bash
$ roslaunch pandar_pointcloud PandarSwift_points.launch
```

2. The driver will publish a PointCloud2 message in the topic.
```
/pandar_points
```

3. Open Rviz and add display by topic.

4. Change fixed frame to "PandarSwift" to view published point clouds.

### View the point clouds from rosbag file

1. Make sure current path in the `rosworkspace` directory
```
$ source devel/setup.bash
$ roslaunch pandar_pointcloud PandarSwift_points.launch
```

2. The driver will publish a raw data packet message in the topic.
```
/pandar_packets
```
3. record raw data rosbag
```
$rosbag record -b 4096 /hesai/pandar_packets
```

4. stop roslaunch and rosbag record by "Ctrl + C"

5. play raw data rosbag
```
$rosbag play <rosbagfile>
```

6. launch transform_nodelet.launch
```
$ roslaunch pandar_pointcloud transform_nodelet.launch data_type:=rosbag
```
7. Open Rviz and add display by topic.
8. Change fixed frame to "PandarSwift" to view published point clouds.

## Details of launch file parameters and utilities
|Parameter | Default Value|
|---------|---------------|
|calibration|Path of correction file, will be used when not able to get correction file from a connected Liar|
|device_ip_ip|The IP address of connected Lidar, will be used to get correction file|
|host_ip|The IP address of host pc, will be used to get udp data from lidar|
|frame_id|frame id of published messages|
|firetime_file|Path of firetime files|
|pcap|Path of the pcap file, once not empty, driver will get data from pcap file instead of a connected Lidar|
|port|The destination port of Lidar, driver will monitor this port to get point clouds packets from Lidar|
|start_angle|Driver will publish one frame point clouds data when azimuth angle step over start_angle, make sure set to within FOV|
|publish_model|default "points":publish point clouds "raw":publish raw UDP packets "both_point_raw":publish point clouds and UDP packets|
|namespace|namesapce of the launching node|
|coordinate_correction_flag|default "false":Disable coordinate correction "true":Enable coordinate correction|
|channel_config_file|Path of channel config file, will be used when not able to get channel config file from a connected Liar|
|cert_file|Path of the user's certificate|
|private_key_file|Path of the user's private key|
|ca_file|Path of the root certificate|
|multicast_ip|The multicast IP address of connected Lidar, will be used to get udp packets from multicast ip address|