# HesaiLidar_Pandar128_ROS

## About the project
HesaiLidar_Pandar128_ROS project includes the ROS Driver for **Pandar128** LiDAR sensor manufactured by Hesai Technology.    
This branch supports UDP protocol version 1.3.    
After launched, the project will monitor UDP packets from Lidar, parse data and publish point cloud frames into ROS. It can also be used as an official demo showing how to work with **HesaiLidar_Pandar128_SDK**


## Environment and Dependencies
**System environment requirement: Linux + ROS**  

　Recommanded:  
　Ubuntu 16.04 - with ROS kinetic desktop-full installed or  
　Ubuntu 18.04 - with ROS melodic desktop-full installed  
　Check resources on http://ros.org for installation guide 
 
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**  
```
$sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev
```

## Download and Build

**Install `catkin_tools`**
```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```
**Download code**  
1. Create ROS Workspace. i.e. `rosworkspace`. Please note that "Workspace" is defined by your own, but subfolder "src" is required and cannot be renamed, i.e.
```
$ mkdir -p rosworkspace/src
```
2. cd to the /src subfolder, i.e.
```
$ cd rosworkspace/src
```
3. Clone code and checkout to correct branch
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_Pandar128_ROS.git --recursive
$ git checkout -b UDP_V1.3 origin/UDP_V1.3
```   
### Compile    
```
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```


## Configuration    
```
 $ gedit src/HesaiLidar_Pandar128_ROS/pandar_pointcloud/launch/Pandar128_points.launch
```
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
|calibration| calibration file path|

Data source will be read from pcap file instead of LiDAR once "pcap_file" not empty


## Run

1. Set UDP buffer default size
```
$sudo sysctl -w net.core.rmem_default=26214400
```
2. Enter super user mode
```
$sudo -s
```
3. Make sure current path in the `rosworkspace` directory
```
$ source devel/setup.bash
$ roslaunch pandar_pointcloud Pandar128_points.launch
```

4. The driver will publish a PointCloud2 message in the topic.
```
/pandar_points
```

3. Open Rviz and add display by topic.

4. Change fixed frame to "Pandar128" to view published point clouds.

## Details of launch file parameters and utilities
|Parameter | Default Value|
|---------|---------------|
|calibration|Path of calibration file, will be used when not able to get calibration file from a connected Liar|
|device_ip|The IP address of connected Lidar, will be used to get calibration file|
|frame_id|Fill in the Lidar module Type|
|manager|For developers only, don't need to care for normal usage|
|max_range|Points futher than max range will be filtered|
|firetime_file|Path of firetime file|
|min_range|Points closer than min range will be filtered|
|pcap|Path of the pcap file, once not empty, driver will get data from pcap file instead of a connected Lidar|
|port|The destination port of Lidar, driver will monitor this port to get UDP packets from Lidar|
|read_fast|For developers only, don't need to care for normal usage|
|read_once|For developers only, don't need to care for normal usage|
|repeat_delay|For developers only, don't need to care for normal usage|
|rpm|For developers only, don't need to care for normal usage|
|start_angle|Driver will publish one frame point cloud data when azimuth angel step over start_angle, make sure set to within FOV|
|publish_model|"point":publish point clouds "raw":publish raw UDP packets "both_point_raw":publish point clouds and UDP packets|

### Publish raw UDP packets to ROS
set "publish_model" to "raw" or "both_point_raw" then launch driver    
The driver will publish a raw data packet message in the topic
```
/pandar_packets
```
### Record and parse raw UDP packets from ROS topic
1. record raw data rosbag
```
$rosbag record -b 4096 /pandar_packets
```
2. stop rosbag record by "Ctrl + C", a rosbag file will be generated in the terminal working path

3. play raw data rosbag
`
$rosbag play <rosbagfile>
`
    this will publish raw udp packets to ROS under the topic `/pandar_packets`    
4. launch transform_nodelet.launch
```
$ roslaunch pandar_pointcloud transform_nodelet.launch data_type:=rosbag
```
5. Driver will get data from the `/pandar_packets`, open Rviz and add display by topic.
6. Change fixed frame to "Pandar128" to view published point clouds
