# HesaiLidar_Swift_ROS

星期二, 13. 十月 2020 01:25下午 
##version
Pandar128_1.2.1 

##modify
1. add version
2. add the function of read angle configuration from lidar 
3. revise correction.csv file 
4. modify judgment of PandarDriver::poll 
5. fix points clould blink when lidar's mode is last return/standard

星期一, 07. 十二月 2020 23:25下午 
##version
Pandar128_1.0.2

##modify
1. fix timestamp is zero

星期四, 10. 十二月 2020 19:25下午 
##version
Pandar128_1.0.3

##modify
1. Delete unnecessary msg files
2. Modify the launch file
3. Fixed lidar speed reading problem
4. Increase the output log of certificate authentication results
5. Fix pcap file reading

星期五, 18. 十二月 2020 14:05下午 
##version
Pandar128_1.0.4

##modify
1. Fix the problem that the cloud points data  is put in wrong place in dual model
2. Change the size of rosbag


星期三, 23. 十二月 2020 19:30下午 
##version
Pandar128_1.0.6

##modify
1. Fix that all point cloud values have timestamp and ring
2. Fix that sdk will core dumped when input wrong correction file path 

星期一, 01. 二月 2021 17:30下午 
##version
Pandar128_1.0.7

##modify
1. Change readme and branch name

星期二, 04. 二月 2021 17:30下午 
##version
PandarSwiftROS_1.0.8

##modify
1. Support to parser packet with UDP protocol v1.4

星期一, 08. 二月 2021 10:30下午 
##version
PandarSwiftROS_1.0.9

##modify
1. Fix bug in different rpm

星期四, 04. 三月 2021 17:30下午 
##version
PandarSwiftROS_1.0.10

##modify
1. Fix bug in parser half packets

星期五, 19. 三月 2021 19:30下午 
##version
PandarSwiftROS_1.0.11

##modify
1. Support Pandar AT128 first time

星期一, 22. 三月 2021 19:30下午 
##version
PandarSwiftROS_1.0.12

##modify
1. Support Pandar AT128 with three mirror

星期二, 23. 三月 2021 19:30下午 
##version
PandarSwiftROS_1.0.13

##modify
1. Change default correction file of transform nodelet

星期五, 26. 三月 2021 19:30下午 
##version
PandarSwiftROS_1.0.14

##modify
1. Change the calculation formula of azimuth
2. Fix bug in switch frame

Wednesday, April 14th, 2021 17:30
##version
PandarSwiftROS_1.0.15

##modify
1. Optimize calculation efficiency
2. Support QT128

Thursday, May 20th, 2021 17:30
##version
PandarSwiftROS_1.0.17

##modify
1. Support ubuntu 20.04
2. Update firetime correction of QT128

Sunsday, August 15th, 2021 17:30
##version
PandarSwiftROS_1.0.21

##modify
1. Support UDP4.3

Thursday, September 9th, 2021 17:30
##version
PandarSwiftROS_1.0.23

##modify
1. Support to accept lidar fault message

Tuesday, October 26th, 2021 17:30
##version
PandarSwiftROS_1.0.24

##modify
1. Optimize the point to switch frame

Thursday,December 23th, 2021 17:30
##version
PandarSwiftROS_1.0.26

##modify
1. Fix bug in calculate point index
2, fix bug in play rosbag

Friday,January 14th, 2022 17:30
##version
PandarSwiftROS_1.0.27

##modify
1. Support LTS lidar 

Tuesday,March 1th, 2022 17:30
##version
PandarSwiftROS_1.0.28

##modify
1. Support to use diffrernt angle resolution


Friday,May 6th, 2022 12:30
##version
PandarSwiftROS_1.0.29

##modify
1. Support to parser faultmessage
2. Optimize calculation efficiency

Monday,November 28th, 2022 19:30
##version
PandarSwiftROS_1.0.35

##modify
1. Support multcast
2. Fix bug in change frame

Monday December 5th, 2022 17:30
##version
PandarSwiftROS_1.0.36

##modify
1. Optimize multicast
