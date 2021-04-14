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

星期五, 11. 十二月 2020 19:25下午 
##version
Pandar128_1.0.4

##modify
1. use static library

星期五, 18. 十二月 2020 17:05下午 
##version
Pandar128_1.0.5

##modify
1. Fix the problem that the cloud points data  is put in wrong place in dual model
2. Change the size of rosbag

星期三, 23. 十二月 2020 19:30下午 
##version
Pandar128_1.0.6

##modify
1. Fix that all point cloud values have timestamp and ring
2.Fix that sdk will core dumped when input wrong correction file path 

星期一, 01. 二月 2021 17:30下午 
##version
Pandar128_1.0.7

##modify
1. Change readme and branch name

星期二, 04. 二月 2021 17:30下午 
##version
PandarSwift_1.0.8

##modify
1. Support to parser packet with UDP protocol v1.4

星期一, 08. 二月 2021 10:30下午 
##version
PandarSwift_1.0.9

##modify
1. Fix bug in different rpm

星期四, 04. 三月 2021 17:30下午 
##version
PandarSwift_1.0.10

##modify
1. Fix bug in parser half packets

Wednesday, April 14th, 2021 17:30
##version
PandarSwiftROS_1.0.11

##modify
1. Add namespace parmeter for topic
2. Optimize calculation efficiency
3. Support QT128
