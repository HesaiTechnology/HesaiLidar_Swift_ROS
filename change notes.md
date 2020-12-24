# HesaiLidar_Pandar128_ROS

星期五, 16. 十月 2020 23:25下午 
##version
Pandar128_1.0.0

##modify
1. add version
2. add the function of read angle configuration from lidar 
3. revise correction.csv file 
4. modify judgment of PandarDriver::poll 
5. fix points clould blink when lidar's mode is last return/standard

星期一, 07. 十二月 2020 18:25下午 
##version
Pandar128_1.0.2

##modify
1. fix timestamp is zero

星期五, 18. 十二月 2020 16:30下午 
##version
Pandar128_1.0.3

##modify
1. Fix the problem that the cloud points data  is put in wrong place in dual model
2. Change the size of rosbag

星期三, 23. 十二月 2020 19:30下午 
##version
Pandar128_1.0.6

##modify
1. Fix that all point cloud values have timestamp and ring
2. Fix that sdk will core dumped when input wrong correction file path 