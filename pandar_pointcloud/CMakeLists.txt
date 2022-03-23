cmake_minimum_required(VERSION 3.5)
project(pandar_pointcloud)
add_definitions(-fPIC)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package( Boost REQUIRED  COMPONENTS thread)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_c REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenSSL REQUIRED)
message("OpenSSL include dir: ${OPENSSL_INCLUDE_DIR}")

set(libpcap_LIBRARIES -lpcap)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PandarPacket.msg"
  "msg/PandarScan.msg"
  "msg/PandarGps.msg" 
)
find_package(PCL REQUIRED COMPONENTS common)
find_package(PCL REQUIRED COMPONENTS io)

# add_subdirectory(src/lib)
add_library(pandar_input src/lib/input.cc src/lib/platUtil.cc)
target_include_directories(pandar_input PRIVATE
    include
)
target_link_libraries(pandar_input
  ${libpcap_LIBRARIES}
)
set(dependencies 
  "geometry_msgs"  
  "ament_index_cpp"
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "image_transport"
  "std_msgs"
  "rclcpp_action"
  "std_srvs"
)
ament_target_dependencies(pandar_input ${dependencies})

include_directories(taskflow)
include_directories(
	${OPENSSL_INCLUDE_DIR}
)
add_library(PandarSwiftLib SHARED
    src/conversions/convert.cc
    src/conversions/driver.cc
    src/conversions/laser_ts.cpp
    src/conversions/tcp_command_client.c
    src/conversions/util.c
    src/conversions/transform.cc
)
target_include_directories(PandarSwiftLib PRIVATE
    include
    ./include/pandar_pointcloud
    src/taskflow
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
)
target_link_libraries(PandarSwiftLib
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    Boost::thread
    pcap
    pandar_input
)
rosidl_target_interfaces(PandarSwiftLib   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# pandar_cloud_node
add_executable(pandar_cloud_node src/conversions/cloud_node.cc)
target_link_libraries(pandar_cloud_node
					  pandar_input
					  pcap
					  ${PCL_LIBRARIES}
					  ${OPENSSL_LIBRARIES}
            PandarSwiftLib
            )
target_include_directories(pandar_cloud_node PRIVATE
            include
            ./include/pandar_pointcloud
            src/conversions
            ${Boost_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
        )            
					  
ament_target_dependencies(pandar_cloud_node ${dependencies})
rosidl_target_interfaces(pandar_cloud_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# transfrom node
add_executable(pandar_transform_node src/conversions/transform_node.cc)
target_link_libraries(pandar_transform_node
					  pandar_input
					  pcap
					  ${PCL_LIBRARIES}
					  ${OPENSSL_LIBRARIES}
            PandarSwiftLib
            )
target_include_directories(pandar_transform_node PRIVATE
            include
            ./include/pandar_pointcloud
            src/conversions
            ${Boost_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
        )            
					  
ament_target_dependencies(pandar_transform_node ${dependencies})

install(TARGETS PandarSwiftLib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS pandar_cloud_node pandar_transform_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})  

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()

