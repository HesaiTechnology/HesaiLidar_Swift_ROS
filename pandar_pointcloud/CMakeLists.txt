cmake_minimum_required(VERSION 3.5)
project(pandar_pointcloud)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package( Boost REQUIRED  COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS system)
# find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_c REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost COMPONENTS signals)
find_package(OpenSSL REQUIRED)
message("OpenSSL include dir: ${OPENSSL_INCLUDE_DIR}")

set(libpcap_LIBRARIES -lpcap)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PandarPacket.msg"
  "msg/PandarScan.msg"
  "msg/PandarGps.msg"
  DEPENDENCIES builtin_interfaces 
)
find_package(PCL REQUIRED COMPONENTS common)
find_package(PCL REQUIRED COMPONENTS io)
ament_export_dependencies(rosidl_default_runtime)

# add_subdirectory(src/lib)
add_library(pandar_input src/lib/input.cc src/lib/platUtil.cc)
target_include_directories(pandar_input PRIVATE
    include
)
target_link_libraries(pandar_input
  ${libpcap_LIBRARIES}
)

ament_target_dependencies(pandar_input
  "geometry_msgs"  # maybe don't need this
  "ament_index_cpp"
  "pcl_conversions"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "tf2_geometry_msgs"
)
# add_subdirectory(src/conversions)

include_directories(taskflow)

include_directories(
	${OPENSSL_INCLUDE_DIR}
)

add_executable(pandar_cloud_node src/conversions/cloud_node.cc)
# add_executable(pandar_cloud_node cloud_node.cc convert.cc driver.cc laser_ts.cpp tcp_command_client.c util.c)
# add_dependencies(pandar_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_node
					  pandar_input
					  pcap
					  ${PCL_LIBRARIES}
					  ${OPENSSL_LIBRARIES}
            )
target_include_directories(pandar_cloud_node PRIVATE
            include
            ${Boost_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
        )            
					  
ament_target_dependencies(pandar_cloud_node
            "geometry_msgs"  # maybe don't need this
            "ament_index_cpp"
            "pcl_conversions"
            "rcl_interfaces"
            "rclcpp"
            "rcutils"
            "sensor_msgs"
            "std_msgs"
            "tf2_geometry_msgs"
          )
rosidl_target_interfaces(pandar_cloud_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# add_executable(pandar_transform_node transform_node.cc transform.cc convert.cc driver.cc laser_ts.cpp tcp_command_client.c util.c)
# add_dependencies(pandar_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(pandar_transform_node pandar_rawdata
# 						pandar_input
# 						pcap
# 						${PCL_INCLUDE_DIRS}
# 						${OPENSSL_LIBRARIES}
# 					  )

ament_package()

