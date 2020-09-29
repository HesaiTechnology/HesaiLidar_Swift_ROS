include_directories(../taskflow)

find_package(PCL REQUIRED COMPONENTS common)

add_executable(pandar_cloud_node cloud_node.cc convert.cc driver.cc laser_ts.cpp)
add_dependencies(pandar_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_node pandar_rawdata
					  pandar_input
					  pcap
					  ${PCL_INCLUDE_DIRS}
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
					  

add_library(pandar_cloud_nodelet cloud_nodelet.cc convert.cc driver.cc laser_ts.cpp)
add_dependencies(pandar_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_nodelet 
					  pandar_rawdata 
					  pandar_input
					  pcap
					  ${PCL_INCLUDE_DIRS}
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(pandar_transform_node transform_node.cc transform.cc convert.cc driver.cc laser_ts.cpp)
add_dependencies(pandar_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_transform_node pandar_rawdata
						pandar_input
						pcap
						${PCL_INCLUDE_DIRS}
					  	${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(pandar_transform_nodelet transform_nodelet.cc transform.cc convert.cc driver.cc laser_ts.cpp)
add_dependencies(pandar_transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_transform_nodelet pandar_rawdata
						pandar_input
						pcap
						${PCL_INCLUDE_DIRS}
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS 
            pandar_cloud_node
            pandar_cloud_nodelet
            pandar_transform_node
            pandar_transform_nodelet
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
