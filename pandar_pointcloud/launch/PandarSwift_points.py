from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package ='pandar_pointcloud',
            node_namespace ='hesai',
            node_executable ='pandar_cloud_node',
            name ='pandar_pointcloud',
            output ="screen",
            parameters=[
                {"pcap": "''"},
                {"device_ip"  : "192.168.1.201"},
                {"port"  : 2368},
                {"start_angle"  : 0.0},
                {"frame_id"  : "PandarSwift"},
                {"publish_model"  : "both_point_raw"},
                {"calibration"  : "./src/ROS_Swift/pandar_pointcloud/params/Pandar128_Correction.csv"},
                {"firetime_file"  : "./src/ROS_Swift/pandar_pointcloud/params/Pandar128_Firetimes.csv"},
                {"coordinate_correction_flag"  : False},
                {"cert_file"  : "''"},
                {"private_key_file"  : "''"},
                {"ca_file"  : "''"},
            ]
        )
    ])
