from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package ='pandar_pointcloud',
            node_namespace ='hesai',
            node_executable ='pandar_transform_node',
            name ='pandar_transform',
            output ="screen",
            parameters=[
                {"frame_id"  : "PandarSwift"},
                {"calibration"  : "./src/ROS_Swift/pandar_pointcloud/params/PandarQT128_Correction.csv"},
                {"firetime_file"  : "./src/ROS_Swift/pandar_pointcloud/params/PandarQT128_Firetimes.csv"},
                {"coordinate_correction_flag"  : False},
            ]
        )
    ])
