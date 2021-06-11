/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node transforms raw Pandar128 LIDAR packets to PointCloud2
    in the /odom frame of reference.

*/

#include <rclcpp/rclcpp.hpp>
#include "transform.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("transform_node");
  pandar_pointcloud::Transform transform(nh_);
  rclcpp::spin(nh_);
  rclcpp::shutdown();
  return 0;
}
