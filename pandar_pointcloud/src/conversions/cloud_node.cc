/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Pandar128 LIDAR packets to PointCloud2.

*/
#include <rclcpp/rclcpp.hpp>
#include "convert.h"

/** Main node entry point. */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("pandar_pointcloud");
  pandar_pointcloud::Convert conv(nh_);
  rclcpp::spin(nh_);
  rclcpp::shutdown();
  return 0;
}
