/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Pandar128 3D LIDAR packets to PointCloud2
    in the /odom frame of reference.

*/

#ifndef _PANDAR_POINTCLOUD_TRANSFORM_H_
#define _PANDAR_POINTCLOUD_TRANSFORM_H_ 1

#include "convert.h"
#include <rclcpp/rclcpp.hpp>
#include <pandar_pointcloud/msg/pandar_scan.hpp>

namespace pandar_pointcloud
{
  class Transform
  {
  public:

    Transform(rclcpp::Node::SharedPtr& private_nh);
    ~Transform() {}

  private:

    void processScan(const pandar_pointcloud::msg::PandarScan::SharedPtr scanMsg);
    boost::shared_ptr<Convert> m_spConver;
    rclcpp::Subscription<pandar_pointcloud::msg::PandarScan>::SharedPtr pandar_scan_;
  };

} // namespace pandar_pointcloud

#endif // _PANDAR_POINTCLOUD_TRANSFORM_H_
