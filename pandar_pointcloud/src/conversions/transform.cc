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

    @author Jack O'Quin
    @author Jesse Vera
    @author Yang Sheng
    @author zhang Yu

*/

#include "transform.h"
#include <pandar_pointcloud/msg/pandar_scan.hpp>
#include <pandar_pointcloud/msg/pandar_packet.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace pandar_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(rclcpp::Node::SharedPtr& private_nh):
    m_spConver(new Convert(private_nh,"transform"))
  {
    printf(" Transform::Transform");
    // subscribe to PandarScan packets using transform filter
    pandar_scan_ = private_nh->create_subscription<pandar_pointcloud::msg::PandarScan>("pandar_packets", 10,\
                       std::bind(&Transform::processScan, this, std::placeholders::_1));
    printf(" Transform::Transform finisher");
  }
  
  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const pandar_pointcloud::msg::PandarScan::SharedPtr scanMsg)
  {
    // process each packet provided by the driver
    // printf(" Transform::processScan");
    for (size_t next = 0; next < scanMsg->packets.size(); ++next) {
        m_spConver->pushLiDARData(scanMsg->packets[next]);
        }
  }
} // namespace pandar_pointcloud
