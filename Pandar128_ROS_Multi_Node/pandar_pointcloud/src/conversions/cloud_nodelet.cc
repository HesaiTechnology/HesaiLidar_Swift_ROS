/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts raw Pandar40 3D LIDAR packets to a
    PointCloud2.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "convert.h"

namespace pandar_pointcloud
{
  class CloudNodelet: public nodelet::Nodelet
  {
  public:

    CloudNodelet() {}
    ~CloudNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Convert> conv_;
  };

  /** @brief Nodelet initialization. */
  void CloudNodelet::onInit()
  {
    conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace pandar_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_pandar.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(pandar_pointcloud::CloudNodelet, nodelet::Nodelet);
