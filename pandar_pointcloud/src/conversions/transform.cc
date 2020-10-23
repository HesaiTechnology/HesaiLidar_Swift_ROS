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

*/

#include "transform.h"

#include <pcl_conversions/pcl_conversions.h>

namespace pandar_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new pandar_rawdata::RawData()),
    m_spConver(new Convert(node, private_nh,"transform"))
  {
    ROS_WARN(" Transform::Transform");
    private_nh.getParam("frame_id", config_.frame_id);
    // Read calibration.
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    // output_ =
    //   node.advertise<sensor_msgs::PointCloud2>("transform_pandar_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
      TransformNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<pandar_pointcloud::TransformNodeConfig>::
      CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);
    
    // subscribe to PandarScan packets using transform filter
    
    pandar_scan_.subscribe(node, "pandar_packets", 10);
    tf_filter_ =
      new tf::MessageFilter<pandar_msgs::PandarScan>(pandar_scan_,
                                                         listener_,
                                                         config_.frame_id, 10);
    tf_filter_->registerCallback(boost::bind(&Transform::processScan, this, _1));
    ROS_WARN(" Transform::processScan config[%s]",config_.frame_id.c_str());
    ROS_WARN(" Transform::Transform finisher");
  }
  
  void Transform::reconfigure_callback(
      pandar_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range, 
                         config.view_direction, config.view_width);
    config_.frame_id = tf::resolve(tf_prefix_, config.frame_id);
    ROS_INFO_STREAM("Target frame ID: " << config_.frame_id);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg)
  {
    // process each packet provided by the driver
    // ROS_WARN(" Transform::processScan");
    for (size_t next = 0; next < scanMsg->packets.size(); ++next) {
        m_spConver->pushLiDARData(scanMsg->packets[next]);
        }
  }
} // namespace pandar_pointcloud
