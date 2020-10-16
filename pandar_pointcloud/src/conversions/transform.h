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

#include <ros/ros.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <pandar_pointcloud/rawdata.h>
#include <pandar_pointcloud/point_types.h>

#include <dynamic_reconfigure/server.h>
#include <pandar_pointcloud/TransformNodeConfig.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>
#include "convert.h"

/** types of point and cloud to work with */
typedef pandar_rawdata::PPoint PPoint;
typedef pandar_rawdata::PPointCloud PPointCloud;

// instantiate template for transforming a PPointCloud
template bool
  pcl_ros::transformPointCloud<PPoint>(const std::string &,
                                       const PPointCloud &,
                                       PPointCloud &,
                                       const tf::TransformListener &);

namespace pandar_pointcloud
{
  class Transform
  {
  public:

    Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Transform() {}

  private:

    void processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg);

    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<pandar_pointcloud::
      TransformNodeConfig> > srv_;
    void reconfigure_callback(pandar_pointcloud::TransformNodeConfig &config,
                  uint32_t level);
    
    const std::string tf_prefix_;
    boost::shared_ptr<pandar_rawdata::RawData> data_;
    boost::shared_ptr<Convert> m_spConver;
    message_filters::Subscriber<pandar_msgs::PandarScan> pandar_scan_;
    tf::MessageFilter<pandar_msgs::PandarScan> *tf_filter_;
    ros::Publisher output_;
    tf::TransformListener listener_;

    /// configuration parameters
    typedef struct {
      std::string frame_id;          ///< target frame ID
    } Config;
    Config config_;

    // Point cloud buffers for collecting points within a packet.  The
    // inPc_ and tfPc_ are class members only to avoid reallocation on
    // every message.
    PPointCloud inPc_;              ///< input packet point cloud
    PPointCloud tfPc_;              ///< transformed packet point cloud
  };

} // namespace pandar_pointcloud

#endif // _PANDAR_POINTCLOUD_TRANSFORM_H_
