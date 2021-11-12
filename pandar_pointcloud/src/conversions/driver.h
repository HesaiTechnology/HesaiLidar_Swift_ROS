/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology Co., Ltd
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the pandar 3D LIDARs
 */

#ifndef _PANDAR_DRIVER_H_
#define _PANDAR_DRIVER_H_ 1

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <string>

#include <pandar_pointcloud/CloudNodeConfig.h>
#include <pandar_pointcloud/input.h>

typedef struct PandarGPS_s PandarGPS;
#define PANDAR128_READ_PACKET_SIZE (1800)
#define PANDARQT128_READ_PACKET_SIZE (200)
#define PANDAR80_READ_PACKET_SIZE (500)
#define PANDAR90_READ_PACKET_SIZE (200)
#define PANDAR64S_READ_PACKET_SIZE (400)
#define PANDAR40S_READ_PACKET_SIZE (100)
#define PANDAR_LASER_NUMBER_INDEX (6)
#define PANDAR_MAJOR_VERSION_INDEX (2)

namespace pandar_pointcloud {
class Convert;

class PandarDriver {
 public:
  PandarDriver(ros::NodeHandle node, ros::NodeHandle private_nh,
               std::string nodetype, pandar_pointcloud::Convert *convert);
  ~PandarDriver() {}

  bool poll(void);
  void publishRawData();
  void setUdpVersion(uint8_t major, uint8_t minor);
  int getPandarScanArraySize(boost::shared_ptr<Input>);

 private:
  /// Callback for dynamic reconfigure
  void callback(pandar_pointcloud::CloudNodeConfig &config, uint32_t level);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<
      dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig> >
      srv_;

  // configuration parameters
  struct {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each pandar time stamp
  } config_;

  boost::shared_ptr<Input> m_spInput;
  ros::Publisher output_;
  ros::Publisher gpsoutput_;
  bool m_bNeedPublish;
  std::array<pandar_msgs::PandarScanPtr, 2> pandarScanArray;
  int m_iScanPushIndex;
  int m_iScanPopIndex;
  pthread_mutex_t piclock;
  int m_iPandarScanArraySize;
  bool m_bGetScanArraySizeFlag;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  std::string publishmodel;
  std::string nodeType;

  pandar_pointcloud::Convert *convert;
};

}  // namespace pandar_driver

#endif  // _PANDAR_DRIVER_H_
