/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Pandar128 3D LIDARs
 */

#include <cmath>
#include <string>

#include <pandar_msgs/PandarGps.h>
#include <pandar_msgs/PandarScan.h>
#include <pandar_pointcloud/platUtil.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "convert.h"
#include "driver.h"

namespace pandar_pointcloud {

PandarDriver::PandarDriver(ros::NodeHandle node, ros::NodeHandle private_nh,
                           std::string nodetype,
                           pandar_pointcloud::Convert *cvt) {
  convert = cvt;
  // use private node handle to get parameters
  private_nh.getParam("frame_id", config_.frame_id);
  private_nh.getParam("publish_model", publishmodel);
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
  nodeType = nodetype;

  // get model name, validate string, determine packet rate
  config_.model = "Pandar128";
  std::string model_full_name = std::string("Hesai") + config_.model;
  double packet_rate = 3000;  // packet frequency (Hz)
  std::string deviceName(model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  // ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int)ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  // ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int)DATA_PORT_NUMBER);

  std::string multicast_ip;
  private_nh.param("multicast_ip", multicast_ip, std::string(""));

  std::string host_ip;
  private_nh.param("host_ip", host_ip, std::string(""));

  pthread_mutex_init(&piclock, NULL);

  m_bNeedPublish = false;
  m_iScanPushIndex = 0;
  m_iScanPopIndex = 1;
  m_bGetScanArraySizeFlag = false;
  m_iPandarScanArraySize = PANDAR128_READ_PACKET_SIZE;
  pandar_msgs::PandarScanPtr scan0(new pandar_msgs::PandarScan);
  scan0->packets.resize(m_iPandarScanArraySize);
  pandar_msgs::PandarScanPtr scan1(new pandar_msgs::PandarScan);
  scan1->packets.resize(m_iPandarScanArraySize);

  pandarScanArray[m_iScanPushIndex] = scan0;
  pandarScanArray[m_iScanPopIndex] = scan1;

  // // Initialize dynamic reconfigure
  // srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
  //   CloudNodeConfig> > (private_nh);
  // dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
  //   CallbackType f;
  // f = boost::bind (&PandarDriver::callback, this, _1, _2);
  // srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  // using namespace diagnostic_updater;
  // diag_topic_.reset(new TopicDiagnostic("pandar_packets", diagnostics_,
  //                                       FrequencyStatusParam(&diag_min_freq_,
  //                                                            &diag_max_freq_,
  //                                                            0.1, 10),
  //                                       TimeStampStatusParam()));

  // open Pandar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    m_spInput.reset(new pandar_pointcloud::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
  } else {
    // read data from live socket
    m_spInput.reset(new pandar_pointcloud::InputSocket(private_nh, host_ip, udp_port, 10010, multicast_ip));
  }
  // ROS_WARN("drive nodeType[%s]", nodeType.c_str());
  // ROS_WARN("drive publishmodel[%s]", publishmodel.c_str());
  // raw packet output topic
  if ((publishmodel == "both_point_raw" || publishmodel == "raw") &&
      (nodeType == LIDAR_NODE_TYPE)) {
    // ROS_WARN("node.advertise pandar_packets");
    output_ = node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10000);
  }

  // raw packet output topic
  if (nodeType == LIDAR_NODE_TYPE) {
    // ROS_WARN("node.advertise pandar_gps");
    gpsoutput_ = node.advertise<pandar_msgs::PandarGps>("pandar_gps", 10);
  }
}

//-------------------------------------------------------------------------------
int parseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size) {
  if (size != GPS_PACKET_SIZE) {
    return -1;
  }

  int index = 0;
  packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
  index += GPS_PACKET_FLAG_SIZE;
  packet->year =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_YEAR_SIZE;
  packet->month =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MONTH_SIZE;
  packet->day =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_DAY_SIZE;
  packet->second =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_SECOND_SIZE;
  packet->minute =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MINUTE_SIZE;
  packet->hour =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_HOUR_SIZE;
  packet->fineTime =
      (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);

  return 0;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool PandarDriver::poll(void) {
  uint64_t startTime = 0;
  uint64_t endTime = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  // ROS_WARN("PandarDriver::poll(void)");

  // int readpacket = config_.npackets / 3;
  // int readpacket = 360;
  // ROS_WARN("PandarDriver::poll(void),config_.npackets[%d]",config_.npackets);
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  // pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);
  // scan->packets.resize(readpacket);
  // pandar_msgs::PandarPacketPtr sp_Packet(new pandar_msgs::PandarPacket);

  // Since the pandar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  // ROS_WARN("PandarDriver::poll(void),readpacket[%d]",readpacket);
  if(!m_bGetScanArraySizeFlag){
    m_iPandarScanArraySize = getPandarScanArraySize(m_spInput);
    pandarScanArray[m_iScanPushIndex]->packets.resize(m_iPandarScanArraySize);
    pandarScanArray[m_iScanPopIndex]->packets.resize(m_iPandarScanArraySize);
    m_bGetScanArraySizeFlag = true;
  }
  for (int i = 0; i < m_iPandarScanArraySize; ++i) {
    // while (true)
    // {
    // keep reading until full packet received
    PandarPacket packet;
    int rc = m_spInput->getPacket(&packet);
    if(rc == 0){
      pandarScanArray[m_iScanPushIndex]->packets[i].stamp = packet.stamp;
      pandarScanArray[m_iScanPushIndex]->packets[i].size = packet.size;
      pandarScanArray[m_iScanPushIndex]->packets[i].data.resize(packet.size);
      memcpy(&pandarScanArray[m_iScanPushIndex]->packets[i].data[0], &packet.data[0], packet.size);
    }

    // ROS_WARN("PandarDriver::poll(void),rc[%d]",rc);
    // if (rc == 0) break;       // got a full packet?
    if (rc == 2) {
      // gps packet;
      PandarGPS packet;
      if (parseGPS(&packet,
                   &pandarScanArray[m_iScanPushIndex]->packets[i].data[0],
                   GPS_PACKET_SIZE) == 0) {
        pandar_msgs::PandarGpsPtr gps(new pandar_msgs::PandarGps);
        gps->stamp = ros::Time::now();

        gps->year = packet.year;
        gps->month = packet.month;
        gps->day = packet.day;
        gps->hour = packet.hour;
        gps->minute = packet.minute;
        gps->second = packet.second;

        gps->used = 0;
        if (gps->year > 30 || gps->year < 17) {
          // ROS_ERROR("Ignore wrong GPS data (year)%d" , gps->year);
          return true;
          // continue;
        }
        convert->processGps(*gps);
        // gpsoutput_.publish(gps);
      }
      i--;
      continue;
    }
    if (rc > 0) return false;  // end of file reached?
    // }

    if (publishmodel == "both_point_raw" || publishmodel == "points") {
      convert->pushLiDARData(pandarScanArray[m_iScanPushIndex]->packets[i]);
    }
  }

  if (publishmodel == "both_point_raw" || publishmodel == "raw") {
      int temp;
      temp = m_iScanPushIndex;
      m_iScanPushIndex = m_iScanPopIndex;
      m_iScanPopIndex = temp;
      if (m_bNeedPublish == false)
        m_bNeedPublish = true;
      else
        ROS_WARN(
            "CPU not fast enough, data not published yet, new data "
            "comming!!!!!!!!!!!!!!");
    }

  
  return true;
}

void PandarDriver::publishRawData() {
  // uint32_t start = GetTickCount();

  if (m_bNeedPublish) {
    // ROS_WARN("PandarDriver::publishRawData()[%d]", m_iScanPopIndex);
    pandarScanArray[m_iScanPopIndex]->header.stamp =
        pandarScanArray[m_iScanPopIndex]->packets[m_iPandarScanArraySize - 1].stamp;
    pandarScanArray[m_iScanPopIndex]->header.frame_id = config_.frame_id;
    output_.publish(pandarScanArray[m_iScanPopIndex]);

    m_bNeedPublish = false;
  } else {
    usleep(1000);
  }

  // uint32_t end = GetTickCount();
  // uint32_t delta = end - start;
  // if (delta > 50) ROS_WARN("publishRawData, time %dms", delta);
}

void PandarDriver::setUdpVersion(uint8_t major, uint8_t minor) {
	m_spInput->setUdpVersion(major, minor);
}

void PandarDriver::callback(pandar_pointcloud::CloudNodeConfig &config,
                            uint32_t level) {
  ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

int PandarDriver::getPandarScanArraySize(boost::shared_ptr<Input> m_spInput){
  for (int i = 0; i < 256; ++i) {
    PandarPacket packet;
    int rc = m_spInput->getPacket(&packet);
    switch (packet.data[PANDAR_MAJOR_VERSION_INDEX])
    {
    case UDP_VERSION_MAJOR_1:
      switch (packet.data[PANDAR_LASER_NUMBER_INDEX]){
        case PANDAR128_LASER_NUM:
          return PANDAR128_READ_PACKET_SIZE;
        case PANDAR80_LASER_NUM:
          return PANDAR80_READ_PACKET_SIZE;
        case PANDAR64S_LASER_NUM:
          return PANDAR64S_READ_PACKET_SIZE;
        case PANDAR40S_LASER_NUM:
          return PANDAR40S_READ_PACKET_SIZE;
        case PANDAR90_LASER_NUM:
          return PANDAR90_READ_PACKET_SIZE;
        default:
          break;
      }
    case UDP_VERSION_MAJOR_3:
    return PANDARQT128_READ_PACKET_SIZE;
    default:
      break;
    }
  }
}
}  // namespace pandar_driver
