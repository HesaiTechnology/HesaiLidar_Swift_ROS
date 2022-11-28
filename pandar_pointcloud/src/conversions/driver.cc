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

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;

  // open Pandar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    input_.reset(new pandar_pointcloud::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    m_bPaserPacp = true;                                              
  } else {
    // read data from live socket
    input_.reset(new pandar_pointcloud::InputSocket(private_nh, udp_port, 10010, multicast_ip));
    m_bPaserPacp = false;   
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
    m_iPandarScanArraySize = getPandarScanArraySize(input_);
    pandarScanArray[m_iScanPushIndex]->packets.resize(m_iPandarScanArraySize);
    pandarScanArray[m_iScanPopIndex]->packets.resize(m_iPandarScanArraySize);
    m_bGetScanArraySizeFlag = true;
  }
  bool skipSleep = true;
  for (int i = 0; i < m_iPandarScanArraySize; ++i) {
    // keep reading until full packet received
    PandarPacket packet;
    int rc = 0;
    if(skipSleep){
			usleep(1000);
		}
		if (m_bPaserPacp)  // have PCAP file?
		{
			int count = 0;
			while(convert->getIsSocketTimeout()&& count < 2000){
				// ROS_WARN("timeout %d", convert->getIsSocketTimeout());
				usleep(1000);
				count++;
			}
		}
    bool isSocketTimeout = convert->getIsSocketTimeout();
    rc = input_->getPacket(&packet, isSocketTimeout, skipSleep);
    convert->setIsSocketTimeout(isSocketTimeout);
    if(rc == ERROR_PACKET){    // error packet
			i--;
			continue;
		} 
    if(rc == FAULT_MESSAGE_PACKET) {
      // convert->processFaultMessage(packet);
			i--;
			continue;
		}
		if(rc == LOG_REPORT_PACKET) {
			i--;
			continue;
		}
		
		if(rc == PCAP_END_PACKET){
			// convert->SetIsReadPcapOver(true);
			return true;
		}
			
		if(packet.size < 500){
			i--;
			continue;
		}
    pandarScanArray[m_iScanPushIndex]->packets[i].stamp = packet.stamp;
    pandarScanArray[m_iScanPushIndex]->packets[i].size = packet.size;
    pandarScanArray[m_iScanPushIndex]->packets[i].data.resize(packet.size);
    memcpy(&pandarScanArray[m_iScanPushIndex]->packets[i].data[0], &packet.data[0], packet.size);

    if(rc == GPS_PACKET) {
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

    if (publishmodel == "both_point_raw" || publishmodel == "point") {
      convert->pushLiDARData(pandarScanArray[m_iScanPushIndex]->packets[i]);
    }
  }

  int temp;
  temp = m_iScanPushIndex;
  m_iScanPushIndex = m_iScanPopIndex;
  m_iScanPopIndex = temp;
  if (m_bNeedPublish == false)
    m_bNeedPublish = true;
  // else
  //   ROS_WARN(
  //       "CPU not fast enough, data not published yet, new data "
  //       "comming!!!!!!!!!!!!!!");
  return true;
}

void PandarDriver::setUdpVersion(uint8_t major, uint8_t minor) {
	input_->setUdpVersion(major, minor);
}

void PandarDriver::publishRawData() {
  uint32_t start = GetTickCount();

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

  uint32_t end = GetTickCount();
  uint32_t delta = end - start;
  // if (delta > 50) ROS_WARN("publishRawData, time %dms", delta);
}

void PandarDriver::callback(pandar_pointcloud::CloudNodeConfig &config,
                            uint32_t level) {
  ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

int PandarDriver::getPandarScanArraySize(boost::shared_ptr<Input> input_){
  for (int i = 0; i < 256; ++i) {
    PandarPacket packet;
    bool isTimeout = false;
    bool isSkipSleep = true;
    int rc = input_->getPacket(&packet, isTimeout, isSkipSleep);
    switch (packet.data[PANDAR_LASER_NUMBER_INDEX]){
    case PANDAR128_LASER_NUM:
      return PANDAR128_READ_PACKET_SIZE;
    case PANDAR80_LASER_NUM:
      return PANDAR80_READ_PACKET_SIZE;
    case PANDAR64S_LASER_NUM:
      return PANDAR64S_READ_PACKET_SIZE;
    case PANDAR40S_LASER_NUM:
      return PANDAR40S_READ_PACKET_SIZE;
    default:
      break;
    }
  }
}
}  // namespace pandar_driver
