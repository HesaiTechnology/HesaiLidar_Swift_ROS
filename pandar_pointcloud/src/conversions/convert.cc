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

    This class converts raw Pandar128 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <image_transport/image_transport.h>
#include <pandar_pointcloud/platUtil.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include "taskflow.hpp"
// #define PRINT_FLAG 

namespace pandar_pointcloud {

static tf::Executor executor;

float degreeToRadian(float degree) { return degree * M_PI / 180.0f; }

static const float elev_angle[] = {
    14.436f,  13.535f,  13.08f,   12.624f,  12.163f,  11.702f,  11.237f,
    10.771f,  10.301f,  9.83f,    9.355f,   8.88f,    8.401f,   7.921f,
    7.437f,   6.954f,   6.467f,   5.98f,    5.487f,   4.997f,   4.501f,
    4.009f,   3.509f,   3.014f,   2.512f,   2.014f,   1.885f,   1.761f,
    1.637f,   1.511f,   1.386f,   1.258f,   1.13f,    1.009f,   0.88f,
    0.756f,   0.63f,    0.505f,   0.379f,   0.251f,   0.124f,   0.0f,
    -0.129f,  -0.254f,  -0.38f,   -0.506f,  -0.632f,  -0.76f,   -0.887f,
    -1.012f,  -1.141f,  -1.266f,  -1.393f,  -1.519f,  -1.646f,  -1.773f,
    -1.901f,  -2.027f,  -2.155f,  -2.282f,  -2.409f,  -2.535f,  -2.662f,
    -2.789f,  -2.916f,  -3.044f,  -3.172f,  -3.299f,  -3.425f,  -3.552f,
    -3.680f,  -3.806f,  -3.933f,  -4.062f,  -4.190f,  -4.318f,  -4.444f,
    -4.571f,  -4.698f,  -4.824f,  -4.951f,  -5.081f,  -5.209f,  -5.336f,
    -5.463f,  -5.589f,  -5.717f,  -5.843f,  -5.968f,  -6.099f,  -6.607f,
    -7.118f,  -7.624f,  -8.135f,  -8.64f,   -9.149f,  -9.652f,  -10.16f,
    -10.664f, -11.17f,  -11.67f,  -12.174f, -12.672f, -13.173f, -13.668f,
    -14.166f, -14.658f, -15.154f, -15.643f, -16.135f, -16.62f,  -17.108f,
    -17.59f,  -18.073f, -18.548f, -19.031f, -19.501f, -19.981f, -20.445f,
    -20.92f,  -21.379f, -21.85f,  -22.304f, -22.77f,  -23.219f, -23.68f,
    -24.123f, -25.016f,
};

static const float azimuth_offset[] = {
    3.257f,  3.263f, -1.083f, 3.268f, -1.086f, 3.273f,  -1.089f, 3.278f,
    -1.092f, 3.283f, -1.094f, 3.288f, -1.097f, 3.291f,  -1.1f,   1.1f,
    -1.102f, 1.1f,   -3.306f, 1.102f, -3.311f, 1.103f,  -3.318f, 1.105f,
    -3.324f, 1.106f, 7.72f,   5.535f, 3.325f,  -3.33f,  -1.114f, -5.538f,
    -7.726f, 1.108f, 7.731f,  5.543f, 3.329f,  -3.336f, -1.116f, -5.547f,
    -7.738f, 1.108f, 7.743f,  5.551f, 3.335f,  -3.342f, -1.119f, -5.555f,
    -7.75f,  1.11f,  7.757f,  5.56f,  3.34f,   -3.347f, -1.121f, -5.564f,
    -7.762f, 1.111f, 7.768f,  5.569f, 3.345f,  -3.353f, -1.123f, -5.573f,
    -7.775f, 1.113f, 7.780f,  5.578f, 3.351f,  -3.358f, -1.125f, -5.582f,
    -7.787f, 1.115f, 7.792f,  5.586f, 3.356f,  -3.363f, -1.126f, -5.591f,
    -7.799f, 1.117f, 7.804f,  5.595f, 3.36f,   -3.369f, -1.128f, -5.599f,
    -7.811f, 1.119f, -3.374f, 1.12f,  -3.379f, 1.122f,  -3.383f, 3.381f,
    -3.388f, 3.386f, -1.135f, 3.39f,  -1.137f, 3.395f,  -1.138f, 3.401f,
    -1.139f, 3.406f, -1.14f,  3.41f,  -1.141f, 3.416f,  -1.142f, 1.14f,
    -1.143f, 1.143f, -3.426f, 1.146f, -3.429f, 1.147f,  -3.433f, 1.15f,
    -3.436f, 1.152f, -3.44f,  1.154f, -3.443f, 1.157f,  -3.446f, -3.449f,
};

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh,
                 std::string node_type)
    : data_(new pandar_rawdata::RawData()),
      drv(node, private_nh, node_type, this) {
  
  m_sRosVersion = "PandarSwiftROS_1.0.17";
  ROS_WARN("--------PandarSwift ROS version: %s--------\n\n",m_sRosVersion.c_str());

  publishmodel = "";
  if (LIDAR_NODE_TYPE == node_type) {
    private_nh.getParam("publish_model", publishmodel);
    double start_angle;
    private_nh.param("start_angle", start_angle, 0.0);
    m_iLidarRotationStartAngle = int(start_angle * 100);
    data_->setup(private_nh);
    // advertise output point cloud (before subscribing to input data)
    srv_ = boost::make_shared<
        dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig> >(
        private_nh);
    dynamic_reconfigure::Server<
        pandar_pointcloud::CloudNodeConfig>::CallbackType f;
    f = boost::bind(&Convert::callback, this, _1, _2);
    srv_->setCallback(f);
  }

  private_nh.getParam("device_ip", m_sDeviceIp);
  private_nh.getParam("frame_id", m_sFrameId);
  private_nh.getParam("firetime_file", lidarFiretimeFile);
  private_nh.getParam("calibration", lidarCorrectionFile);
  private_nh.getParam("pcap", m_sPcapFile);

  std::string cert;
  std::string privateKey;
  std::string ca;
  private_nh.getParam("cert_file", cert);
  private_nh.getParam("private_key_file", privateKey);
  private_nh.getParam("ca_file", ca);
  private_nh.getParam("view_mode", m_iViewMode);
  TcpCommandSetSsl(cert.c_str(), privateKey.c_str(), ca.c_str());
  
  ROS_WARN("frame_id [%s]", m_sFrameId.c_str());
  ROS_WARN("lidarFiretimeFile [%s]", lidarFiretimeFile.c_str());
  ROS_WARN("lidarCorrectionFile [%s]", lidarCorrectionFile.c_str());
  SetEnvironmentVariableTZ();

  m_iWorkMode = 0;
  m_iReturnMode = 0;
  m_iMotorSpeed = 0;
  m_iLaserNum = 0;
  m_iBlockNum = 0;
  m_iFirstAzimuthIndex = 0;
  m_iLastAzimuthIndex = 0;
  m_iTotalPointsNum = 0;
  m_bClockwise == true;

  hasGps = 0;
  // subscribe to PandarScan packets
  // pandar_scan_ =
  //     node.subscribe("pandar_packets", 100,
  //                    &Convert::processScan, (Convert *) this,
  //                    ros::TransportHints().tcpNoDelay(true));
  // pandar_gps_ =
  //     node.subscribe("pandar_gps", 1,
  //                    &Convert::processGps, (Convert *) this,
  //                    ros::TransportHints().tcpNoDelay(true));

  tz_second_ = 0 * 3600;  // time zone
  start_angle_ = 0;

  for (int i = 0; i < PANDAR128_LASER_NUM; i++) {
    elev_angle_[i] = elev_angle[i];
    horizatal_azimuth_[i] = azimuth_offset[i];
  }

  memset(cos_all_angle_, 0, sizeof(cos_all_angle_));
  memset(sin_all_angle_, 0, sizeof(sin_all_angle_));

  for (int j = 0; j < CIRCLE; j++) {
    float angle = static_cast<float>(j) / 100.0f;
    cos_all_angle_[j] = cosf(degreeToRadian(angle));
    sin_all_angle_[j] = sinf(degreeToRadian(angle));
  }

  sem_init(&picsem, 0, 0);
  pthread_mutex_init(&piclock, NULL);
  pthread_mutex_init(&m_RedundantPointLock, NULL);

  bool loadCorrectionFileSuccess = false;
  int ret;
  if(m_sPcapFile.empty()) {
    m_pTcpCommandClient =TcpCommandClientNew(m_sDeviceIp.c_str(), PANDARSDK_TCP_COMMAND_PORT);
    if(NULL != m_pTcpCommandClient) {
      char *buffer = NULL;
      uint32_t len = 0;
      std::string correntionString;
      ret = TcpCommandGetLidarCalibration(m_pTcpCommandClient, &buffer, &len);
      if (ret == 0 && buffer) {
        ROS_WARN("Load correction file from lidar now!");
        ret = loadCorrectionFile(buffer);
          if (ret != 0) {
            ROS_WARN("Parse Lidar Correction Error");
          } 
          else {
            loadCorrectionFileSuccess = true;
            ROS_WARN("Parse Lidar Correction Success!!!");
          }
        free(buffer);
      }
      else{
        ROS_WARN("Get lidar calibration filed");
      }
    }
  }
  if(!loadCorrectionFileSuccess) {
    ROS_WARN("load correction file from local correction.csv now!");
    std::ifstream fin(lidarCorrectionFile);
    if (fin.is_open()) {
      ROS_WARN("Open correction file success\n");
      int length = 0;
      std::string strlidarCalibration;
      fin.seekg(0, std::ios::end);
      length = fin.tellg();
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      ret = loadCorrectionFile(buffer);
      if (ret != 0) {
        ROS_WARN("Parse local Correction file Error");
      } 
      else {
        ROS_WARN("Parse local Correction file Success!!!");
      }
    }
    else{
      ROS_WARN("Open correction file failed\n");
    }
  }

  loadOffsetFile(
      lidarFiretimeFile);  // parameter is the path of lidarFiretimeFil
  ROS_WARN("node_type[%s]", node_type.c_str());


  if (LIDAR_NODE_TYPE == node_type) {
    boost::thread thrd(boost::bind(&Convert::DriverReadThread, this));
  }

  if (publishmodel == "both_point_raw" || publishmodel == "point" ||
      LIDAR_NODE_TYPE != node_type) {
    ROS_WARN("node.advertise pandar_points");
    output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10000);
    boost::thread processThr(boost::bind(&Convert::processLiDARData, this));
  }

  m_iPublishPointsIndex = 0;
  // m_bPublishPointsFlag = false;
  // boost::thread publishPointsThr(
  //     boost::bind(&Convert::publishPointsThread, this));

  if ((publishmodel == "both_point_raw" || publishmodel == "raw") &&
      LIDAR_NODE_TYPE == node_type) {
    boost::thread processThr(boost::bind(&Convert::publishRawDataThread, this));
  }
}

int Convert::loadCorrectionFile(char * data) {
  try
    {
      char * p = data;
      PandarATCorrectionsHeader header = *(PandarATCorrectionsHeader *)p;
      if ( 0xee == header.delimiter[0] && 0xff == header.delimiter[1])
      {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          memcpy((void *)&m_PandarAT_corrections.start_frame, p, sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.end_frame, p, sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          ROS_WARN( "frame_num: %d" ,frame_num);
          ROS_WARN( "start_frame, end_frame: ");
          for (int i = 0; i < frame_num; ++i) 
              ROS_WARN("%lf,   %lf", m_PandarAT_corrections.start_frame[i] / 100.f , m_PandarAT_corrections.end_frame[i] / 100.f);
          memcpy((void *)&m_PandarAT_corrections.azimuth, p, sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.elevation, p, sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p, sizeof(int8_t) * 36000);
          p += sizeof(int8_t) * 36000;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p, sizeof(int8_t) * 36000);
          p += sizeof(int8_t) * 36000;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p, sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;

          for (int i = 0; i < channel_num; i++) {
              horizatal_azimuth_[i] = m_PandarAT_corrections.azimuth[i] / 100.f;
              elev_angle_[i] = m_PandarAT_corrections.elevation[i] / 100.f;
              
          }
          return 0;
      }
        
      return 1;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
    
    return 1;

}

void Convert::DriverReadThread() {
  sched_param param;
  param.sched_priority = 99;
  // SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
  printf("DriverReadThread:set result [%d]", rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("DriverReadThread:get thead %lu, policy %d and priority %d\n",
           pthread_self(), ret_policy, param.sched_priority);

  while (1) {
    drv.poll();
  }
}

void Convert::publishRawDataThread() {
  sched_param param;
  param.sched_priority = 90;
  // SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  ROS_WARN("publishRawDataThread:set result [%d]", rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("publishRawDataThread:get thead %lu, policy %d and priority %d\n",
           pthread_self(), ret_policy, param.sched_priority);

  while (1) {
    drv.publishRawData();
  }
}

void Convert::callback(pandar_pointcloud::CloudNodeConfig &config,
                       uint32_t level) {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range,
                       config.view_direction, config.view_width);
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg) {
  if (output_.getNumSubscribers() == 0)  // no one listening?
    return;                              // avoid much work

  // allocate a point cloud with same time and frame ID as raw data
  pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
  // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
  outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
  // outMsg->is_dense = false;
  outMsg->header.frame_id = scanMsg->header.frame_id;
  outMsg->height = 1;

  double firstStamp = 0.0f;
  int ret = data_->unpack(scanMsg, *outMsg, gps1, gps2, firstStamp,
                          m_iLidarRotationStartAngle);

  // publish the accumulated cloud message
  ROS_DEBUG_STREAM("Publishing "
                   << outMsg->height * outMsg->width
                   << " Pandar128 points, time: " << outMsg->header.stamp);

  if (ret == 1) {
    pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
    // output_.publish(outMsg);
  }
}

void Convert::processGps(pandar_msgs::PandarGps &gpsMsg) {
  struct tm t;
  t.tm_sec = gpsMsg.second;
  t.tm_min = gpsMsg.minute;
  t.tm_hour = gpsMsg.hour;
  t.tm_mday = gpsMsg.day;
  t.tm_mon = gpsMsg.month - 1;
  t.tm_year = gpsMsg.year + 2000 - 1900;
  t.tm_isdst = 0;
  if (lastGPSSecond != (mktime(&t) + 1)) {
    lastGPSSecond = (mktime(&t) + 1);
    gps2.gps = mktime(&t) + 1;  // the gps always is the last gps, the newest
                                // GPS data is after the PPS(Serial port
                                // transmition speed...)
    gps2.used = 0;
  }
  // ROS_ERROR("Got a gps data %d " ,gps2.gps);
}

void Convert::pushLiDARData(pandar_msgs::PandarPacket packet) {
  //  ROS_WARN("Convert::pushLiDARData");
  m_PacketsBuffer.push_back(packet);
}

int Convert::parseData(Pandar128Packet &packet, const uint8_t *recvbuf,
                       const int len) {
  int index = 0;

  if (recvbuf[0] != 0xEE && recvbuf[1] != 0xFF && recvbuf[2] != 1 ) {    
    ROS_WARN("Lidar type is error\n");
    return -1;
  }

  memcpy(&(packet.head), &recvbuf[index], PANDAR128_HEAD_SIZE);
  
  index += PANDAR128_HEAD_SIZE;
  memcpy(&(packet.blocks), &recvbuf[index], PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM);
  
  index += PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM + PANDAR128_CRC_SIZE + PANDAR128_FUNCTION_SAFETY_SIZE;
  memcpy(&(packet.tail.nReserved1[0]),  &recvbuf[index], 3);

  index += 3;
  memcpy(&(packet.tail.nReserved2[0]),  &recvbuf[index], 3);

  index += 3 + 3 + 2;
  packet.tail.nShutdownFlag = recvbuf[index];

  index += 1;
  packet.tail.nReturnMode = recvbuf[index];

  index += 1;
  packet.tail.nMotorSpeed = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

  index += 2;
  memcpy(&(packet.tail.nUTCTime[0]),  &recvbuf[index], 6);

  index += 6;
  memcpy(&(packet.tail.nTimestamp),  &recvbuf[index], 4);

  index += 4 + 1;
  memcpy(&(packet.tail.nSeqNum),  &recvbuf[index], 4);

  // ROS_WARN("timestamp[%u]",packet.tail.nTimestamp);
  return 0;
}

int Convert::processLiDARData() {
  double lastTimestamp = 0.0;
  struct timespec ts;
  int ret = 0;

  int pktCount = 0;
  int cursor = 0;
  init();
  sched_param param;
  param.sched_priority = 91;
  // SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  ROS_WARN("processLiDARData:pthread_setschedparam result [%d]", rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("processLiDARData:get thead %lu, policy %d and priority %d\n",
           pthread_self(), ret_policy, param.sched_priority);
  while (1) {
    if (!m_PacketsBuffer.hasEnoughPackets()) {
      usleep(1000);
      continue;
    }

    if (0 == checkLiadaMode()) {
      // ROS_WARN("checkLiadaMode now!!");
      m_OutMsgArray[cursor]->clear();
      m_OutMsgArray[cursor]->resize(calculatePointBufferSize());
      m_PacketsBuffer.creatNewTask();
      pktCount = 0;
      continue;
    }
		if(isNeedPublish()){
    	uint32_t startTick1 = GetTickCount();
			moveTaskEndToStartAngle();
			doTaskFlow(cursor);
			uint32_t startTick2 = GetTickCount();
			// printf("move and taskflow time:%d\n", startTick2 - startTick1);
      m_iPublishPointsIndex = cursor;
      cursor = (cursor + 1) % 2;
      publishPoints();
        
			m_OutMsgArray[cursor]->clear();
			m_OutMsgArray[cursor]->resize(calculatePointBufferSize());
      if(m_RedundantPointBuffer.size() > 0 && m_RedundantPointBuffer.size() < MAX_REDUNDANT_POINT_NUM){
        for(int i = 0; i < m_RedundantPointBuffer.size(); i++){
          m_OutMsgArray[cursor]->points[m_RedundantPointBuffer[i].index] = m_RedundantPointBuffer[i].point;
        }
      }
      m_RedundantPointBuffer.clear();
			uint32_t endTick2 = GetTickCount();
			if(endTick2 - startTick2 > 2) {
				// printf("m_OutMsgArray time:%d\n", endTick2 - startTick2);
			}
			m_OutMsgArray[cursor]->header.frame_id = m_sFrameId;
			m_OutMsgArray[cursor]->height = 1;
			continue;
		}
    // uint32_t taskflow1 = GetTickCount();
			// printf("if compare time: %d\n", ifTick - startTick);
		doTaskFlow(cursor);
		// uint32_t taskflow2 = GetTickCount();
			// printf("taskflow time: %d\n", taskflow2 - taskflow1);
  }
}

void Convert::moveTaskEndToStartAngle() {
	uint32_t startTick = GetTickCount();
  if(m_iViewMode == 1){
    for(PktArray::iterator iter = m_PacketsBuffer.m_iterTaskBegin; iter < m_PacketsBuffer.m_iterTaskEnd; iter++) {
      for(int i = 0; i < m_PandarAT_corrections.header.frame_number; i++){
        if((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) < (m_PandarAT_corrections.start_frame[i])) &&
          ((m_PandarAT_corrections.start_frame[i]) <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) ||
          (*(uint16_t*)(&((iter)->data[0]) + m_iFirstAzimuthIndex) > *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) &&
          ((m_PandarAT_corrections.start_frame[i]) <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex))){
            m_PacketsBuffer.moveTaskEnd(iter + 1);
            break;
        }
      }
    }
  }
  else{
    for(PktArray::iterator iter = m_PacketsBuffer.m_iterTaskBegin; iter < m_PacketsBuffer.m_iterTaskEnd; iter++) {
      if ((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) < m_PandarAT_corrections.start_frame[0]) &&
        (m_PandarAT_corrections.start_frame[0] <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) || 
        (*(uint16_t*)(&((iter)->data[0]) + m_iFirstAzimuthIndex) > *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) &&
        ((m_PandarAT_corrections.start_frame[0]) <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex))) {
        // printf("move iter to : %d\n", (iter + 1)->blocks[0].fAzimuth);
        m_PacketsBuffer.moveTaskEnd(iter + 1);
        break;
      }
    }
  }
	uint32_t endTick = GetTickCount();
	// printf("moveTaskEndToStartAngle time: %d\n", endTick - startTick);
}

void Convert::init() {
	while (1) {
		if(!m_PacketsBuffer.hasEnoughPackets()) {
			usleep(1000);
			continue;
		}
		uint16_t lidarmotorspeed = 0;
    auto header = (PandarAT128Head*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
    switch(header->u8VersionMinor){
			case 1:
			{
				auto tail = (PandarAT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR_AT128_HEAD_SIZE +
                    PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum + 
                    PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum );
				m_iWorkMode = tail->nShutdownFlag & 0x03;
				m_iReturnMode = tail->nReturnMode;
				lidarmotorspeed = tail->nMotorSpeed;
				m_iLaserNum = header->u8LaserNum;
				m_iFirstAzimuthIndex = PANDAR_AT128_HEAD_SIZE; 
				m_iLastAzimuthIndex = PANDAR_AT128_HEAD_SIZE + 
                              PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) + 
                              PANDAR_AT128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
			}
			break;
			case 2:
			{
				auto tail = (PandarAT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR_AT128_HEAD_SIZE +
                    (header->hasConfidence() ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) +
                    PANDAR_AT128_CRC_SIZE + 
                    PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum );
				m_iWorkMode = tail->nShutdownFlag & 0x03;
				m_iReturnMode = tail->nReturnMode;
				lidarmotorspeed = tail->nMotorSpeed;
				m_iLaserNum = header->u8LaserNum;
				m_iFirstAzimuthIndex = PANDAR_AT128_HEAD_SIZE;
				m_iLastAzimuthIndex = PANDAR_AT128_HEAD_SIZE + 
                              (header->hasConfidence() ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1): PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
                              PANDAR_AT128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
			}
			default:
			break;
		}
		if(abs(lidarmotorspeed - MOTOR_SPEED_600) < 30) { //ignore the speed gap of 600 rpm
			lidarmotorspeed = MOTOR_SPEED_600;
		}
    else if(abs(lidarmotorspeed - MOTOR_SPEED_500) < 30) { //ignore the speed gap of 500 rpm
    lidarmotorspeed = MOTOR_SPEED_500;
    }
		else if(abs(lidarmotorspeed - MOTOR_SPEED_400) < 30) { //ignore the speed gap of 400 rpm
    lidarmotorspeed = MOTOR_SPEED_400;
    }
    else if(abs(lidarmotorspeed - MOTOR_SPEED_300) < 30) { //ignore the speed gap of 300 rpm
      lidarmotorspeed = MOTOR_SPEED_300;
    }
    else if(abs(lidarmotorspeed - MOTOR_SPEED_200) < 30) { //ignore the speed gap of 200 rpm
      lidarmotorspeed = MOTOR_SPEED_200;
    }
		else {
			lidarmotorspeed = MOTOR_SPEED_200; //changing the speed,give enough size
		}
		m_iMotorSpeed = lidarmotorspeed;
		printf("init mode: workermode: %x,return mode: %x,speed: %d\n",m_iWorkMode, m_iReturnMode, m_iMotorSpeed);
		changeAngleSize();
		changeReturnBlockSize();
		boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(calculatePointBufferSize(), 1));
		boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(calculatePointBufferSize(), 1));
		m_OutMsgArray[0] = outMag0;
		m_OutMsgArray[1] = outMag1;
		break;
	}
}

void Convert::publishPoints() {
  // uint32_t start = GetTickCount();

  pcl_conversions::toPCL(ros::Time(timestamp),
                         m_OutMsgArray[m_iPublishPointsIndex]->header.stamp);
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*m_OutMsgArray[m_iPublishPointsIndex], output);
  output_.publish(output);
#ifdef PRINT_FLAG
  ROS_WARN("ts %lf cld size %u", timestamp, m_OutMsgArray[m_iPublishPointsIndex]->points.size());
#endif  
  timestamp = 0;

  // uint32_t end = GetTickCount();
  // if (end - start > 150) ROS_WARN("publishPoints time:%d", end - start);
}


void Convert::checkClockwise(){
  if((*(uint16_t*)(&(m_PacketsBuffer.m_iterTaskBegin->data[0]) + m_iFirstAzimuthIndex) < 
    (*(uint16_t*)(&((m_PacketsBuffer.m_iterTaskBegin + 1)->data[0]) + m_iFirstAzimuthIndex))) ||
    (*(uint16_t*)(&(m_PacketsBuffer.m_iterTaskBegin->data[0]) + m_iFirstAzimuthIndex) - *(uint16_t*)(&((m_PacketsBuffer.m_iterTaskBegin + 1)->data[0]) + m_iFirstAzimuthIndex)) > CIRCLE_ANGLE / 2)
  {
    m_bClockwise = true; //Clockwise
  }

  if((*(uint16_t*)(&(m_PacketsBuffer.m_iterTaskBegin->data[0]) + m_iFirstAzimuthIndex) > 
    (*(uint16_t*)(&((m_PacketsBuffer.m_iterTaskBegin + 1)->data[0]) + m_iFirstAzimuthIndex))) ||
    (*(uint16_t*)(&((m_PacketsBuffer.m_iterTaskBegin + 1)->data[0]) + m_iFirstAzimuthIndex) - *(uint16_t*)(&((m_PacketsBuffer.m_iterTaskBegin)->data[0]) + m_iFirstAzimuthIndex)) > CIRCLE_ANGLE / 2)
  {
    m_bClockwise = false; //countClockwise
  }

}

void Convert::doTaskFlow(int cursor) {
  tf::Taskflow taskFlow;
  taskFlow.parallel_for(m_PacketsBuffer.getTaskBegin(),
                        m_PacketsBuffer.getTaskEnd(),
                        [this, &cursor](auto &taskpkt) {
                          calcPointXYZIT(taskpkt,cursor);
                        });
  executor.run(taskFlow).wait();
  m_PacketsBuffer.creatNewTask();

}

int Convert::checkLiadaMode() {
  uint8_t lidarworkmode = 0;
  uint8_t lidarreturnmode = 0;
  uint16_t lidarmotorspeed = 0;
  uint8_t laserNum = 0;
  uint8_t blockNum = 0;
  auto header = (PandarAT128Head*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
  switch(header->u8VersionMinor){
	case 1:
	{
		auto tail = (PandarAT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR_AT128_HEAD_SIZE +
                PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum + 
                PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum );
		lidarworkmode = tail->nShutdownFlag & 0x03;
		lidarreturnmode = tail->nReturnMode;
		lidarmotorspeed = tail->nMotorSpeed;
		laserNum = header->u8LaserNum;
		blockNum = header->u8BlockNum;
		m_iFirstAzimuthIndex = PANDAR_AT128_HEAD_SIZE;
		m_iLastAzimuthIndex = PANDAR_AT128_HEAD_SIZE + 
                          PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) + 
                          PANDAR_AT128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
	}
	break;
	case 2:
	{
		auto tail = (PandarAT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR_AT128_HEAD_SIZE +
                (header->hasConfidence() ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) +
                PANDAR_AT128_CRC_SIZE + 
                PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum );
		lidarworkmode = tail->nShutdownFlag & 0x03;
		lidarreturnmode = tail->nReturnMode;
		lidarmotorspeed = tail->nMotorSpeed;
		laserNum = header->u8LaserNum;
		blockNum = header->u8BlockNum;
		m_iFirstAzimuthIndex = PANDAR_AT128_HEAD_SIZE;
		m_iLastAzimuthIndex = PANDAR_AT128_HEAD_SIZE + 
                          (header->hasConfidence() ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1): PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
                          PANDAR_AT128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
	}
	default:
	break;
  }
  if(abs(lidarmotorspeed - MOTOR_SPEED_600) < 30) { //ignore the speed gap of 600 rpm
    lidarmotorspeed = MOTOR_SPEED_600;
  }
  else if(abs(lidarmotorspeed - MOTOR_SPEED_500) < 30) { //ignore the speed gap of 500 rpm
    lidarmotorspeed = MOTOR_SPEED_500;
  }
  else if(abs(lidarmotorspeed - MOTOR_SPEED_400) < 30) { //ignore the speed gap of 400 rpm
    lidarmotorspeed = MOTOR_SPEED_400;
  }
  else if(abs(lidarmotorspeed - MOTOR_SPEED_300) < 30) { //ignore the speed gap of 300 rpm
    lidarmotorspeed = MOTOR_SPEED_300;
  }
  else if(abs(lidarmotorspeed - MOTOR_SPEED_200) < 30) { //ignore the speed gap of 200 rpm
    lidarmotorspeed = MOTOR_SPEED_200;
  }
  else {
      lidarmotorspeed = MOTOR_SPEED_200; //changing the speed,give enough size
  }

  if (0 == m_iWorkMode && 0 == m_iReturnMode && 0 == m_iMotorSpeed && 0 == m_iLaserNum) { //init lidar mode 
    m_iWorkMode = lidarworkmode;
    m_iReturnMode = lidarreturnmode;
    m_iMotorSpeed = lidarmotorspeed;
    m_iLaserNum = laserNum;
    m_iBlockNum = blockNum;
    ROS_WARN("init mode: workermode: %x,return mode: %x,speed: %d,laser number: %d",m_iWorkMode, m_iReturnMode, m_iMotorSpeed, m_iLaserNum);
    changeAngleSize();
    changeReturnBlockSize(); 
    checkClockwise();
    boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(calculatePointBufferSize(), 1));
    boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(calculatePointBufferSize(), 1));
    m_OutMsgArray[0] = outMag0;
    m_OutMsgArray[1] = outMag1;
    return 1;
  } 
  else { //mode change 
    if (m_iReturnMode != lidarreturnmode) { //return mode change
      ROS_WARN("change return mode:  %x to %x ",m_iReturnMode, lidarreturnmode);
      m_iReturnMode = lidarreturnmode;
      changeReturnBlockSize();
      return 0;
    }
    if (m_iMotorSpeed != lidarmotorspeed) { //motor speed change
      // ROS_WARN("change motor speed:  %d to %d ",m_iMotorSpeed, lidarmotorspeed);
      m_iMotorSpeed = lidarmotorspeed;
      changeAngleSize();
      return 0;
    }
    if (m_iLaserNum != laserNum) { //laser number change
      m_iLaserNum = laserNum;
      return 0;
    }
    return 1;
  }
}

void Convert::changeAngleSize() {
  if (MOTOR_SPEED_600 == m_iMotorSpeed || MOTOR_SPEED_400 == m_iMotorSpeed) {
    m_iAngleSize = LIDAR_ANGLE_SIZE_10;  // 10->0.1degree
  }
  else if(MOTOR_SPEED_300 == m_iMotorSpeed || MOTOR_SPEED_200 == m_iMotorSpeed){
    m_iAngleSize = LIDAR_ANGLE_SIZE_5;  // 5->0.05degree
  }
}

void Convert::changeReturnBlockSize() {
  if (0x39 == m_iReturnMode || 0x3b == m_iReturnMode || 0x3c == m_iReturnMode) {
    m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_2;
  } else {
    m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_1;
  }
}

void Convert::calcPointXYZIT(pandar_msgs::PandarPacket &packet, int cursor) {
  if(packet.data[0] != 0xEE || packet.data[1] != 0xFF)
    return;
  auto header = (PandarAT128Head*)(&packet.data[0]);
  PandarAT128Tail* tail = nullptr;  
  switch(header->u8VersionMinor){
    case 1:
    {
      tail = (PandarAT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR_AT128_HEAD_SIZE +
            PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum + 
            PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum );
    }
    break;
    case 2:
    {
      tail = (PandarAT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR_AT128_HEAD_SIZE +
            (header->hasConfidence() ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) +
            PANDAR_AT128_CRC_SIZE + 
            PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum );

    }
  }
      // ROS_WARN(" return mode:  %x, speed :%d ",tail->nReturnMode, tail->nMotorSpeed);         
  struct tm t = {0};
  t.tm_year = tail->nUTCTime[0];
  if (t.tm_year >= 200) {
    t.tm_year -= 100;
  }
  t.tm_mon = tail->nUTCTime[1] - 1;
  t.tm_mday = tail->nUTCTime[2];
  t.tm_hour = tail->nUTCTime[3];
  t.tm_min = tail->nUTCTime[4];
  t.tm_sec = tail->nUTCTime[5];
  t.tm_isdst = 0;

  double unix_second = static_cast<double>(mktime(&t) + tz_second_);
  int index = 0;
  index += PANDAR_AT128_HEAD_SIZE;
  for (int blockid = 0; blockid < header->u8BlockNum; blockid++) {
    uint16_t u16Azimuth = *(uint16_t*)(&packet.data[0] + index);
    // ROS_WARN("#####block.fAzimuth[%u]",u16Azimuth);
    int count = 0, field = 0;
    while ( count < m_PandarAT_corrections.header.frame_number
            && (
            ((u16Azimuth + CIRCLE_ANGLE  - m_PandarAT_corrections.start_frame[field]) % CIRCLE_ANGLE  + (m_PandarAT_corrections.end_frame[field] + CIRCLE_ANGLE  - u16Azimuth) % CIRCLE_ANGLE )
                != (m_PandarAT_corrections.end_frame[field] + CIRCLE_ANGLE  - m_PandarAT_corrections.start_frame[field]) % CIRCLE_ANGLE  )
    ) {
        field = (field + 1) % m_PandarAT_corrections.header.frame_number;
        count++;
    }
    if (count >= m_PandarAT_corrections.header.frame_number)
        continue;
    index += PANDAR_AT128_AZIMUTH_SIZE;
    for (int i = 0; i < header->u8LaserNum; i++) {
      /* for all the units in a block */
      uint16_t u16Distance = *(uint16_t*)(&packet.data[0] + index);
      index += DISTANCE_SIZE;
      uint8_t u8Intensity = *(uint8_t*)(&packet.data[0] + index);
      index += INTENSITY_SIZE;
      if(header->u8VersionMinor == 1 || header->hasConfidence())
        index += CONFIDENCE_SIZE;
      PPoint point;
      float distance =
          static_cast<float>(u16Distance) * PANDAR128_DISTANCE_UNIT;
      auto elevation = m_iViewMode == 0 ?
                m_PandarAT_corrections.elevation[i] * m_PandarAT_corrections.header.resolution : 
              (m_PandarAT_corrections.elevation[i] + m_PandarAT_corrections.elevation_offset[u16Azimuth]) * m_PandarAT_corrections.header.resolution;
      elevation = (CIRCLE_ANGLE + elevation) % CIRCLE_ANGLE;    
      auto azimuth = m_iViewMode == 0 ?
          (u16Azimuth + CIRCLE_ANGLE  - m_PandarAT_corrections.azimuth[i] / 2) % CIRCLE_ANGLE  * m_PandarAT_corrections.header.resolution : 
          ((u16Azimuth + CIRCLE_ANGLE  - m_PandarAT_corrections.start_frame[field]) * 2
          - m_PandarAT_corrections.azimuth[i]
          + m_PandarAT_corrections.azimuth_offset[u16Azimuth]) * m_PandarAT_corrections.header.resolution;
      azimuth = (CIRCLE_ANGLE  + azimuth) % CIRCLE_ANGLE ;
      float xyDistance =
          distance * cos_all_angle_[(elevation)];
      point.x = xyDistance * sin_all_angle_[(azimuth)];
      point.y = xyDistance * cos_all_angle_[(azimuth)];
      point.z = distance * sin_all_angle_[(elevation)];

      point.intensity = u8Intensity;
      point.timestamp =
          unix_second + (static_cast<double>(tail->nTimestamp)) / 1000000.0;

      if (0 == timestamp) {
        timestamp = point.timestamp;
      } else if (timestamp > point.timestamp) {
        timestamp = point.timestamp;
     }

      point.ring = i + 1;
      int point_index;
      point_index = calculatePointIndex(u16Azimuth, blockid, i);
      if(m_OutMsgArray[cursor]->points[point_index].ring == 0){
        m_OutMsgArray[cursor]->points[point_index] = point;
      }
      else{
        pthread_mutex_lock(&m_RedundantPointLock);
        m_RedundantPointBuffer.push_back(RedundantPoint{point_index, point});
        pthread_mutex_unlock(&m_RedundantPointLock);
      }
    }
  }
}

bool Convert::isNeedPublish(){
  uint16_t beginAzimuth = *(uint16_t*)(&(m_PacketsBuffer.getTaskBegin()->data[0]) + m_iFirstAzimuthIndex);
  uint16_t endAzimuth = *(uint16_t*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + m_iLastAzimuthIndex);
  if(m_iViewMode == 1){
    for(int i = 0; i < m_PandarAT_corrections.header.frame_number; i++){
      if((abs(endAzimuth - (m_PandarAT_corrections.start_frame[i])) <= m_iAngleSize) || 
        (beginAzimuth < (m_PandarAT_corrections.start_frame[i])) && ((m_PandarAT_corrections.start_frame[i]) <= endAzimuth) ||
        (endAzimuth < beginAzimuth && (endAzimuth + CIRCLE_ANGLE - beginAzimuth) > PANDAR_AT128_FRAME_ANGLE_SIZE))
            return true;
    }
    return false;
  }
  else{
    if((abs(endAzimuth - m_PandarAT_corrections.start_frame[0]) <= m_iAngleSize) || 
        (beginAzimuth < m_PandarAT_corrections.start_frame[0]) && (m_PandarAT_corrections.start_frame[0] <= endAzimuth) ||
        (endAzimuth < beginAzimuth && (endAzimuth + CIRCLE_ANGLE - beginAzimuth) > PANDAR_AT128_FRAME_ANGLE_SIZE)){
      return true;
    }
    return false;

  }
}


int Convert::calculatePointIndex(uint16_t u16Azimuth, int blockid, int laserid){
  int point_index = 0;
  switch(m_PandarAT_corrections.header.frame_number){
    case 4: // two mirror case
      if (LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
        if(m_iViewMode == 1){
          point_index = ((u16Azimuth  + PANDAR_AT128_FRAME_ANGLE_SIZE / 2) % 9000) % PANDAR_AT128_FRAME_ANGLE_SIZE / m_iAngleSize * m_iLaserNum *
                    m_iReturnBlockSize +
                m_iLaserNum * (blockid % 2) + laserid;
        }
        else{
          if(u16Azimuth >= (CIRCLE_ANGLE - PANDAR_AT128_FRAME_ANGLE_SIZE / 2) || u16Azimuth <= (PANDAR_AT128_FRAME_ANGLE_SIZE / 2)){
            point_index = ((u16Azimuth  + PANDAR_AT128_FRAME_ANGLE_SIZE / 2) % 9000) % PANDAR_AT128_FRAME_ANGLE_SIZE / m_iAngleSize * m_iLaserNum *
                      m_iReturnBlockSize +
                  m_iLaserNum * (blockid % 2) + laserid;
          }
          else{
            point_index = (((u16Azimuth  + PANDAR_AT128_FRAME_ANGLE_SIZE / 2) % 9000) % PANDAR_AT128_FRAME_ANGLE_SIZE + PANDAR_AT128_FRAME_ANGLE_SIZE) / m_iAngleSize * m_iLaserNum *
                      m_iReturnBlockSize +
                  m_iLaserNum * (blockid % 2) + laserid;
          }        
        }

      } else {
        if(m_iViewMode == 1){
          point_index = ((u16Azimuth  + PANDAR_AT128_FRAME_ANGLE_SIZE / 2) % 9000) % PANDAR_AT128_FRAME_ANGLE_SIZE  / m_iAngleSize * m_iLaserNum + laserid;
        }
        else{
          if(u16Azimuth >= (CIRCLE_ANGLE - PANDAR_AT128_FRAME_ANGLE_SIZE / 2) || u16Azimuth <= (PANDAR_AT128_FRAME_ANGLE_SIZE / 2)){
            point_index = ((u16Azimuth  + PANDAR_AT128_FRAME_ANGLE_SIZE / 2) % 9000) % PANDAR_AT128_FRAME_ANGLE_SIZE  / m_iAngleSize * m_iLaserNum + laserid;
          }
          else{
            point_index = (((u16Azimuth  + PANDAR_AT128_FRAME_ANGLE_SIZE / 2) % 9000) % PANDAR_AT128_FRAME_ANGLE_SIZE + PANDAR_AT128_FRAME_ANGLE_SIZE) / m_iAngleSize * m_iLaserNum + laserid;
          }        
        }
      }
    break;
    case 3: // three mirror case
      uint16_t Azimuth = (u16Azimuth + 3000) % CIRCLE_ANGLE;
      if (LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
        if(m_iViewMode == 1){
          point_index = (Azimuth) % PANDAR_AT128_FRAME_ANGLE_SIZE / m_iAngleSize * m_iLaserNum *
                    m_iReturnBlockSize +
                m_iLaserNum * (blockid % 2) + laserid;
        }
        else{
          if(Azimuth >= 0 && Azimuth <= (PANDAR_AT128_FRAME_ANGLE_SIZE)){
            point_index =  Azimuth / m_iAngleSize * m_iLaserNum *
                      m_iReturnBlockSize +
                  m_iLaserNum * (blockid % 2) + laserid;
          }
          else if (Azimuth > (PANDAR_AT128_FRAME_ANGLE_SIZE) && Azimuth <= (PANDAR_AT128_FRAME_ANGLE_SIZE * 3)){
            point_index = (Azimuth % PANDAR_AT128_FRAME_ANGLE_SIZE + PANDAR_AT128_FRAME_ANGLE_SIZE) / m_iAngleSize * m_iLaserNum *
                      m_iReturnBlockSize +
                  m_iLaserNum * (blockid % 2) + laserid;
          }
          else{
            point_index = (Azimuth % PANDAR_AT128_FRAME_ANGLE_SIZE + PANDAR_AT128_FRAME_ANGLE_SIZE * 2) / m_iAngleSize * m_iLaserNum *
                      m_iReturnBlockSize +
                  m_iLaserNum * (blockid % 2) + laserid;
          }        
        }

      } else {
        if(m_iViewMode == 1){
          point_index = (Azimuth) % PANDAR_AT128_FRAME_ANGLE_SIZE  / m_iAngleSize * m_iLaserNum + laserid;
        }
        else {
          if(Azimuth >= 0 && Azimuth <= (PANDAR_AT128_FRAME_ANGLE_SIZE)){
            point_index = (Azimuth)/ m_iAngleSize * m_iLaserNum + laserid;
          }
          else if (Azimuth > (PANDAR_AT128_FRAME_ANGLE_SIZE) && Azimuth <= (PANDAR_AT128_FRAME_ANGLE_SIZE * 3)){
            point_index = (Azimuth % PANDAR_AT128_FRAME_ANGLE_SIZE + PANDAR_AT128_FRAME_ANGLE_SIZE) / m_iAngleSize * m_iLaserNum + laserid;
          }
          else{
            point_index = (Azimuth % PANDAR_AT128_FRAME_ANGLE_SIZE + PANDAR_AT128_FRAME_ANGLE_SIZE * 2) / m_iAngleSize * m_iLaserNum + laserid;
          }        
        }
      }
    break;
  }
	return point_index;
}

int Convert::calculatePointBufferSize(){
  switch(m_PandarAT_corrections.header.frame_number){
    case 4: // two mirror case
      return (m_iViewMode == 0 ? PANDAR_AT128_FRAME_ANGLE_SIZE * 2 : PANDAR_AT128_FRAME_ANGLE_SIZE) / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize;
    break;
    case 3: // three mirror case
      return (m_iViewMode == 0 ? PANDAR_AT128_FRAME_ANGLE_SIZE * 3 : PANDAR_AT128_FRAME_ANGLE_SIZE) / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize;
  }
}
void Convert::loadOffsetFile(std::string file) {
  laserOffset.setFilePath(file);
}

void Convert::processGps(const pandar_msgs::PandarGps::ConstPtr &gpsMsg) {
  hasGps = 1;
  struct tm t;
  t.tm_sec = gpsMsg->second;
  t.tm_min = gpsMsg->minute;
  t.tm_hour = gpsMsg->hour;
  t.tm_mday = gpsMsg->day;
  t.tm_mon = gpsMsg->month - 1;
  t.tm_year = gpsMsg->year + 2000 - 1900;
  t.tm_isdst = 0;
  if (lastGPSSecond != (mktime(&t) + 1)) {
    lastGPSSecond = (mktime(&t) + 1);
    gps2.gps = mktime(&t) + 1;  // the gps always is the last gps, the newest
                                // GPS data is after the PPS(Serial port
                                // transmition speed...)
    gps2.used = 0;
  }
  // ROS_ERROR("Got data second : %f " ,(double)gps2.gps);
}
void Convert::SetEnvironmentVariableTZ(){
  char *TZ; 
  if((TZ = getenv("TZ"))){
    printf("TZ=%s\n",TZ); 
    return;
  } 
  unsigned int timezone = 0;
  time_t t1, t2 ;
  struct tm *tm_local, *tm_utc;
  time(&t1);
  t2 = t1;
  tm_local = localtime(&t1);
  t1 = mktime(tm_local) ;
  tm_utc = gmtime(&t2);
  t2 = mktime(tm_utc);
  timezone = 0;
  std::string data = "TZ=UTC" + std::to_string(timezone);
  int len = data.length();
  TZ = (char *)malloc((len + 1) * sizeof(char));
  data.copy(TZ, len, 0); 
  if(putenv(TZ) == 0){
    printf("set environment %s\n", TZ);
  }
  else{
    printf("set environment fail\n");
  }
}

}  // namespace pandar_pointcloud
