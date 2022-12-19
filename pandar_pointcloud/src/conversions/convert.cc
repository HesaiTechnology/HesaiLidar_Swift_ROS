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
// #define FIRETIME_CORRECTION_CHECK 
// #define COORDINATE_CORRECTION_CHECK

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
  
  m_sRosVersion = "PandarSwiftROS_1.0.37";
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
  private_nh.getParam("firetime_file", m_sLidarFiretimeFile);
  private_nh.getParam("calibration", m_sLidarCorrectionFile);
  private_nh.getParam("pcap", m_sPcapFile);

  std::string cert;
  std::string privateKey;
  std::string ca;
  private_nh.getParam("cert_file", cert);
  private_nh.getParam("private_key_file", privateKey);
  private_nh.getParam("ca_file", ca);
  private_nh.getParam("coordinate_correction_flag", m_bCoordinateCorrectionFlag);
  private_nh.getParam("channel_config_file", m_sLidarChannelConfigFile);
  TcpCommandSetSsl(cert.c_str(), privateKey.c_str(), ca.c_str());
  
  ROS_WARN("frame_id [%s]", m_sFrameId.c_str());
  ROS_WARN("lidarFiretimeFile [%s]", m_sLidarFiretimeFile.c_str());
  ROS_WARN("lidarCorrectionFile [%s]", m_sLidarCorrectionFile.c_str());

  m_iWorkMode = 0;
  m_iReturnMode = 0;
  m_iMotorSpeed = 0;
  m_iLaserNum = 0;
  m_iBlockNum = 0;
  m_u8UdpVersionMajor = 0;
  m_u8UdpVersionMinor = 0;
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

  m_iTimeZoneSecond = 0 * 3600;  // time zone
  start_angle_ = 0;

  for (int i = 0; i < PANDAR128_LASER_NUM; i++) {
    m_fElevAngle[i] = elev_angle[i];
    m_fHorizatalAzimuth[i] = azimuth_offset[i];
  }

  memset(m_fCosAllAngle, 0, sizeof(m_fCosAllAngle));
  memset(m_fSinAllAngle, 0, sizeof(m_fSinAllAngle));

  for (int j = 0; j < CIRCLE; j++) {
    float angle = static_cast<float>(j) / 100.0f;
    m_fCosAllAngle[j] = cosf(degreeToRadian(angle));
    m_fSinAllAngle[j] = sinf(degreeToRadian(angle));
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
        correntionString = std::string(buffer);
        ret = loadCorrectionFile(correntionString);
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
    std::ifstream fin(m_sLidarCorrectionFile);
    if (fin.is_open()) {
      ROS_WARN("Open correction file success");
      int length = 0;
      std::string strlidarCalibration;
      fin.seekg(0, std::ios::end);
      length = fin.tellg();
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      strlidarCalibration = buffer;
      ret = loadCorrectionFile(strlidarCalibration);
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
      m_sLidarFiretimeFile);  // parameter is the path of lidarFiretimeFil
  ROS_WARN("node_type[%s]", node_type.c_str());

  SetEnvironmentVariableTZ();

  if (LIDAR_NODE_TYPE == node_type) {
    boost::thread thrd(boost::bind(&Convert::DriverReadThread, this));
  }

  if (publishmodel == "both_point_raw" || publishmodel == "points" ||
      LIDAR_NODE_TYPE != node_type) {
    ROS_WARN("node.advertise pandar_points");
    output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10000);
    boost::thread processThr(boost::bind(&Convert::processLiDARData, this));
  }

  m_iPublishPointsIndex = 0;
  m_bPublishPointsFlag = false;
  boost::thread publishPointsThr(
      boost::bind(&Convert::publishPointsThread, this));

  if ((publishmodel == "both_point_raw" || publishmodel == "raw") &&
      LIDAR_NODE_TYPE == node_type) {
    boost::thread processThr(boost::bind(&Convert::publishRawDataThread, this));
  }
}

int Convert::loadCorrectionFile(std::string correction_content) {
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    ROS_WARN("Parse Lidar Correction...");
  }

  float pitchList[PANDAR128_LASER_NUM];
  float azimuthList[PANDAR128_LASER_NUM];
  int lineCounter = 0;
  std::vector<std::string>  firstLine;
  boost::split(firstLine, line, boost::is_any_of(","));
  if(firstLine[0] == "EEFF" || firstLine[0] == "eeff"){
    std::getline(ifs, line);
    for (int i = 0; i < PANDAR128_LASER_NUM; i++) {
      std::getline(ifs, line);
      if (line.length() < strlen("1,1,1")) {
        return -1;
      } else {
        lineCounter++;
      }

      float elev, azimuth;
      int lineId = 0;
      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;

      if (lineId != lineCounter) {
        ROS_WARN("laser id error %d %d", lineId, lineCounter);
        return -1;
      }
      pitchList[lineId - 1] = elev;
      azimuthList[lineId - 1] = azimuth;
    }
  }
  else{
    while (std::getline(ifs, line)) {
      if (line.length() < strlen("1,1,1")) {
        return -1;
      } else {
        lineCounter++;
      }

      float elev, azimuth;
      int lineId = 0;

      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;

      if (lineId != lineCounter) {
        ROS_WARN("laser id error %d %d", lineId, lineCounter);
        return -1;
      }

      pitchList[lineId - 1] = elev;
      azimuthList[lineId - 1] = azimuth;
    }
  }
  for (int i = 0; i < lineCounter; ++i) {
    m_fElevAngle[i] = pitchList[i];
    m_fHorizatalAzimuth[i] = azimuthList[i];
  }

  return 0;
}

int Convert::loadChannelConfigFile(std::string channel_config_content){
  m_PandarQTChannelConfig.m_bIsChannelConfigObtained = false;
  std::istringstream ifs(channel_config_content);
  std::string line;
  
  std::getline(ifs, line);
  std::vector<std::string>  versionLine;
  boost::split(versionLine, line, boost::is_any_of(","));
  if(versionLine[0] == "EEFF" || versionLine[0] == "eeff"){
      m_PandarQTChannelConfig.m_u8MajorVersion = std::stoi(versionLine[1].c_str());
      m_PandarQTChannelConfig.m_u8MinVersion = std::stoi(versionLine[2].c_str());
  }
  else{
      std::cout << "channel config file delimiter is wrong" << versionLine[0] << std::endl;
      return -1;
  }
  std::getline(ifs, line);
  std::vector<std::string>  channelNumLine;
  boost::split(channelNumLine, line, boost::is_any_of(","));
  m_PandarQTChannelConfig.m_u8LaserNum = std::stoi(channelNumLine[1].c_str());
  m_PandarQTChannelConfig.m_u8BlockNum = std::stoi(channelNumLine[3].c_str());

  std::getline(ifs, line);
  std::vector<std::string> firstChannelLine;
  boost::split(firstChannelLine, line, boost::is_any_of(","));
  int loop_num = firstChannelLine.size();
  m_PandarQTChannelConfig.m_vChannelConfigTable.resize(loop_num);

  for(int i = 0; i < loop_num; i++){
      m_PandarQTChannelConfig.m_vChannelConfigTable[i].resize(m_PandarQTChannelConfig.m_u8LaserNum);
  }
  for(int i = 0; i < m_PandarQTChannelConfig.m_u8LaserNum; i++){
    std::getline(ifs, line);
    std::vector<std::string> ChannelLine;
    boost::split(ChannelLine, line, boost::is_any_of(","));
    if (ChannelLine.size() != loop_num) {
      std::cout << "channel config file format is wrong" << std::endl;
      return -1;
    }
    for(int j = 0; j < loop_num; j++){
      if(ChannelLine.size() == loop_num){
        m_PandarQTChannelConfig.m_vChannelConfigTable[j][i] = std::stoi(ChannelLine[j].c_str());
        // printf("%d  %d  \n",i, m_PandarQTChannelConfig.m_vChannelConfigTable[j][i]);
      }
      else{
        std::cout << "loop num is not equal to the first channel line" << std::endl;
        return -1;
      }
        
    }
  }
  std::getline(ifs, line);
  m_PandarQTChannelConfig.m_sHashValue = line;
  m_PandarQTChannelConfig.m_bIsChannelConfigObtained = true;
  return 0;
}

void Convert::loadChannelConfigFile(){
  bool loadChannelConfigFileSuccess = false;
  int ret;
  if(m_sPcapFile.empty()) {
    if(NULL != m_pTcpCommandClient) {
      char *buffer = NULL;
      uint32_t len = 0;
      std::string correntionString;
      ret = TcpCommandGetLidarChannelConfig(m_pTcpCommandClient, &buffer, &len);
      if (ret == 0 && buffer) {
        ROS_WARN("Load channel config file from lidar now!");
        correntionString = std::string(buffer);
        ret = loadChannelConfigFile(correntionString);
          if (ret != 0) {
            ROS_WARN("Parse lidar channel config Error");
          } 
          else {
            loadChannelConfigFileSuccess = true;
            ROS_WARN("Parse lidar channel config success!!!");
          }
        free(buffer);
      }
      else{
        ROS_WARN("Get lidar calibration filed");
      }
    }
  }
  if(!loadChannelConfigFileSuccess) {
    ROS_WARN("load channel config file from local channelConfig.csv now!");
    std::ifstream fin(m_sLidarChannelConfigFile);
    if (fin.is_open()) {
      ROS_WARN("Open channel config file success");
      int length = 0;
      std::string strlidarCalibration;
      fin.seekg(0, std::ios::end);
      length = fin.tellg();
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      strlidarCalibration = buffer;
      ret = loadChannelConfigFile(strlidarCalibration);
      if (ret != 0) {
        ROS_WARN("Parse local channel config file Error");
      } 
      else {
        ROS_WARN("Parse local channel config file Success!!!");
      }
    }
    else{
      ROS_WARN("Open channel config file failed");
    }
  }

}



void Convert::loadFireTimeFile(){
  bool loadFireTimeFileSuccess = false;
  int ret;
  if(m_sPcapFile.empty()) {
    if(NULL != m_pTcpCommandClient) {
      char *buffer = NULL;
      uint32_t len = 0;
      std::string correntionString;
      ret = TcpCommandGetLidarFiretime(m_pTcpCommandClient, &buffer, &len);
      if (ret == 0 && buffer) {
        ROS_WARN("Load firetime file from lidar now!");
        correntionString = std::string(buffer);
        ret = m_objLaserOffset.ParserFiretimeData(correntionString);
          if (ret != 0) {
            ROS_WARN("Parse lidar firetime error");
          } 
          else {
            loadFireTimeFileSuccess = true;
            ROS_WARN("Parse lidar firetime success!!!");
          }
        free(buffer);
      }
      else{
        ROS_WARN("Get lidar firetime filed");
      }
    }
  }
  if(!loadFireTimeFileSuccess) {
    ROS_WARN("load firetime file from local firetime.csv now!");
    std::ifstream fin(m_sLidarFiretimeFile);
    if (fin.is_open()) {
      ROS_WARN("Open firetime file success");
      int length = 0;
      std::string strlidarCalibration;
      fin.seekg(0, std::ios::end);
      length = fin.tellg();
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      strlidarCalibration = buffer;
      ret = m_objLaserOffset.ParserFiretimeData(strlidarCalibration);
      if (ret != 0) {
        ROS_WARN("Parse local firetime file Error");
      } 
      else {
        ROS_WARN("Parse local firetime file Success!!!");
      }
    }
    else{
      ROS_WARN("Open firetime file failed\n");
    }
  }

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

int Convert::parseData(Pandar128PacketVersion13 &packet, const uint8_t *recvbuf,
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
      m_OutMsgArray[cursor]->resize(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize);
      m_PacketsBuffer.creatNewTask();
      pktCount = 0;
      continue;
    }
    checkClockwise();
    if(isNeedPublish()) {   // Judging whether pass the  start angle
			// uint32_t startTick1 = GetTickCount();
			moveTaskEndToStartAngle();
			doTaskFlow(cursor);
			// uint32_t startTick2 = GetTickCount();
			// printf("move and taskflow time:%d\n", startTick2 - startTick1);
      if(m_bPublishPointsFlag == false) {
				m_bPublishPointsFlag = true;
				m_iPublishPointsIndex = cursor;
				cursor = (cursor + 1) % 2;
#ifdef PRINT_FLAG
        ROS_WARN("ts %lf cld size %u", m_dTimestamp, m_OutMsgArray[m_iPublishPointsIndex]->points.size());
#endif  
			} 
			else
				ROS_WARN("publishPoints not done yet, new publish is comming\n");
        
			m_OutMsgArray[cursor]->clear();
			m_OutMsgArray[cursor]->resize(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize );
      if(m_RedundantPointBuffer.size() > 0 && m_RedundantPointBuffer.size() < MAX_REDUNDANT_POINT_NUM){
        for(int i = 0; i < m_RedundantPointBuffer.size(); i++){
          m_OutMsgArray[cursor]->points[m_RedundantPointBuffer[i].index] = m_RedundantPointBuffer[i].point;
        }
      }
      m_RedundantPointBuffer.clear();
			// uint32_t endTick2 = GetTickCount();
			// if(endTick2 - startTick2 > 2) {
				// printf("m_OutMsgArray time:%d\n", endTick2 - startTick2);
			// }
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
	// uint32_t startTick = GetTickCount();
  if(m_bClockwise == true){
		for(PktArray::iterator iter = m_PacketsBuffer.m_iterTaskBegin; iter < m_PacketsBuffer.m_iterTaskEnd; iter++) {
			if ((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) > *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) &&
				(m_iLidarRotationStartAngle <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) ||
				((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) < m_iLidarRotationStartAngle) &&
				(m_iLidarRotationStartAngle <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)))) {
				// printf("move iter to : %d\n", (iter + 1)->blocks[0].fAzimuth);
				m_PacketsBuffer.moveTaskEnd(iter + 1);
				break;
			}
		}
	}
	else{
		for(PktArray::iterator iter = m_PacketsBuffer.m_iterTaskBegin; iter < m_PacketsBuffer.m_iterTaskEnd; iter++) {
			if ((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) < *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) &&
				((((CIRCLE_ANGLE - m_iLidarRotationStartAngle > CIRCLE_ANGLE / 2)) &&(m_iLidarRotationStartAngle <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex))) ||
				(((CIRCLE_ANGLE - m_iLidarRotationStartAngle < CIRCLE_ANGLE / 2)) &&(m_iLidarRotationStartAngle >= *(uint16_t*)(&((iter)->data[0]) + m_iFirstAzimuthIndex)))) ||
				((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) > m_iLidarRotationStartAngle) &&
				(m_iLidarRotationStartAngle >= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)))) {
				// printf("move iter to : %d\n", (iter + 1)->blocks[0].fAzimuth);
				m_PacketsBuffer.moveTaskEnd(iter + 1);
				break;
			}
		}
	}
	// uint32_t endTick = GetTickCount();
	// printf("moveTaskEndToStartAngle time: %d\n", endTick - startTick);
}

void Convert::init() {
	while (1) {
		if(!m_PacketsBuffer.hasEnoughPackets()) {
			usleep(1000);
			continue;
		}
		uint16_t lidarmotorspeed = 0;
    m_u8UdpVersionMajor = (m_PacketsBuffer.getTaskEnd() - 1)->data[2];
		m_u8UdpVersionMinor = (m_PacketsBuffer.getTaskEnd() - 1)->data[3];
    ROS_WARN("UDP Version is:%d.%d", m_u8UdpVersionMajor, m_u8UdpVersionMinor);
    switch (m_u8UdpVersionMajor){
      case 1:
        switch (m_u8UdpVersionMinor)
        {
          case 3:
          {
            Pandar128PacketVersion13 packet;
            memcpy(&packet, &((m_PacketsBuffer.getTaskEnd() - 1)->data[0]), sizeof(Pandar128PacketVersion13));
            m_iWorkMode = packet.tail.nShutdownFlag & 0x03;
            m_iReturnMode = packet.tail.nReturnMode;
            drv.setUdpVersion(m_u8UdpVersionMajor, m_u8UdpVersionMinor);
            lidarmotorspeed = packet.tail.nMotorSpeed;
            m_iLaserNum = packet.head.u8LaserNum;
            m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
            m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + PANDAR128_BLOCK_SIZE;
          }
          break;
          case 4:
          {
            auto header = (Pandar128HeadVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
            auto tail = (Pandar128TailVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR128_HEAD_SIZE +
                  header->unitSize() * header->u8LaserNum * header->u8BlockNum + 
                  PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
                  PANDAR128_CRC_SIZE + 
                  (header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0));
            m_iWorkMode = tail->getOperationMode();
            m_iReturnMode = tail->nReturnMode;
            drv.setUdpVersion(m_u8UdpVersionMajor, m_u8UdpVersionMinor);
            lidarmotorspeed = tail->nMotorSpeed;
            m_iLaserNum = header->u8LaserNum;
            m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
            m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + 
                        (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
                        PANDAR128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
          }
          break;
          default:
          break;
        }
      case 3:
        switch (m_u8UdpVersionMinor)
        {
          case 2:
          {
            auto header = (PandarQT128Head*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
            auto tail = (PandarQT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR128_HEAD_SIZE +
                  (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) + 
                  PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
                  PANDAR128_CRC_SIZE + 
                  (header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0));
            m_iWorkMode = tail->nShutdownFlag & 0x03;
            m_iReturnMode = tail->nReturnMode;
            drv.setUdpVersion(m_u8UdpVersionMajor, m_u8UdpVersionMinor);
            lidarmotorspeed = tail->nMotorSpeed;
            m_iLaserNum = header->u8LaserNum;
            m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
            m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + 
                        (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
                        PANDAR128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
            m_PacketsBuffer.m_stepSize = PANDARQT128_TASKFLOW_STEP_SIZE; 
            if(header->isSelfDefine()){
              loadChannelConfigFile();
            }  
            loadFireTimeFile();          
          }
          break;
          default:
          break;
        }
        default:
        break;
      }
      if(abs(lidarmotorspeed - MOTOR_SPEED_600) < 100) { //ignore the speed gap of 6000 rpm
        lidarmotorspeed = MOTOR_SPEED_600;
      }
      else if(abs(lidarmotorspeed - MOTOR_SPEED_1200) < 100) { //ignore the speed gap of 1200 rpm
        lidarmotorspeed = MOTOR_SPEED_1200;
      }
      else {
        lidarmotorspeed = MOTOR_SPEED_600; //changing the speed,give enough size
      }
      m_iMotorSpeed = lidarmotorspeed;
      printf("init mode: workermode: %x,return mode: %x,speed: %d\n",m_iWorkMode, m_iReturnMode, m_iMotorSpeed);
      changeAngleSize();
      changeReturnBlockSize();
      checkClockwise();
      boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
      boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
      m_OutMsgArray[0] = outMag0;
      m_OutMsgArray[1] = outMag1;
      break;
    }
}

void Convert::publishPoints() {
  // uint32_t start = GetTickCount();

  pcl_conversions::toPCL(ros::Time(m_dTimestamp),
                         m_OutMsgArray[m_iPublishPointsIndex]->header.stamp);
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*m_OutMsgArray[m_iPublishPointsIndex], output);
  output_.publish(output);
  m_bPublishPointsFlag = false;
  m_dTimestamp = 0;

  // uint32_t end = GetTickCount();
  // if (end - start > 150) ROS_WARN("publishPoints time:%d", end - start);
}

void Convert::publishPointsThread() {
  sched_param param;
  param.sched_priority = 90;
  // SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  ROS_WARN("publishPointsThread:set result [%d]", rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("publishPointsThread:get thead %lu, policy %d and priority %d\n",
           pthread_self(), ret_policy, param.sched_priority);

  while (1) {
    usleep(100);
    if (m_bPublishPointsFlag) publishPoints();
  }
}

void Convert::checkClockwise(){
  uint16_t frontAzimuth = *(uint16_t*)(&(m_PacketsBuffer.m_iterTaskBegin->data[0]) + m_iFirstAzimuthIndex);
  uint16_t backAzimuth = *(uint16_t*)(&((m_PacketsBuffer.m_iterTaskBegin + 1)->data[0]) + m_iFirstAzimuthIndex);
  if(((frontAzimuth < backAzimuth) && ((backAzimuth - frontAzimuth) <  m_iAngleSize * 10)) 
     ||
    ((frontAzimuth > backAzimuth) && (frontAzimuth - backAzimuth) > m_iAngleSize * 10))
  {
    m_bClockwise = true; //Clockwise
    return;
  }

  if(((frontAzimuth > backAzimuth) && ((frontAzimuth - backAzimuth) <  m_iAngleSize * 10))
     ||
    ((frontAzimuth < backAzimuth) && (backAzimuth - frontAzimuth) > m_iAngleSize * 10))
  {
    m_bClockwise = false; //countClockwise
    return;
  }

}

void Convert::doTaskFlow(int cursor) {
  tf::Taskflow taskFlow;
  switch (m_u8UdpVersionMajor)
  {
    case 1:
    {
      taskFlow.parallel_for(m_PacketsBuffer.getTaskBegin(),
                            m_PacketsBuffer.getTaskEnd(),
                            [this, &cursor](auto &taskpkt) {
                              calcPointXYZIT(taskpkt,cursor);
                            });
    }
    break;
    case 3:
    {
      taskFlow.parallel_for(m_PacketsBuffer.getTaskBegin(),
                            m_PacketsBuffer.getTaskEnd(),
                            [this, &cursor](auto &taskpkt) {
                              calcQT128PointXYZIT(taskpkt,cursor);
                            });
    }
    break;
    default: 
    break;             
  }
  executor.run(taskFlow).wait();
  m_PacketsBuffer.creatNewTask();

}

int Convert::checkLiadaMode() {
  uint8_t lidarworkmode = 0;
	uint8_t lidarreturnmode = 0;
	uint16_t lidarmotorspeed = 0;
	uint8_t laserNum = 0;
  uint8_t blockNum = 0;
  switch (m_u8UdpVersionMajor)
  {
    case 1:
      switch (m_u8UdpVersionMinor)
      {
        case 3:
        {
          Pandar128PacketVersion13 packet;
          memcpy(&packet, &((m_PacketsBuffer.getTaskEnd() - 1)->data[0]), sizeof(Pandar128PacketVersion13));
          lidarworkmode = packet.tail.nShutdownFlag & 0x03;
          lidarreturnmode = packet.tail.nReturnMode;
          lidarmotorspeed = packet.tail.nMotorSpeed;
          laserNum = packet.head.u8LaserNum;
        }
        break;
        case 4:
        {
          auto header = (Pandar128HeadVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
          auto tail = (Pandar128TailVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR128_HEAD_SIZE +
                  header->unitSize() * header->u8LaserNum * header->u8BlockNum + 
                  PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
                  PANDAR128_CRC_SIZE + 
                  (header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0));
                lidarworkmode = tail->getOperationMode();
                lidarreturnmode = tail->nReturnMode;
                lidarmotorspeed = tail->nMotorSpeed;
                laserNum = header->u8LaserNum;
                blockNum = header->u8BlockNum;
                m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
                m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + 
                                      (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
                                      PANDAR128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
        }
        break;
        default:
        break;
      }
    case 3:
      switch (m_u8UdpVersionMinor)
      {
        case 2:
        {
          auto header = (PandarQT128Head*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
          auto tail = (PandarQT128Tail*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR128_HEAD_SIZE +
                  (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) + 
                  PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
                  PANDAR128_CRC_SIZE + 
                  (header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0));
                lidarworkmode = tail->nShutdownFlag & 0x03;
                lidarreturnmode = tail->nReturnMode;
                lidarmotorspeed = tail->nMotorSpeed;
                laserNum = header->u8LaserNum;
                blockNum = header->u8BlockNum;
                m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
                m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + 
                                      (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
                                      PANDAR128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
          if(header->isSelfDefine() && header->u8LaserNum != m_PandarQTChannelConfig.m_u8LaserNum && m_sPcapFile.empty()){
            ROS_WARN("Laser num is changed");
            loadChannelConfigFile();
          }                             
        }
        break;
        default:
        break;
      }
    default:
    break;  
  }
  if(abs(lidarmotorspeed - MOTOR_SPEED_600) < 100) { //ignore the speed gap of 6000 rpm
    lidarmotorspeed = MOTOR_SPEED_600;
  }
  else if(abs(lidarmotorspeed - MOTOR_SPEED_1200) < 100) { //ignore the speed gap of 1200 rpm
    lidarmotorspeed = MOTOR_SPEED_1200;
  }
  else {
      lidarmotorspeed = MOTOR_SPEED_600; //changing the speed,give enough size
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
    boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(
        CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
    boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(
        CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
    m_OutMsgArray[0] = outMag0;
    m_OutMsgArray[1] = outMag1;
    return 1;
  } 
  else { //mode change 
    if (m_iWorkMode != lidarworkmode) { //work mode change
      ROS_WARN("change work mode:  %x to %x ",m_iWorkMode, lidarworkmode);
      m_iWorkMode = lidarworkmode;
      m_iMotorSpeed = lidarmotorspeed;
      changeAngleSize();
      return 0;
    }
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
  switch (m_u8UdpVersionMajor)
  {
    case 1:
    {
      if (m_iLaserNum == PANDAR80_LASER_NUM) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_18;  // 18->0.18degree
        return;
      } 
      if (m_iLaserNum == PANDAR64S_LASER_NUM || m_iLaserNum == PANDAR40S_LASER_NUM) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_20 * m_iMotorSpeed / MOTOR_SPEED_600;  // 20->0.2degree
        return;
      } 
      if (0 == m_iWorkMode && MOTOR_SPEED_600 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_10;  // 10->0.1degree
      }
      if (0 == m_iWorkMode && MOTOR_SPEED_1200 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_20;  // 20->0.2degreepktCount[2]
      }
      if (0 != m_iWorkMode && MOTOR_SPEED_600 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_20;  // 20->0.2degree
      }
      if (0 != m_iWorkMode && MOTOR_SPEED_1200 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_40;  // 40->0.4degree
      }
    }
    break;
    case 3:
    {
      if (MOTOR_SPEED_600 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_40;  // 20->0.2degree
      }
      if (MOTOR_SPEED_1200 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_80;  // 40->0.4degree
      }
    }
    break;
    default:
    break;
  }
}

void Convert::changeReturnBlockSize() {
  if (0x39 == m_iReturnMode || 0x3b == m_iReturnMode || 0x3c == m_iReturnMode || 0x3a == m_iReturnMode || 0x3e == m_iReturnMode) {
    m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_2;
  } else {
    m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_1;
  }
}
void Convert::calcPointXYZIT(pandar_msgs::PandarPacket &packet, int cursor) {
  if (packet.data[3] == 3){
		Pandar128PacketVersion13 pkt;
		memcpy(&pkt, &packet.data[0], sizeof(Pandar128PacketVersion13));
		struct tm t = {0};
		t.tm_year = pkt.tail.nUTCTime[0];
		t.tm_mon = pkt.tail.nUTCTime[1] - 1;
		t.tm_mday = pkt.tail.nUTCTime[2];
		t.tm_hour = pkt.tail.nUTCTime[3];
		t.tm_min = pkt.tail.nUTCTime[4];
		t.tm_sec = pkt.tail.nUTCTime[5];
		t.tm_isdst = 0;
		double unix_second = static_cast<double>(mktime(&t) + m_iTimeZoneSecond);
		for (int blockid = 0; blockid < pkt.head.u8BlockNum; blockid++) {
			Pandar128Block &block = pkt.blocks[blockid];
			int mode = pkt.tail.nShutdownFlag & 0x03;
			int state = 0;
			if(0 == blockid)
				state = (pkt.tail.nShutdownFlag & 0xC0) >> 6;
			if(1 == blockid)
				state = (pkt.tail.nShutdownFlag & 0x30) >> 4;
			for (int i = 0; i < pkt.head.u8LaserNum; i++) {
				/* for all the units in a block */
				Pandar128Unit &unit = block.units[i];
				PPoint point;
				float distance =static_cast<float>(unit.u16Distance) * PANDAR128_DISTANCE_UNIT;
				/* filter distance */
				// if(distance < 0.1) {
				// 	continue;
				// }
				float azimuth = m_fHorizatalAzimuth[i] + (block.fAzimuth / 100.0f);
				float originAzimuth = azimuth;
				float pitch = m_fElevAngle[i];
				float offset = m_bClockwise ? m_objLaserOffset.getTSOffset(i, mode, state, distance, m_u8UdpVersionMajor) : -m_objLaserOffset.getTSOffset(i, mode, state, distance, m_u8UdpVersionMajor);
				azimuth += m_objLaserOffset.getAngleOffset(offset, pkt.tail.nMotorSpeed, m_u8UdpVersionMajor);
#ifdef FIRETIME_CORRECTION_CHECK 
        ROS_WARN("Laser ID = %d, speed = %d, origin azimuth = %f, azimuth = %f, delt = %f", i + 1, pkt.tail.nMotorSpeed, originAzimuth, azimuth, azimuth - originAzimuth);  
#endif     
        if(m_bCoordinateCorrectionFlag){
          pitch += m_objLaserOffset.getPitchOffset(m_sFrameId, pitch, distance);
        }
				int pitchIdx = static_cast<int>(pitch * 100 + 0.5);
        if (pitchIdx  >= CIRCLE) {
          pitchIdx  -= CIRCLE;
        } else if (pitchIdx  < 0) {
          pitchIdx  += CIRCLE;
        }
				float xyDistance = distance * m_fCosAllAngle[pitchIdx];
        if(m_bCoordinateCorrectionFlag){
          azimuth += m_objLaserOffset.getAzimuthOffset(m_sFrameId, originAzimuth, block.fAzimuth / 100.0f, xyDistance);
          
        }
				int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
				if(azimuthIdx >= CIRCLE) {
					azimuthIdx -= CIRCLE;
				} 
				else if(azimuthIdx < 0) {
					azimuthIdx += CIRCLE;
				}
				point.x = xyDistance * m_fSinAllAngle[azimuthIdx];
				point.y = xyDistance * m_fCosAllAngle[azimuthIdx];
				point.z = distance * m_fSinAllAngle[pitchIdx];
				point.intensity = unit.u8Intensity;
				point.timestamp = unix_second + (static_cast<double>(pkt.tail.nTimestamp)) / 1000000.0;
				point.timestamp = point.timestamp + m_objLaserOffset.getBlockTS(blockid, pkt.tail.nReturnMode, mode, pkt.head.u8LaserNum) / 1000000000.0 + offset / 1000000000.0;
				if(0 == m_dTimestamp) {
					m_dTimestamp = point.timestamp;
				}
				else if(m_dTimestamp > point.timestamp) {
					m_dTimestamp = point.timestamp;
				}
				point.ring = i + 1;
				int point_index;
				if(LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
					point_index = (block.fAzimuth - start_angle_) / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize + m_iLaserNum * blockid + i;
					// printf("block 2 index:[%d]",index);
				} 
				else {
					point_index = (block.fAzimuth - start_angle_) / m_iAngleSize * m_iLaserNum + i;
				}
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
	else if(packet.data[3] == 4){
    auto header = (Pandar128HeadVersion14*)(&packet.data[0]);
    auto tail = (Pandar128TailVersion14*)(&packet.data[0] + PANDAR128_HEAD_SIZE + 
                header->unitSize() * header->u8LaserNum * header->u8BlockNum + 
                PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
                PANDAR128_CRC_SIZE + 
                (header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0));
    struct tm t = {0};

    t.tm_year = tail->nUTCTime[0];
    t.tm_mon = tail->nUTCTime[1] - 1;
    t.tm_mday = tail->nUTCTime[2];
    t.tm_hour = tail->nUTCTime[3];
    t.tm_min = tail->nUTCTime[4];
    t.tm_sec = tail->nUTCTime[5];
    t.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&t) + m_iTimeZoneSecond);
    // ROS_WARN("#####block.fAzimuth[%u][%u]",pkt.blocks[0].fAzimuth,pkt.blocks[1].fAzimuth);
    int index = 0;
    index += PANDAR128_HEAD_SIZE;
    for (int blockid = 0; blockid < header->u8BlockNum; blockid++) {
      uint16_t u16Azimuth = *(uint16_t*)(&packet.data[0] + index);
      // ROS_WARN("#####block.fAzimuth[%u]",u16Azimuth);
      index += PANDAR128_AZIMUTH_SIZE;

      int mode = tail->getOperationMode();
      int state = tail->getAngleState(blockid);
      for (int i = 0; i < header->u8LaserNum; i++) {
        /* for all the units in a block */
        uint16_t u16Distance = *(uint16_t*)(&packet.data[0] + index);
        index += DISTANCE_SIZE;
        uint8_t u8Intensity = *(uint8_t*)(&packet.data[0] + index);
        index += INTENSITY_SIZE;
        index += header->hasConfidence() ? CONFIDENCE_SIZE : 0;
        index += header->hasWeightFactor() ? WEIGHT_FACTOR_SIZE : 0;
				index += header->hasEnvLight() ? ENVLIGHT_SIZE : 0;
        PPoint point;

        float distance =
            static_cast<float>(u16Distance) * PANDAR128_DISTANCE_UNIT;

        float azimuth = m_fHorizatalAzimuth[i] + (u16Azimuth / 100.0f);
        float originAzimuth = azimuth;
        float pitch = m_fElevAngle[i];
        float offset = m_bClockwise ? m_objLaserOffset.getTSOffset(i, mode, state, distance, m_u8UdpVersionMajor) : -m_objLaserOffset.getTSOffset(i, mode, state, distance, m_u8UdpVersionMajor);

        azimuth += m_objLaserOffset.getAngleOffset(offset, tail->nMotorSpeed, m_u8UdpVersionMajor);
#ifdef FIRETIME_CORRECTION_CHECK 
        ROS_WARN("Laser ID = %d, speed = %d, origin azimuth = %f, azimuth = %f, delt = %f", i + 1, tail->nMotorSpeed, originAzimuth, azimuth, azimuth - originAzimuth);  
#endif  
        if(m_bCoordinateCorrectionFlag){
          pitch += m_objLaserOffset.getPitchOffset(m_sFrameId, pitch, distance);
        }
        int pitchIdx = static_cast<int>(pitch * 100 + 0.5);
        if (pitchIdx  >= CIRCLE) {
          pitchIdx  -= CIRCLE;
        } else if (pitchIdx  < 0) {
          pitchIdx  += CIRCLE;
        }

        float xyDistance =
            distance * m_fCosAllAngle[pitchIdx];
        if(m_bCoordinateCorrectionFlag){
          azimuth += m_objLaserOffset.getAzimuthOffset(m_sFrameId, originAzimuth, u16Azimuth / 100.0f, xyDistance);
        }

        int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
        if (azimuthIdx >= CIRCLE) {
          azimuthIdx -= CIRCLE;
        } else if (azimuthIdx < 0) {
          azimuthIdx += CIRCLE;
        }

        point.x = xyDistance * m_fSinAllAngle[azimuthIdx];
        point.y = xyDistance * m_fCosAllAngle[azimuthIdx];
        point.z = distance * m_fSinAllAngle[pitchIdx];

        point.intensity = u8Intensity;
        point.timestamp =
            unix_second + (static_cast<double>(tail->nTimestamp)) / 1000000.0;
            // ROS_WARN("#####block.fAzimuth[%u]",tail->nShutdownFlag);

        point.timestamp =
            point.timestamp +
            m_objLaserOffset.getBlockTS(blockid, tail->nReturnMode, mode, header->u8LaserNum) /
                1000000000.0 +
            offset / 1000000000.0;

        if (0 == m_dTimestamp) {
          m_dTimestamp = point.timestamp;
        } else if (m_dTimestamp > point.timestamp) {
          m_dTimestamp = point.timestamp;
        }

        point.ring = i + 1;

        int point_index;
        if (LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
          point_index = (u16Azimuth - start_angle_) / m_iAngleSize * header->u8LaserNum *
                      m_iReturnBlockSize +
                  m_iLaserNum * (blockid % 2) + i;
          // ROS_WARN("block 2 index:[%d]",index);
        } else {
          point_index = (u16Azimuth - start_angle_) / m_iAngleSize * m_iLaserNum + i;
        }
        if(m_OutMsgArray[cursor]->points[point_index].ring == 0){
          m_OutMsgArray[cursor]->points[point_index] = point;
        }
        else{
          pthread_mutex_lock(&m_RedundantPointLock);
          if (point.timestamp - m_OutMsgArray[cursor]->points[point_index].timestamp  > 0.01 && point.timestamp - m_OutMsgArray[cursor]->points[point_index].timestamp  < 0.11)
						m_RedundantPointBuffer.push_back(RedundantPoint{point_index, point});
          pthread_mutex_unlock(&m_RedundantPointLock);
        }
      }
    }
  }
}

void Convert::calcQT128PointXYZIT(pandar_msgs::PandarPacket &packet, int cursor) {
  auto header = (PandarQT128Head*)(&packet.data[0]);
  auto tail = (PandarQT128Tail*)(&packet.data[0] + PANDAR128_HEAD_SIZE + 
              (header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) + 
              PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
              PANDAR128_CRC_SIZE + 
              (header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0));
  if (packet.data[0] != 0xEE && packet.data[1] != 0xFF) {    
    return ;
  }
    // bool hasWeightFactor = header->hasWeightFactor();
  bool isSelfDefine = header->isSelfDefine();
  struct tm t = {0};

  t.tm_year = tail->nUTCTime[0];
  t.tm_mon = tail->nUTCTime[1] - 1;
  t.tm_mday = tail->nUTCTime[2];
  t.tm_hour = tail->nUTCTime[3];
  t.tm_min = tail->nUTCTime[4];
  t.tm_sec = tail->nUTCTime[5];
  t.tm_isdst = 0;

  double unix_second = static_cast<double>(mktime(&t) + m_iTimeZoneSecond);
  // ROS_WARN("#####block.fAzimuth[%u][%u]",pkt.blocks[0].fAzimuth,pkt.blocks[1].fAzimuth);
  int index = 0;
  index += PANDAR128_HEAD_SIZE;
  for (int blockid = 0; blockid < header->u8BlockNum; blockid++) {
    int loopIndex = (tail->nModeFlag + (blockid / ((tail->nReturnMode < 0x39) ? 1 : 2)) + 1) % header->u8BlockNum;
    if((isSelfDefine && m_PandarQTChannelConfig.m_bIsChannelConfigObtained)){
        loopIndex = (tail->nModeFlag + (blockid / ((tail->nReturnMode < 0x39) ? 1 : 2)) + 1) % m_PandarQTChannelConfig.m_vChannelConfigTable.size();
    }
    uint16_t u16Azimuth = *(uint16_t*)(&packet.data[0] + index);
    // ROS_WARN("#####block.fAzimuth[%u]",u16Azimuth);
    index += PANDAR128_AZIMUTH_SIZE;

    int mode = tail->nShutdownFlag & 0x03;
    int state = (tail->nShutdownFlag & 0xF0) >> 4;

    for (int i = 0; i < header->u8LaserNum; i++) {
      /* for all the units in a block */
      uint16_t u16Distance = *(uint16_t*)(&packet.data[0] + index);
      index += DISTANCE_SIZE;
      uint8_t u8Intensity = *(uint8_t*)(&packet.data[0] + index);
      index += INTENSITY_SIZE;
      index += header->hasConfidence() ? CONFIDENCE_SIZE : 0;
      PPoint point;

      float distance = static_cast<float>(u16Distance) * (header->u8DistUnit / 1000.0f);
      int laserId = (isSelfDefine && m_PandarQTChannelConfig.m_bIsChannelConfigObtained && i < m_PandarQTChannelConfig.m_vChannelConfigTable[loopIndex].size()) ? m_PandarQTChannelConfig.m_vChannelConfigTable[loopIndex][i] - 1 : i;
      float azimuth = m_fHorizatalAzimuth[laserId] + (u16Azimuth / 100.0f);
      float originAzimuth = azimuth;
      float pitch = m_fElevAngle[laserId];
      float offset = m_bClockwise ? m_objLaserOffset.getTSOffset(laserId, loopIndex, state, distance, m_u8UdpVersionMajor) : -m_objLaserOffset.getTSOffset(i, loopIndex, state, distance, m_u8UdpVersionMajor);
      azimuth += m_objLaserOffset.getAngleOffset(offset, tail->nMotorSpeed, m_u8UdpVersionMajor);
#ifdef FIRETIME_CORRECTION_CHECK 
        ROS_WARN("Laser ID = %d, speed = %d, correction mode = %d, block id = %d, origin azimuth = %f, azimuth = %f, delt = %f", laserId + 1, tail->nMotorSpeed, loopIndex, blockid, originAzimuth, azimuth, azimuth - originAzimuth);  
#endif  
      int pitchIdx = static_cast<int>(pitch * 100 + 0.5);
      if (pitchIdx  >= CIRCLE) {
        pitchIdx  -= CIRCLE;
      } else if (pitchIdx  < 0) {
        pitchIdx  += CIRCLE;
      }

      int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
      if (azimuthIdx >= CIRCLE) {
        azimuthIdx -= CIRCLE;
      } else if (azimuthIdx < 0) {
        azimuthIdx += CIRCLE;
      }
      if(m_bCoordinateCorrectionFlag && distance > 0.1){
				if (m_fSinAllAngle[pitchIdx] != 0){
					float c = (HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG +
							HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT * HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT - 
							distance * distance ) * 
							m_fSinAllAngle[pitchIdx] * m_fSinAllAngle[pitchIdx];
					float b = 2 * m_fSinAllAngle[pitchIdx] * m_fCosAllAngle[pitchIdx] * (HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * m_fCosAllAngle[azimuthIdx] - HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT * m_fSinAllAngle[azimuthIdx]);
					point.z = (- b + sqrt(b * b - 4 * c)) / 2;
					point.x = point.z * m_fSinAllAngle[azimuthIdx] * m_fCosAllAngle[pitchIdx] / m_fSinAllAngle[pitchIdx] - HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT;
					point.y = point.z * m_fCosAllAngle[azimuthIdx] * m_fCosAllAngle[pitchIdx] / m_fSinAllAngle[pitchIdx] + HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG;
					if(((point.x + HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT) * m_fCosAllAngle[pitchIdx] * m_fSinAllAngle[azimuthIdx] + 
					(point.y - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG) * m_fCosAllAngle[pitchIdx] * m_fCosAllAngle[azimuthIdx] + 
						point.z * m_fSinAllAngle[pitchIdx]) <= 0){
					point.z = (- b - sqrt(b * b - 4 * c)) / 2;
					point.x = point.z * m_fSinAllAngle[azimuthIdx] * m_fCosAllAngle[pitchIdx] / m_fSinAllAngle[pitchIdx] - HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT;
					point.y = point.z * m_fCosAllAngle[azimuthIdx] * m_fCosAllAngle[pitchIdx] / m_fSinAllAngle[pitchIdx] + HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG;
					}
				}
				else if (m_fCosAllAngle[azimuthIdx] != 0){
					float tan_azimuth = m_fSinAllAngle[azimuthIdx] / m_fCosAllAngle[azimuthIdx];
					float c = (HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * tan_azimuth + HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT) *
							(HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * tan_azimuth + HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT) - 
							distance * distance ;
					float a = 1 + tan_azimuth * tan_azimuth;
					float b = - 2 * tan_azimuth * (HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * tan_azimuth + HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT);
					point.z = 0;
					point.y = (- b + sqrt(b * b - 4 * a * c)) / (2 * a);
					point.x = (point.y - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG) * tan_azimuth - HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT;
					if(((point.x + HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT) * m_fCosAllAngle[pitchIdx] * m_fSinAllAngle[azimuthIdx] + 
					(point.y - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG) * m_fCosAllAngle[pitchIdx] * m_fCosAllAngle[azimuthIdx] + 
						point.z * m_fSinAllAngle[pitchIdx]) <= 0){
					point.z = 0;
					point.y = (- b - sqrt(b * b - 4 * a * c)) / (2 * a);
					point.x = (point.y - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG) * tan_azimuth - HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT;
					}
				}
				else {
					point.x = sqrt(distance * distance - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG);
					point.y = HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG;
					point.z = 0;
					if(((point.x + HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT) * m_fCosAllAngle[pitchIdx] * m_fSinAllAngle[azimuthIdx] + 
					(point.y - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG) * m_fCosAllAngle[pitchIdx] * m_fCosAllAngle[azimuthIdx] + 
						point.z * m_fSinAllAngle[pitchIdx]) <= 0){
					point.x = - sqrt(distance * distance - HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG * HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG);
					point.y = HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG;
					point.z = 0;
					}
				}
#ifdef COORDINATE_CORRECTION_CHECK
					float xyDistance = distance * m_fCosAllAngle[pitchIdx];
					float point_x = static_cast<float>(xyDistance * m_fSinAllAngle[azimuthIdx]); // without coordinate correction 
					float point_y = static_cast<float>(xyDistance * m_fCosAllAngle[azimuthIdx]);
					float point_z = static_cast<float>(distance * m_fSinAllAngle[pitchIdx]);
					ROS_WARN("distance = %f; elevation = %f; azimuth = %f; delta X = %f; delta Y = %f; delta Z = %f;", 
						distance , float(pitchIdx) / 100, float(azimuthIdx) / 100, point.x - point_x, point.y - point_y, point.z - point_z);
#endif

			}
			else{
				float xyDistance = distance * m_fCosAllAngle[pitchIdx];
				point.x = static_cast<float>(xyDistance * m_fSinAllAngle[azimuthIdx]); // without coordinate correction 
				point.y = static_cast<float>(xyDistance * m_fCosAllAngle[azimuthIdx]);
				point.z = static_cast<float>(distance * m_fSinAllAngle[pitchIdx]);
			}


      point.intensity = u8Intensity;
      point.timestamp =
            unix_second + (static_cast<double>(tail->nTimestamp)) / 1000000.0;
            // ROS_WARN("#####block.fAzimuth[%u]",tail->nShutdownFlag);
      // point.timestamp = unix_second + ((packet.data[1082]& 0xff) | \
      //     (packet.data[1082+1]& 0xff) << 8 | \
      //     ((packet.data[1082+2]& 0xff) << 16) | \
      //     ((packet.data[1082+3]& 0xff) << 24)) / 1000000.0;
          // ROS_WARN("#####block.fAzimuth[%u]",tail->nShutdownFlag);

      if (0 == m_dTimestamp) {
        m_dTimestamp = point.timestamp;
      } else if (m_dTimestamp > point.timestamp) {
        m_dTimestamp = point.timestamp;
      }

      point.ring = laserId + 1;

      int point_index;
      if (LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
        point_index = (u16Azimuth - start_angle_) / m_iAngleSize * header->u8LaserNum *
                    m_iReturnBlockSize +
                m_iLaserNum * (blockid % 2) + i;
        // ROS_WARN("block 2 index:[%d]",index);
      } else {
        point_index = (u16Azimuth - start_angle_) / m_iAngleSize * m_iLaserNum + i;
      }
      if(m_OutMsgArray[cursor]->points[point_index].ring == 0){
        m_OutMsgArray[cursor]->points[point_index] = point;
      }
      else{
        pthread_mutex_lock(&m_RedundantPointLock);
        if (point.timestamp - m_OutMsgArray[cursor]->points[point_index].timestamp  > 0.01 && point.timestamp - m_OutMsgArray[cursor]->points[point_index].timestamp  < 0.11)
						m_RedundantPointBuffer.push_back(RedundantPoint{point_index, point});
        pthread_mutex_unlock(&m_RedundantPointLock);
      }
    }
  }
}

void Convert::loadOffsetFile(std::string file) {
  m_objLaserOffset.setFilePath(file);
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
  timezone = t2 >= t1 ? (t2 - t1) / 3600 : (t1 - t2) / 3600;
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

bool Convert::isNeedPublish(){
  uint16_t beginAzimuth = *(uint16_t*)(&(m_PacketsBuffer.getTaskBegin()->data[0]) + m_iFirstAzimuthIndex);
  uint16_t endAzimuth = *(uint16_t*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + m_iLastAzimuthIndex);
  if(((m_bClockwise == true) &&
			((beginAzimuth > endAzimuth) && (((CIRCLE_ANGLE - m_iLidarRotationStartAngle > CIRCLE_ANGLE / 2)) && (m_iLidarRotationStartAngle <= endAzimuth) || 
      ((CIRCLE_ANGLE - m_iLidarRotationStartAngle < CIRCLE_ANGLE / 2)) && (m_iLidarRotationStartAngle > beginAzimuth)) ||
			((beginAzimuth < m_iLidarRotationStartAngle) && (m_iLidarRotationStartAngle <= endAzimuth) ||
      abs(endAzimuth - m_iLidarRotationStartAngle) <= m_iAngleSize ||
			abs(int(CIRCLE_ANGLE) - endAzimuth - m_iLidarRotationStartAngle) <= m_iAngleSize))) ||
			((m_bClockwise == false) &&
			((beginAzimuth < endAzimuth) && (((CIRCLE_ANGLE - m_iLidarRotationStartAngle > CIRCLE_ANGLE / 2)) && (m_iLidarRotationStartAngle < beginAzimuth) ||
			((CIRCLE_ANGLE - m_iLidarRotationStartAngle < CIRCLE_ANGLE / 2)) && (m_iLidarRotationStartAngle >= endAzimuth)) || 
			((beginAzimuth > m_iLidarRotationStartAngle) && (m_iLidarRotationStartAngle >= endAzimuth) ||
			abs(endAzimuth - m_iLidarRotationStartAngle) <= m_iAngleSize)))){
    return true;
  }
  else{
    return false;
  }
}

}  // namespace pandar_pointcloud
