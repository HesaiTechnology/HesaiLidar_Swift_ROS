/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Pandar40 3D LIDAR packets to PointCloud2.

*/
#include <ros/ros.h>
#include "convert.h"
#include "taskflow.hpp"
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/syscall.h>
#include <pandar_pointcloud/platUtil.h>


namespace pandar_pointcloud           
{

static tf::Executor executor;

float degreeToRadian(float degree) { return degree * M_PI / 180.0f; }

static const float elev_angle[] = {\
    14.436f, 13.535f, 13.08f, 12.624f, 12.163f, 11.702f, 11.237f, 10.771f, \
    10.301f, 9.83f, 9.355f, 8.88f, 8.401f, 7.921f, 7.437f, 6.954f, \
    6.467f, 5.98f, 5.487f, 4.997f, 4.501f, 4.009f, 3.509f, 3.014f, \
    2.512f, 2.014f, 1.885f, 1.761f, 1.637f, 1.511f, 1.386f, 1.258f, \
    1.13f, 1.009f, 0.88f, 0.756f, 0.63f, 0.505f, 0.379f, 0.251f, \
    0.124f, 0.0f, -0.129f, -0.254f, -0.38f, -0.506f, -0.632f, -0.76f, \
    -0.887f, -1.012f, -1.141f, -1.266f, -1.393f, -1.519f, -1.646f, -1.773f, \
    -1.901f, -2.027f, -2.155f, -2.282f, -2.409f, -2.535f, -2.662f, -2.789f, \
    -2.916f, -3.044f, -3.172f, -3.299f, -3.425f, -3.552f, -3.680f, -3.806f, \
    -3.933f, -4.062f, -4.190f, -4.318f, -4.444f, -4.571f, -4.698f, -4.824f, \
    -4.951f, -5.081f, -5.209f, -5.336f, -5.463f, -5.589f, -5.717f, -5.843f, \
    -5.968f, -6.099f, -6.607f, -7.118f, -7.624f, -8.135f, -8.64f, -9.149f, \
    -9.652f, -10.16f, -10.664f, -11.17f, -11.67f, -12.174f, -12.672f, -13.173f, \
    -13.668f, -14.166f, -14.658f, -15.154f, -15.643f, -16.135f, -16.62f, -17.108f, \
    -17.59f, -18.073f, -18.548f, -19.031f, -19.501f, -19.981f, -20.445f, -20.92f, \
    -21.379f, -21.85f, -22.304f, -22.77f, -23.219f, -23.68f, -24.123f, -25.016f, \
};

static const float azimuth_offset[] = {\
    3.257f, 3.263f, -1.083f, 3.268f, -1.086f, 3.273f, -1.089f, 3.278f, \
    -1.092f, 3.283f, -1.094f, 3.288f, -1.097f, 3.291f, -1.1f, 1.1f, \
    -1.102f, 1.1f, -3.306f, 1.102f, -3.311f, 1.103f, -3.318f, 1.105f, \
    -3.324f, 1.106f, 7.72f, 5.535f, 3.325f, -3.33f, -1.114f, -5.538f, \
    -7.726f, 1.108f, 7.731f, 5.543f, 3.329f, -3.336f, -1.116f, -5.547f, \
    -7.738f, 1.108f, 7.743f, 5.551f, 3.335f, -3.342f, -1.119f, -5.555f, \
    -7.75f, 1.11f, 7.757f, 5.56f, 3.34f, -3.347f, -1.121f, -5.564f, \
    -7.762f, 1.111f, 7.768f, 5.569f, 3.345f, -3.353f, -1.123f, -5.573f, \
    -7.775f, 1.113f, 7.780f, 5.578f, 3.351f, -3.358f, -1.125f, -5.582f, \
    -7.787f, 1.115f, 7.792f, 5.586f, 3.356f, -3.363f, -1.126f, -5.591f, \
    -7.799f, 1.117f, 7.804f, 5.595f, 3.36f, -3.369f, -1.128f, -5.599f, \
    -7.811f, 1.119f, -3.374f, 1.12f, -3.379f, 1.122f, -3.383f, 3.381f, \
    -3.388f, 3.386f, -1.135f, 3.39f, -1.137f, 3.395f, -1.138f, 3.401f, \
    -1.139f, 3.406f, -1.14f, 3.41f, -1.141f, 3.416f, -1.142f, 1.14f, \
    -1.143f, 1.143f, -3.426f, 1.146f, -3.429f, 1.147f, -3.433f, 1.15f, \
    -3.436f, 1.152f, -3.44f, 1.154f, -3.443f, 1.157f, -3.446f, -3.449f, \
};

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh, std::string node_type):
  data_(new pandar_rawdata::RawData()), drv(node , private_nh , node_type,this)
{

int maxpolicy = sched_get_priority_max(SCHED_RR);
ROS_WARN("Convert ,max[%d]",maxpolicy);

int minpolicy = sched_get_priority_min( SCHED_RR);
ROS_WARN("Convert ,min[%d]",minpolicy);
  publishmodel = "";
  if(LIDAR_NODE_TYPE == node_type) {
    private_nh.getParam("publish_model", publishmodel);
    double start_angle;
    private_nh.param("start_angle", start_angle, 0.0);
    lidarRotationStartAngle = int(start_angle * 100);
    data_->setup(private_nh);
    // advertise output point cloud (before subscribing to input data)
    srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
          CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
    CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);
  }

  private_nh.getParam("frame_id", frame_id_);
  private_nh.getParam("firetime_file", lidarFiretimeFile);
  private_nh.getParam("calibration", lidarCorrectionFile);
  
  ROS_WARN("frame_id [%s]",frame_id_.c_str());
  ROS_WARN("lidarFiretimeFile [%s]",lidarFiretimeFile.c_str());
  ROS_WARN("lidarCorrectionFile [%s]",lidarCorrectionFile.c_str());

  
  m_iWorkMode = 0;
  m_iReturnMode = 0;

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

  tz_second_ = 0*3600; //time zone
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

  std::ifstream fin(lidarCorrectionFile);
  int length = 0;
  std::string strlidarCalibration;
  fin.seekg(0, std::ios::end);
  length = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char *buffer = new char[length];
  fin.read(buffer, length);
  fin.close();
  strlidarCalibration = buffer;

  loadCorrectionFile(strlidarCalibration);
  loadOffsetFile(lidarFiretimeFile);  // parameter is the path of lidarFiretimeFil
  ROS_WARN("node_type[%s]",node_type.c_str());
  if(LIDAR_NODE_TYPE == node_type) {
    boost::thread thrd(boost::bind(&Convert::DriverReadThread, this));
  }

  if (publishmodel == "both_point_raw" || publishmodel == "point" || LIDAR_NODE_TYPE != node_type) {
    ROS_WARN("node.advertise pandar_points");
    output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10000);
    boost::thread processThr(boost::bind(&Convert::processLiDARData, this)); 
  }

  if ((publishmodel == "both_point_raw" || publishmodel == "raw") && LIDAR_NODE_TYPE == node_type) {
    boost::thread processThr(boost::bind(&Convert::publishRawDataThread, this)); 
  }
}

int Convert::loadCorrectionFile(std::string correction_content) {
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  float pitchList[PANDAR128_LASER_NUM];
  float azimuthList[PANDAR128_LASER_NUM];

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    if (line.length() < strlen("1,1,1")) {
      break;
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
      printf("laser id error %d %d\n", lineId, lineCounter);
      break;
    }

    pitchList[lineId-1] = elev;
    azimuthList[lineId-1] = azimuth;
  }

  for (int i = 0; i < lineCounter; ++i) {
    elev_angle_[i] = pitchList[i];
    horizatal_azimuth_[i] = azimuthList[i];
  }

  return 0;
}

void Convert::DriverReadThread()
{
  sched_param param;
  param.sched_priority = 99;
  //SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  ROS_WARN("DriverReadThread:set result [%d]",rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("DriverReadThread:get thead %lu, policy %d and priority %d\n", pthread_self(), ret_policy, param.sched_priority);

  while(1)
  {
      drv.poll();
  }
}

void Convert::publishRawDataThread()
{
  sched_param param;
  param.sched_priority = 90;
  //SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  ROS_WARN("publishRawDataThread:set result [%d]",rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("publishRawDataThread:get thead %lu, policy %d and priority %d\n", pthread_self(), ret_policy, param.sched_priority);

  while(1)
  {
      drv.publishRawData();
  }
}

void Convert::callback(pandar_pointcloud::CloudNodeConfig &config,
                       uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    data_->setParameters(config.min_range, config.max_range, config.view_direction,
                         config.view_width);
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg)
{
    if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
    // outMsg->is_dense = false;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    // for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    // {
    //     data_->unpack(scanMsg->packets[i], *outMsg);
    // }
    double firstStamp = 0.0f;
    int ret = data_->unpack(scanMsg, *outMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);

    // publish the accumulated cloud message
	ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
					 << " Pandar40 points, time: " << outMsg->header.stamp);

    if(ret == 1)
    {
        pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
        // output_.publish(outMsg);
    }
}

void Convert::processGps(pandar_msgs::PandarGps &gpsMsg)
{

    struct tm t;
    t.tm_sec = gpsMsg.second;
    t.tm_min = gpsMsg.minute;
    t.tm_hour = gpsMsg.hour;
    t.tm_mday = gpsMsg.day;
    t.tm_mon = gpsMsg.month - 1;
    t.tm_year = gpsMsg.year + 2000 - 1900;
    t.tm_isdst = 0;
    if(lastGPSSecond != (mktime(&t) + 1))
    {
        lastGPSSecond = (mktime(&t) + 1);
        gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
        gps2.used = 0;
    }
     // ROS_ERROR("Got a gps data %d " ,gps2.gps);
}

void Convert::pushLiDARData(pandar_msgs::PandarPacket packet)
{
  //  ROS_WARN("Convert::pushLiDARData");
    pthread_mutex_lock(&piclock);
    Pandar128Packet pkt;
    if (0 != parseData(pkt, &packet.data[0], packet.size)) {
      pthread_mutex_unlock(&piclock);
      // ROS_WARN("Convert::pushLiDARData return");
      return;
    }
    lidar_packets_.push_back(pkt);
    sem_post(&picsem);
    pthread_mutex_unlock(&piclock);

    // pthread_mutex_lock(&piclock);
    // lidar_packets_.push_back(packet);
    // // ROS_WARN("pushLiDARData [%d]",lidar_packets_.size());
    // if(lidar_packets_.size() > 6)
    // {
    //     sem_post(&picsem);
    // }
    // pthread_mutex_unlock(&piclock);

}

int Convert::parseData(Pandar128Packet &packet, \
    const uint8_t *recvbuf, const int len) {
  int index = 0;
  int lidarType = -1;

  if (recvbuf[0] != 0xEE && recvbuf[1] != 0xFF && \
      recvbuf[2] != 1 && recvbuf[2] != 3) {
    ROS_WARN("Lidar type is error\n");
    return -1;
  }

  memcpy(&packet, recvbuf, sizeof(Pandar128Packet));

  return 0;
}

int Convert::processLiDARData()
{
  double lastTimestamp = 0.0;
  struct timespec ts;
  int ret = 0;

  int pktCount = 0;
  int cursor = 0;

int maxpolicy = sched_get_priority_max(SCHED_RR);
ROS_WARN(",max[%d]",maxpolicy);

int minpolicy = sched_get_priority_min(SCHED_RR);
ROS_WARN(",min[%d]",minpolicy);


  sched_param param;
  param.sched_priority = 99;
  //SCHED_FIFO和SCHED_RR
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  ROS_WARN("processLiDARData:pthread_setschedparam result [%d]", rc);
  int ret_policy;
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  ROS_WARN("processLiDARData:get thead %lu, policy %d and priority %d\n", pthread_self(), ret_policy, param.sched_priority);

  while (1) {
    // ROS_WARN("Convert::processLiDARData");
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      std::cout << "get time error" << std::endl;
    }

    ts.tv_sec += 1;

    if (sem_timedwait(&picsem, &ts) == -1) {
      ROS_WARN("Convert::processLiDARData sem_timedwait");
      continue;
    }

    if (!lidar_packets_.hasEnoughPackets()){
      // ROS_WARN("Convert::processLiDARData don't has Enough Packets");
      continue;
    }
    // ROS_WARN("Convert::processLiDARData pktCount[%d]",pktCount);

    if((0 != pktCount) && (0 == pktCount % (CIRCLE_ANGLE/(TASKFLOW_STEP_SIZE*2*m_iAngleSize/100/m_iReturnBlockSize)))) {
      ROS_WARN("#########angle[%u],", lidar_packets_.getTaskBegin()->blocks[0].fAzimuth);
      ROS_WARN("pktCount[%d]", pktCount);
      ROS_WARN("ts %lf cld size %u", timestamp, outMsgArray[cursor]->points.size());
      pcl_conversions::toPCL(ros::Time(timestamp), outMsgArray[cursor]->header.stamp);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*outMsgArray[cursor], output);
      output_.publish(output);
      // pcl_callback_(outMsgArray[cursor], timestamp,scan);
      cursor = (cursor + 1) % 2;
      timestamp = 0;
      uint32_t startTick = GetTickCount();
      //outMsgArray[cursor]->clear();
      //outMsgArray[cursor]->assign(CIRCLE_ANGLE*100/m_iAngleSize*128*m_iReturnBlockSize,p);
      uint32_t endTick = GetTickCount();
      if(endTick - startTick > 2) {
        ROS_WARN("OutMagArray time:%d",endTick - startTick);
      }
    }

    if(0 == checkLiadaMode()){
      ROS_WARN("checkLiadaMode now!!");
      // outMsgArray[cursor]->clear();
      //outMsgArray[cursor]->assign(CIRCLE_ANGLE*100/m_iAngleSize*128*m_iReturnBlockSize,PPoint());
      lidar_packets_.creatNewTask();
      pktCount = 0;
      continue;
    }
    // ROS_WARN("angle[%u][%u],", lidar_packets_.getTaskBegin()->blocks[0].fAzimuth,lidar_packets_.getTaskBegin()->blocks[1].fAzimuth);
    uint32_t startTick = GetTickCount();
    doTaskFlow(cursor);
    uint32_t endTick = GetTickCount();
    if(endTick - startTick > 2) {
      ROS_WARN("taskflow time:%d",endTick - startTick);
    }
    pktCount++;

    outMsgArray[cursor]->header.frame_id = frame_id_;
    outMsgArray[cursor]->height = 1;

    // double lastTimestamp = 0.0f;
    // pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
    // int frame_id = 0;
    // struct timespec ts;
    // while(1)
    // {ROS_WARN("processLiDARData");
    //     if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
    //     {
    //         ROS_ERROR("get time error");
    //     }

    //     ts.tv_sec += 1;
    //     if (sem_timedwait(&picsem, &ts) == -1)
    //     {
    //         ROS_WARN("No Pic");
    //         continue;
    //     }
    //     pthread_mutex_lock(&piclock);
    //     pandar_msgs::PandarPacket packet = lidar_packets_.front();
    //     lidar_packets_.pop_front();
    //     pthread_mutex_unlock(&piclock);

    //     if (output_.getNumSubscribers() == 0) { // no one listening?
    //         ROS_WARN("No NumSubscribers");
    //         continue;
    //     }    // avoid much work

    //     // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    //     // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
    //     // outMsg->is_dense = false;
    //     outMsg->header.frame_id = "pandar";
    //     outMsg->height = 1;



    //     double firstStamp = 0.0f;
    //     int ret = data_->unpack(packet, *outMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);

    //     if(ret == 1)
    //     {
    //         // ROS_ERROR("timestamp : %f " , firstStamp);
    //         if(lastTimestamp != 0.0f)
    //         {
    //             if(lastTimestamp > firstStamp)
    //             {
    //                 ROS_ERROR("errrrrrrrrr");
    //             }
    //         }

    //         lastTimestamp = firstStamp;
    //         if(hasGps)
    //         {
    //           pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
    //         }
    //         else
    //         {
    //           pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
    //         }
    //         output_.publish(outMsg);
    //         outMsg->clear();


    //     }
    // }
  }
}

void Convert::doTaskFlow(int cursor) {
    tf::Taskflow taskFlow;
    taskFlow.parallel_for(lidar_packets_.getTaskBegin(), lidar_packets_.getTaskEnd(), \
    [this,&cursor](auto &taskpkt) {
      calcPointXYZIT(taskpkt, outMsgArray[cursor]);
    }
    );
    executor.run(taskFlow).wait();
    lidar_packets_.creatNewTask();
}

int Convert::checkLiadaMode() {
  uint8_t lidarworkmode = lidar_packets_.getTaskEnd()->tail.nShutdownFlag & 0x03;
  uint8_t lidarreturnmode = lidar_packets_.getTaskEnd()->tail.nReturnMode;
  if(lidarworkmode ==3 || lidarreturnmode== 59){ //for some error value
    return 1;
  }
  if (0 == m_iWorkMode && 0 == m_iReturnMode) {
    m_iWorkMode = lidarworkmode;
    m_iReturnMode = lidarreturnmode;
    ROS_WARN("init m_iWorkMode[%d], lidarworkmode[%d],m_iReturnMode[%x],lidarreturnmode[%x],",m_iWorkMode,lidarworkmode,m_iReturnMode,lidarreturnmode);
    if(0 == m_iWorkMode){
      m_iAngleSize = LIDAR_ANGLE_SIZE_10; //10->0.1degree,20->0.2degree
    }else{
      m_iAngleSize = LIDAR_ANGLE_SIZE_20; //10->0.1degree,20->0.2degree
    }
    if(0x39 == m_iReturnMode){
      m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_2;
    }else{
      m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_1; 
    }
    boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(CIRCLE_ANGLE*100/m_iAngleSize*128*m_iReturnBlockSize, 1));
    boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(CIRCLE_ANGLE*100/m_iAngleSize*128*m_iReturnBlockSize, 1));
    outMsgArray[0] = outMag0;
    outMsgArray[1] = outMag1;
    return 1;
  }
  else{
    if( m_iWorkMode != lidarworkmode) {
      ROS_WARN("modify m_iWorkMode m_iWorkMode[%d], lidarworkmode[%d],m_iReturnMode[%x],lidarreturnmode[%x],",m_iWorkMode,lidarworkmode,m_iReturnMode,lidarreturnmode);
      m_iWorkMode = lidarworkmode;
      if(0 == m_iWorkMode){
        m_iAngleSize = LIDAR_ANGLE_SIZE_10; //10->0.1degree,20->0.2degree
      }else{
        m_iAngleSize = LIDAR_ANGLE_SIZE_20; //10->0.1degree,20->0.2degree
      }
      return 0;
    }
    if(m_iReturnMode != lidarreturnmode) {
      ROS_WARN("modify m_iReturnMode m_iWorkMode[%d], lidarworkmode[%d],m_iReturnMode[%x],lidarreturnmode[%x],",m_iWorkMode,lidarworkmode,m_iReturnMode,lidarreturnmode);
      m_iReturnMode = lidarreturnmode;
      if(0x39 == m_iReturnMode){
       m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_2;
      }else{
        m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_1; 
      }
      return 0;
    }
    return 1;
  }
}

void Convert::calcPointXYZIT(Pandar128Packet &pkt, boost::shared_ptr<PPointCloud> &cld) {
  SetThreadPriority(SCHED_FIFO, 99);

  // ROS_WARN("#####block.fAzimuth[%u][%u]",pkt.blocks[0].fAzimuth,pkt.blocks[1].fAzimuth);
  for (int blockid = 0; blockid < pkt.head.u8BlockNum; blockid++) {
    Pandar128Block &block = pkt.blocks[blockid];
    struct tm t;

    t.tm_year  = pkt.tail.nUTCTime[0];
    t.tm_mon   = pkt.tail.nUTCTime[1] - 1;
    t.tm_mday  = pkt.tail.nUTCTime[2];
    t.tm_hour  = pkt.tail.nUTCTime[3];
    t.tm_min   = pkt.tail.nUTCTime[4];
    t.tm_sec   = pkt.tail.nUTCTime[5];
    t.tm_isdst = 0;

    double unix_second = static_cast<double>(mktime(&t) + tz_second_);

    int mode  = pkt.tail.nShutdownFlag & 0x03;
    int state = (pkt.tail.nShutdownFlag & 0xF0) >> 4;

    for (int i = 0; i < pkt.head.u8LaserNum; i++) { 
      /* for all the units in a block */
      Pandar128Unit &unit = block.units[i];
      PPoint point;

      float distance = static_cast<float>(unit.u16Distance) * \
          PANDAR128_DISTANCE_UNIT;
      /* filter distance */
      if (distance < 0.1) {
        continue;
      }

      float azimuth = horizatal_azimuth_[i] + (block.fAzimuth / 100.0f);
      float originAzimuth = azimuth;
      float pitch = elev_angle_[i];
      float originPitch = pitch;
      int   offset = laserOffset.getTSOffset(i, mode, state, distance);

      azimuth += laserOffset.getAngleOffset(offset);

      pitch += laserOffset.getPitchOffset(frame_id_, pitch, distance);

      if (pitch < 0) {
        pitch += 360.0f;
      } else if (pitch >= 360.0f) {
        pitch -= 360.0f;
      }

      float xyDistance = distance * \
          cos_all_angle_[static_cast<int>(pitch * 100 + 0.5)];
      azimuth += laserOffset.getAzimuthOffset(frame_id_, originAzimuth, \
          block.fAzimuth / 100.0f, xyDistance);

      int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
      if (azimuthIdx >= CIRCLE) {
        azimuthIdx -= CIRCLE;
      } else if (azimuthIdx < 0) {
        azimuthIdx += CIRCLE;
      }

      point.x = xyDistance * sin_all_angle_[azimuthIdx];
      point.y = xyDistance * cos_all_angle_[azimuthIdx];
      point.z = distance * sin_all_angle_[static_cast<int>(pitch * 100 + 0.5)];

      point.intensity = unit.u8Intensity;
      point.timestamp = unix_second + \
          (static_cast<double>(pkt.tail.nTimestamp)) / 1000000.0;

      point.timestamp = point.timestamp + laserOffset.getBlockTS(blockid, \
          pkt.tail.nReturnMode, mode) / 1000000000.0 + offset / 1000000000.0;

      if(0 == timestamp) {
        timestamp = point.timestamp;
      }
      else if(timestamp > point.timestamp){
        timestamp = point.timestamp;
      }
      
      point.ring = i;

      int index;
      if(LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
        index = (block.fAzimuth - start_angle_)/m_iAngleSize*128*m_iReturnBlockSize + 128*(m_iReturnBlockSize-1) + i;
        // ROS_WARN("block 2 index:[%d]",index);
      }
      else{
        index = (block.fAzimuth - start_angle_)/m_iAngleSize*128 + i;
      }
      cld->points[index] = point;
    }
  }
}

void Convert::loadOffsetFile(std::string file) {
  laserOffset.setFilePath(file);
}


void Convert::processGps(const pandar_msgs::PandarGps::ConstPtr &gpsMsg)
{
    hasGps = 1;
    struct tm t;
    t.tm_sec = gpsMsg->second;
    t.tm_min = gpsMsg->minute;
    t.tm_hour = gpsMsg->hour;
    t.tm_mday = gpsMsg->day;
    t.tm_mon = gpsMsg->month - 1;
    t.tm_year = gpsMsg->year + 2000 - 1900;
    t.tm_isdst = 0;
    if(lastGPSSecond != (mktime(&t) + 1))
    {
        lastGPSSecond = (mktime(&t) + 1);
        gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
        gps2.used = 0;
    }
    // ROS_ERROR("Got data second : %f " ,(double)gps2.gps);
}

} // namespace pandar_pointcloud
