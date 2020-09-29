/* -*- mode: C++ -*- */
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

#ifndef _PANDAR_POINTCLOUD_CONVERT_H_
#define _PANDAR_POINTCLOUD_CONVERT_H_ 1

#include <pandar_pointcloud/rawdata.h>
#include <pthread.h>
#include <ros/ros.h>
#include <semaphore.h>
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <pandar_msgs/PandarGps.h>
#include <pandar_msgs/PandarPacket.h>
#include <pandar_msgs/PandarScan.h>
#include <pandar_pointcloud/CloudNodeConfig.h>
#include <pandar_pointcloud/calibration.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/atomic.hpp>
#include <boost/lockfree/queue.hpp>
#include "driver.h"
#include "laser_ts.h"

#ifndef CIRCLE
#define CIRCLE (36000)
#endif

#define LIDAR_NODE_TYPE "lidar"
#define LIDAR_ANGLE_SIZE_10 (10)
#define LIDAR_ANGLE_SIZE_20 (20)
#define LIDAR_RETURN_BLOCK_SIZE_1 (1)
#define LIDAR_RETURN_BLOCK_SIZE_2 (2)

#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define PANDAR128_LASER_NUM (128)
#define PANDAR128_BLOCK_NUM (2)
#define MAX_BLOCK_NUM (8)
#define PANDAR128_DISTANCE_UNIT (0.004)
#define PANDAR128_SOB_SIZE (2)
#define PANDAR128_VERSION_MAJOR_SIZE (1)
#define PANDAR128_VERSION_MINOR_SIZE (1)
#define PANDAR128_HEAD_RESERVED1_SIZE (2)
#define PANDAR128_LASER_NUM_SIZE (1)
#define PANDAR128_BLOCK_NUM_SIZE (1)
#define PANDAR128_ECHO_COUNT_SIZE (1)
#define PANDAR128_ECHO_NUM_SIZE (1)
#define PANDAR128_HEAD_RESERVED2_SIZE (2)
#define PANDAR128_HEAD_SIZE                                       \
  (PANDAR128_SOB_SIZE + PANDAR128_VERSION_MAJOR_SIZE +            \
   PANDAR128_VERSION_MINOR_SIZE + PANDAR128_HEAD_RESERVED1_SIZE + \
   PANDAR128_LASER_NUM_SIZE + PANDAR128_BLOCK_NUM_SIZE +          \
   PANDAR128_ECHO_COUNT_SIZE + PANDAR128_ECHO_NUM_SIZE +          \
   PANDAR128_HEAD_RESERVED2_SIZE)
#define PANDAR128_AZIMUTH_SIZE (2)
#define DISTANCE_SIZE (2)
#define INTENSITY_SIZE (1)
#define CONFIDENCE_SIZE (1)
#define PANDAR128_UNIT_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)
#define PANDAR128_BLOCK_SIZE \
  (PANDAR128_UNIT_SIZE * PANDAR128_LASER_NUM + PANDAR128_AZIMUTH_SIZE)
#define PANDAR128_TAIL_RESERVED1_SIZE (3)
#define PANDAR128_TAIL_RESERVED2_SIZE (3)
#define PANDAR128_SHUTDOWN_FLAG_SIZE (1)
#define PANDAR128_TAIL_RESERVED3_SIZE (3)
#define PANDAR128_MOTOR_SPEED_SIZE (2)
#define PANDAR128_TS_SIZE (4)
#define PANDAR128_RETURN_MODE_SIZE (1)
#define PANDAR128_FACTORY_INFO (1)
#define PANDAR128_UTC_SIZE (6)
#define PANDAR128_TAIL_SIZE                                        \
  (PANDAR128_TAIL_RESERVED1_SIZE + PANDAR128_TAIL_RESERVED2_SIZE + \
   PANDAR128_SHUTDOWN_FLAG_SIZE + PANDAR128_TAIL_RESERVED3_SIZE +  \
   PANDAR128_MOTOR_SPEED_SIZE + PANDAR128_TS_SIZE +                \
   PANDAR128_RETURN_MODE_SIZE + PANDAR128_FACTORY_INFO + PANDAR128_UTC_SIZE)
#define PANDAR128_PACKET_SIZE                                         \
  (PANDAR128_HEAD_SIZE + PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM + \
   PANDAR128_TAIL_SIZE)
#define PANDAR128_SEQ_NUM_SIZE (4)
#define PANDAR128_PACKET_SEQ_NUM_SIZE \
  (PANDAR128_PACKET_SIZE + PANDAR128_SEQ_NUM_SIZE)
#define PANDAR128_WITHOUT_CONF_UNIT_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)
#define TASKFLOW_STEP_SIZE (360)
#define PANDAR128_CRC_SIZE (4)
#define PANDAR128_FUNCTION_SAFETY_SIZE (17)

#define ETHERNET_MTU (1500)

#define CIRCLE_ANGLE (360)

typedef struct PandarPacket_s {
  double stamp;
  uint8_t data[ETHERNET_MTU];
  uint32_t size;
} PandarPacket;

typedef struct __attribute__((__packed__)) Pandar128Unit_s {
  uint16_t u16Distance;
  uint8_t u8Intensity;
  // uint8_t  u8Confidence;
} Pandar128Unit;

typedef struct __attribute__((__packed__)) Pandar128Block_s {
  uint16_t fAzimuth;
  Pandar128Unit units[PANDAR128_LASER_NUM];
} Pandar128Block;

typedef struct Pandar128Head_s {
  uint16_t u16Sob;
  uint8_t u8VersionMajor;
  uint8_t u8VersionMinor;
  uint8_t u8DistUnit;
  uint8_t u8Flags;
  uint8_t u8LaserNum;
  uint8_t u8BlockNum;
  uint8_t u8EchoCount;
  uint8_t u8EchoNum;
  uint16_t u16Reserve1;
} Pandar128Head;

typedef struct Pandar128Tail_s {
  uint8_t nReserved1[3];
  uint8_t nReserved2[3];
  uint8_t nShutdownFlag;
  uint8_t nReserved3[3];
  uint16_t nMotorSpeed;
  uint32_t nTimestamp;
  uint8_t nReturnMode;
  uint8_t nFactoryInfo;
  uint8_t nUTCTime[6];
  uint32_t nSeqNum;
} Pandar128Tail;

typedef struct __attribute__((__packed__)) Pandar128Packet_t {
  Pandar128Head head;
  Pandar128Block blocks[PANDAR128_BLOCK_NUM];
  Pandar128Tail tail;
} Pandar128Packet;

struct PandarGPS_s {
  uint16_t flag;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t second;
  uint16_t minute;
  uint16_t hour;
  uint32_t fineTime;
};

typedef std::array<Pandar128Packet, 36000> PktArray;

typedef struct PacketsBuffer_s {
  PktArray m_buffers{};
  PktArray::iterator m_iterPush;
  PktArray::iterator m_iterTaskBegin;
  PktArray::iterator m_iterTaskEnd;
  int m_stepSize;
  bool m_startFlag;
  inline PacketsBuffer_s() {
    m_stepSize = TASKFLOW_STEP_SIZE;
    m_iterPush = m_buffers.begin();
    m_iterTaskBegin = m_buffers.begin();
    m_iterTaskEnd = m_iterTaskBegin + m_stepSize;
    m_startFlag = false;
  }
  inline int push_back(Pandar128Packet pkt) {
    if (!m_startFlag) {
      *(m_iterPush++) = pkt;
      m_startFlag = true;
      return 1;
    } else {
      if (m_buffers.end() == m_iterPush) {
        m_iterPush = m_buffers.begin();
      }

      static bool lastOverflowed = false;

      if (m_iterPush == m_iterTaskBegin) {
        static uint32_t tmp = m_iterTaskBegin - m_buffers.begin();
        if (m_iterTaskBegin - m_buffers.begin() != tmp) {
          ROS_WARN("buffer don't have space!,%d",
                   m_iterTaskBegin - m_buffers.begin());
          tmp = m_iterTaskBegin - m_buffers.begin();
        }
        lastOverflowed = true;
        return 0;
      }
      if (lastOverflowed) {
        lastOverflowed = false;
        ROS_WARN("buffer recovered");
      }
      *(m_iterPush++) = pkt;
      return 1;
    }
  }
  inline bool hasEnoughPackets() {
    return ((m_iterPush > m_iterTaskBegin && m_iterPush > m_iterTaskEnd) ||
            (m_iterPush < m_iterTaskBegin && m_iterPush < m_iterTaskEnd));
  }
  inline PktArray::iterator getTaskBegin() { return m_iterTaskBegin; }
  inline PktArray::iterator getTaskEnd() { return m_iterTaskEnd; }
  inline void creatNewTask() {
    if (m_buffers.end() == m_iterTaskEnd) {
      ROS_WARN("creat new task end to start");
      m_iterTaskBegin = m_buffers.begin();
      m_iterTaskEnd = m_iterTaskBegin + m_stepSize;
    } else {
      m_iterTaskBegin = m_iterTaskEnd;
      m_iterTaskEnd = m_iterTaskBegin + m_stepSize;
    }
  }

} PacketsBuffer;

typedef pandar_pointcloud::PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;

namespace pandar_pointcloud {
class Convert {
 public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh,
          std::string nod_typee = LIDAR_NODE_TYPE);
  ~Convert() {}

  void DriverReadThread();
  void publishRawDataThread();
  void processGps(pandar_msgs::PandarGps &gpsMsg);
  void pushLiDARData(pandar_msgs::PandarPacket packet);
  int processLiDARData();
  void publishPointsThread();
  void publishPoints();

 private:
  void callback(pandar_pointcloud::CloudNodeConfig &config, uint32_t level);
  void processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg);
  void processGps(const pandar_msgs::PandarGps::ConstPtr &gpsMsg);

  int parseData(Pandar128Packet &pkt, const uint8_t *buf, const int len);
  void calcPointXYZIT(Pandar128Packet &pkt,
                      boost::shared_ptr<PPointCloud> &cld);
  void doTaskFlow(int cursor);
  void loadOffsetFile(std::string file);
  int loadCorrectionFile(std::string correction_content);
  int checkLiadaMode();

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<
      dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig> >
      srv_;

  boost::shared_ptr<pandar_rawdata::RawData> data_;
  ros::Subscriber pandar_scan_;
  ros::Subscriber pandar_gps_;
  ros::Publisher output_;

  /// configuration parameters
  typedef struct {
    int npackets;  ///< number of packets to combine
    double rpm;
  } Config;
  Config config_;

  time_t gps1;
  pandar_rawdata::gps_struct_t gps2;
  bool hasGps;

  unsigned int lastGPSSecond;
  int lidarRotationStartAngle;

  pandar_pointcloud::PandarDriver drv;

  pthread_mutex_t piclock;
  sem_t picsem;
  // std::list<pandar_msgs::PandarPacket> LiDARDataSet;

  std::array<boost::shared_ptr<PPointCloud>, 2> outMsgArray;
  PacketsBuffer lidar_packets_;
  double timestamp;
  int start_angle_;
  float cos_all_angle_[CIRCLE];
  float sin_all_angle_[CIRCLE];
  float elev_angle_[PANDAR128_LASER_NUM];
  float horizatal_azimuth_[PANDAR128_LASER_NUM];
  LasersTSOffset laserOffset;
  int tz_second_;
  std::string frame_id_;
  std::string lidarFiretimeFile;
  std::string lidarCorrectionFile;
  std::string publishmodel;
  int m_iWorkMode;
  int m_iReturnMode;
  int m_iAngleSize;  // 10->0.1degree,20->0.2degree
  int m_iReturnBlockSize;
  bool m_bPublishPointsFlag;
  int m_iPublishPointsIndex;
};

}  // namespace pandar_pointcloud

#endif  // _PANDAR_POINTCLOUD_CONVERT_H_
