/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Pandar40 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Yang Sheng
 */

#ifndef __PANDAR_RAWDATA_H
#define __PANDAR_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pandar_msgs/PandarScan.h>
#include <pandar_msgs/PandarGps.h>
#include <pandar_pointcloud/point_types.h>
#include <pandar_pointcloud/calibration.h>

namespace pandar_rawdata
{
// Shorthand typedefs for point cloud representations
typedef pandar_pointcloud::PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;

static const int SOB_ANGLE_SIZE = 4;
static const int RAW_MEASURE_SIZE = 5;
static const int LASER_COUNT = 40;
static const int BLOCKS_PER_PACKET = 6;
static const int BLOCK_SIZE = RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE;
static const int TIMESTAMP_SIZE = 4;
static const int FACTORY_ID_SIZE = 2;
static const int RESERVE_SIZE = 8;
static const int REVOLUTION_SIZE = 2;
static const int INFO_SIZE = TIMESTAMP_SIZE + FACTORY_ID_SIZE
                             + RESERVE_SIZE + REVOLUTION_SIZE;
static const int PACKET_SIZE = BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE;
static const int ROTATION_MAX_UNITS = 36001;
static const float ROTATION_RESOLUTION = 0.01;

typedef struct raw_measure {
    uint32_t range;
    uint16_t reflectivity;
} raw_measure_t;

/** \brief Raw Pandar40 data block.
 */
typedef struct raw_block
{
    uint16_t sob;
    uint16_t azimuth;
    raw_measure_t measures[LASER_COUNT];
} raw_block_t;


/** \brief Raw Pandar40 packet.
 */
typedef struct raw_packet
{
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint8_t reserved[RESERVE_SIZE];
    uint16_t revolution;
    uint32_t timestamp;
    uint8_t factory[2];
    double recv_time;
} raw_packet_t;

typedef struct gps_struct{
    int used;
    time_t gps;
}gps_struct_t;

/** \brief Pandar40 data conversion class */
class RawData
{
public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - read device-specific angles calibration
     *
     *  @param private_nh private node handle for ROS parameters
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh);

    /** \brief Set up for data processing offline.
      * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
      * this method is useful if unpacking data directly from bag files, without passing
      * through a communication overhead.
      *
      * @param calibration_file path to the calibration file
      * @param max_range_ cutoff for maximum range
      * @param min_range_ cutoff for minimum range
      * @returns 0 if successful;
      *           errno value for failure
     */
    int setupOffline(std::string calibration_file, double max_range_, double min_range_);

    void unpack(const pandar_msgs::PandarPacket &pkt, PPointCloud &pc);
    int unpack(const pandar_msgs::PandarScan::ConstPtr &scanMsg, PPointCloud &pc, time_t& gps1 , 
                                            gps_struct_t &gps2 , double& firstStamp, int& lidarRotationStartAngle);

    int unpack(pandar_msgs::PandarPacket &packet, PPointCloud &pc, time_t& gps1 , 
                                            gps_struct_t &gps2 , double& firstStamp, int& lidarRotationStartAngle);

    void setParameters(double min_range, double max_range, double view_direction,
                       double view_width);

private:

    /** configuration parameters */
    typedef struct {
        std::string calibrationFile;     ///< calibration file name
        double max_range;                ///< maximum range to publish
        double min_range;                ///< minimum range to publish
        int min_angle;                   ///< minimum angle to publish
        int max_angle;                   ///< maximum angle to publish

        double tmp_min_angle;
        double tmp_max_angle;
    } Config;
    Config config_;

    /**
     * Calibration file
     */
    pandar_pointcloud::Calibration calibration_;
    float sin_lookup_table_[ROTATION_MAX_UNITS];
    float cos_lookup_table_[ROTATION_MAX_UNITS];

    /** in-line test whether a point is in range */
    bool pointInRange(float range)
    {
        return (range >= config_.min_range
                && range <= config_.max_range);
    }

	int parseRawData(raw_packet* packet, const uint8_t* buf, const int len);
	void toPointClouds (raw_packet* packet, PPointCloud& pc);
    void toPointClouds (raw_packet_t* packet,int laser ,  PPointCloud& pc , double stamp , double& firstStamp);
    void toPointClouds (raw_packet_t* packet,int laser , int block,  PPointCloud& pc);
	void computeXYZIR(PPoint& point, int azimuth,
			const raw_measure_t& laserReturn,
			const pandar_pointcloud::PandarLaserCorrection& correction);

    int lastBlockEnd;

    raw_packet_t *bufferPacket;
    int bufferPacketSize;

    int currentPacketStart;

    int lastTimestamp;

    int lastAzumith;
};

} // namespace pandar_rawdata

#endif // __PANDAR_RAWDATA_H
