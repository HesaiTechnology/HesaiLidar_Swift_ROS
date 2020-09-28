/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Pandar40 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Pandar40 LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Pandar40 data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Yang Sheng
 *
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <pandar_pointcloud/rawdata.h>

namespace pandar_rawdata
{

static double block_offset[BLOCKS_PER_PACKET];
static double laser_offset[LASER_COUNT];

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData()
{
    bufferPacket = new raw_packet_t[1000];
    bufferPacketSize = 0;
    lastBlockEnd = 0;
    lastTimestamp = 0;

    block_offset[5] = 55.1f * 0.0 + 45.18f;
    block_offset[4] = 55.1f * 1.0 + 45.18f;
    block_offset[3] = 55.1f * 2.0 + 45.18f;
    block_offset[2] = 55.1f * 3.0 + 45.18f;
    block_offset[1] = 55.1f * 4.0 + 45.18f;
    block_offset[0] = 55.1f * 5.0 + 45.18f;

    laser_offset[3] = 0.93f * 1.0f;
    laser_offset[35] = 0.93f * 2.0f;
    laser_offset[39] = 0.93f * 3.0f;
    laser_offset[23] = 0.93f * 3.0f + 1.6f * 1.0f;
    laser_offset[16] = 0.93f * 3.0f + 1.6f * 2.0f;
    laser_offset[27] = 0.93f * 4.0f + 1.6f * 2.0f;
    laser_offset[11] = 0.93f * 4.0f + 1.6f * 3.0f;
    laser_offset[31] = 0.93f * 5.0f + 1.6f * 3.0f;
    laser_offset[28] = 0.93f * 6.0f + 1.6f * 3.0f;
    laser_offset[15] = 0.93f * 6.0f + 1.6f * 4.0f;
    laser_offset[2] = 0.93f * 7.0f + 1.6f * 4.0f;
    laser_offset[34] = 0.93f * 8.0f + 1.6f * 4.0f;
    laser_offset[38] = 0.93f * 9.0f + 1.6f * 4.0f;
    laser_offset[20] = 0.93f * 9.0f + 1.6f * 5.0f;
    laser_offset[13] = 0.93f * 9.0f + 1.6f * 6.0f;
    laser_offset[24] = 0.93f * 9.0f + 1.6f * 7.0f;
    laser_offset[8] = 0.93f * 9.0f + 1.6f * 8.0f;
    laser_offset[30] = 0.93f * 10.0f + 1.6f * 8.0f;
    laser_offset[25] = 0.93f * 11.0f + 1.6f * 8.0f;
    laser_offset[12] = 0.93f * 11.0f + 1.6f * 9.0f;
    laser_offset[1] = 0.93f * 12.0f + 1.6f * 9.0f;
    laser_offset[33] = 0.93f * 13.0f + 1.6f * 9.0f;
    laser_offset[37] = 0.93f * 14.0f + 1.6f * 9.0f;
    laser_offset[17] = 0.93f * 14.0f + 1.6f * 10.0f;
    laser_offset[10] = 0.93f * 14.0f + 1.6f * 11.0f;
    laser_offset[21] = 0.93f * 14.0f + 1.6f * 12.0f;
    laser_offset[5] = 0.93f * 14.0f + 1.6f * 13.0f;
    laser_offset[29] = 0.93f * 15.0f + 1.6f * 13.0f;
    laser_offset[22] = 0.93f * 15.0f + 1.6f * 14.0f;
    laser_offset[9] = 0.93f * 15.0f + 1.6f * 15.0f;
    laser_offset[0] = 0.93f * 16.0f + 1.6f * 15.0f;
    laser_offset[32] = 0.93f * 17.0f + 1.6f * 15.0f;
    laser_offset[36] = 0.93f * 18.0f + 1.6f * 15.0f;
    laser_offset[14] = 0.93f * 18.0f + 1.6f * 16.0f;
    laser_offset[7] = 0.93f * 18.0f + 1.6f * 17.0f;
    laser_offset[18] = 0.93f * 18.0f + 1.6f * 18.0f;
    laser_offset[4] = 0.93f * 19.0f + 1.6f * 18.0f;
    laser_offset[26] = 0.93f * 20.0f + 1.6f * 18.0f;
    laser_offset[19] = 0.93f * 20.0f + 1.6f * 19.0f;
    laser_offset[6] = 0.93f * 20.0f + 1.6f * 20.0f;
    

}

/** Update parameters: conversions and update */
void RawData::setParameters(double min_range,
                            double max_range,
                            double view_direction,
                            double view_width)
{
    config_.min_range = min_range;
    config_.max_range = max_range;

	//TODO: YS: not support view angle setting currently.
    //converting angle parameters into the pandar reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;

    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);

    //converting into the hardware pandar ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
        //avoid returning empty cloud if min_angle = max_angle
        config_.min_angle = 0;
        config_.max_angle = 36000;
    }
}

/** Set up for on-line operation. */
int RawData::setup(ros::NodeHandle private_nh)
{
    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
    {
        ROS_ERROR_STREAM("No calibration angles specified! Using default values!");

        std::string pkgPath = ros::package::getPath("pandar_pointcloud");
        config_.calibrationFile = pkgPath + "/params/Lidar-Correction-18.csv";
    }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);
    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
        ROS_ERROR_STREAM("Unable to open calibration file: " <<
                         config_.calibrationFile);
        return -1;
    }

    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        cos_lookup_table_[rot_index] = cosf(rotation);
        sin_lookup_table_[rot_index] = sinf(rotation);
    }
    return 0;
}


/** Set up for offline operation */
int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
{

    config_.max_range = max_range_;
    config_.min_range = min_range_;
    ROS_INFO_STREAM("data ranges to publish: ["
                    << config_.min_range << ", "
                    << config_.max_range << "]");

    config_.calibrationFile = calibration_file;

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
        ROS_ERROR_STREAM("Unable to open calibration file: " <<
                         config_.calibrationFile);
        return -1;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        cos_lookup_table_[rot_index] = cosf(rotation);
        sin_lookup_table_[rot_index] = sinf(rotation);
    }
    return 0;
}

int RawData::parseRawData(raw_packet_t* packet, const uint8_t* buf, const int len)
{
    if(len != PACKET_SIZE) {
		ROS_WARN_STREAM("packet size mismatch!");
        return -1;
	}

    int index = 0;
    // 6x BLOCKs
    for(int i = 0 ; i < BLOCKS_PER_PACKET ; i++) {
		raw_block_t& block = packet->blocks[i];
        block.sob = (buf[index] & 0xff)| ((buf[index + 1] & 0xff)<< 8);
        block.azimuth = (buf[index + 2]& 0xff) | ((buf[index + 3]& 0xff) << 8);
        index += SOB_ANGLE_SIZE;
        // 40x measures
        for(int j = 0 ; j < LASER_COUNT ; j++) {
			raw_measure_t& measure = block.measures[j];
            measure.range = (buf[index]& 0xff)
				| ((buf[index + 1]& 0xff) << 8)
				| ((buf[index + 2]& 0xff) << 16 );
            measure.reflectivity = (buf[index + 3]& 0xff)
				| ((buf[index + 4]& 0xff) << 8);

            // TODO: Filtering wrong data for LiDAR Bugs.
            if((measure.range == 0x010101 && measure.reflectivity == 0x0101)
					|| measure.range > (200 * 1000 /2 /* 200m -> 2mm */)) {
                measure.range = 0;
                measure.reflectivity = 0;
            }
            index += RAW_MEASURE_SIZE;
        }
    }

    index += RESERVE_SIZE; // skip reserved bytes

    packet->revolution = (buf[index]& 0xff)| (buf[index + 1]& 0xff) << 8;
    index += REVOLUTION_SIZE;

    packet->timestamp = (buf[index]& 0xff)| (buf[index + 1]& 0xff) << 8 |
                        ((buf[index + 2 ]& 0xff) << 16) | ((buf[index + 3]& 0xff) << 24);
    index += TIMESTAMP_SIZE;
    packet->factory[0] = buf[index]& 0xff;
    packet->factory[1] = buf[index + 1]& 0xff;
    index += FACTORY_ID_SIZE;
    return 0;
}

void RawData::computeXYZIR(PPoint& point, int azimuth,
		const raw_measure_t& laserReturn, const pandar_pointcloud::PandarLaserCorrection& correction)
{
    double cos_azimuth, sin_azimuth;
    double distanceM = laserReturn.range * 0.002;

    point.intensity = static_cast<float> (laserReturn.reflectivity >> 8);
    if (distanceM < config_.min_range || distanceM > config_.max_range)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        return;
    }
    if (correction.azimuthCorrection == 0)
    {
        cos_azimuth = cos_lookup_table_[azimuth];
        sin_azimuth = sin_lookup_table_[azimuth];
    }
    else
    {
        double azimuthInRadians = angles::from_degrees( (static_cast<double> (azimuth) / 100.0) + correction.azimuthCorrection);
        cos_azimuth = std::cos (azimuthInRadians);
        sin_azimuth = std::sin (azimuthInRadians);
    }

    distanceM += correction.distanceCorrection;

    double xyDistance = distanceM * correction.cosVertCorrection;

    point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
    point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
    point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);

    // float a = point.x;
    // point.x = - point.y;
    // point.y = a;

    if (point.x == 0 && point.y == 0 && point.z == 0)
    {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
    }
}

static int PandarEnableList[LASER_COUNT] = {
	1,
	1,
	1,
    1,
	1,
    1,
	1,
    1,
	1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

void RawData::toPointClouds (raw_packet_t* packet, PPointCloud& pc)
{
    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
		const raw_block_t& firing_data = packet->blocks[i];

        for (int j = 0; j < LASER_COUNT; j++) {
	    if(PandarEnableList[j] != 1)
		continue;
            PPoint xyzir;
            computeXYZIR (xyzir, firing_data.azimuth,
					firing_data.measures[j], calibration_.laser_corrections[j]);
            if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
            {
                continue;
            }
			// xyzir.ring = j;
			pc.points.push_back(xyzir);
			pc.width++;
        }
    }
}

// void RawData::toPointClouds (raw_packet_t* packet, PPointCloud& pc)
// {
//     for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
//         const raw_block_t& firing_data = packet->blocks[i];

//         for (int j = 0; j < LASER_COUNT; j++) {
//         if(PandarEnableList[j] != 1)
//         continue;
//             PPoint xyzir;
//             computeXYZIR (xyzir, firing_data.azimuth,
//                     firing_data.measures[j], calibration_.laser_corrections[j]);
//             if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
//             {
//                 continue;
//             }
//             xyzir.ring = j;
//             pc.points.push_back(xyzir);
//             pc.width++;
//         }
//     }
// }

void RawData::toPointClouds (raw_packet_t* packet,int block ,  PPointCloud& pc , double stamp , double& firstStamp)
{
    int first = 0;
    const raw_block_t& firing_data = packet->blocks[block];
    for (int i = 0; i < LASER_COUNT; i++) {
        // if(i == 0)
        // {
        //     double cur_time = stamp - ((double)(block_offset[block] + laser_offset[i])/1000000.0f);
        //     // double cur_time = stamp - ((block_offset[block] + laser_offset[i])/1000000);
        //     double diff = packet->recv_time - cur_time;
        //     if(diff < 0.0f)
        //     {
        //         ROS_ERROR("ERROR TIME %lf %lf %f " , cur_time , packet->recv_time , diff);
        //     }
        // }
            PPoint xyzir;
            computeXYZIR (xyzir, firing_data.azimuth,
                    firing_data.measures[i], calibration_.laser_corrections[i]);
            if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
            {
                continue;
            }

            xyzir.timestamp = stamp - ((double)(block_offset[block] + laser_offset[i])/1000000.0f);
            if(!first)
            {
                // ROS_ERROR("%f" , xyzir.timestamp);
                firstStamp = xyzir.timestamp;
                first = 1;
            }
            

            xyzir.ring = i;
            pc.points.push_back(xyzir);
            pc.width++;
    }
}

void RawData::toPointClouds (raw_packet_t* packet,int laser , int block,  PPointCloud& pc)
{
    int i = block;
    {
        const raw_block_t& firing_data = packet->blocks[i];
            PPoint xyzir;
            computeXYZIR (xyzir, firing_data.azimuth,
                    firing_data.measures[laser], calibration_.laser_corrections[laser]);
            if (pcl_isnan (xyzir.x) || pcl_isnan (xyzir.y) || pcl_isnan (xyzir.z))
            {
                return;
            }
            // xyzir.ring = laser;
            pc.points.push_back(xyzir);
            pc.width++;
    }
}

int RawData::unpack(pandar_msgs::PandarPacket &packet, PPointCloud &pc, time_t& gps1 , 
                                            gps_struct_t &gps2 , double& firstStamp, int& lidarRotationStartAngle)
{
    currentPacketStart = bufferPacketSize == 0 ? 0 :bufferPacketSize -1 ;

    parseRawData(&bufferPacket[bufferPacketSize++], &packet.data[0], packet.data.size());

    int hasAframe = 0;
    int currentBlockEnd = 0;
    int currentPacketEnd = 0;
    if(bufferPacketSize > 1)
    {
        int lastAzumith = -1;
        for(int i = currentPacketStart ; i < bufferPacketSize ; i++)
        {
            if(hasAframe)
            {
                break;
            }

            int j = 0;
            if (i == currentPacketStart)
            {
                /* code */
                j = lastBlockEnd;
            }
            else
            {
                j = 0;
            }
            for (; j < BLOCKS_PER_PACKET; ++j)
            {
                /* code */
                if(lastAzumith == -1)
                {
                    lastAzumith = bufferPacket[i].blocks[j].azimuth;
                    continue;
                }


                if(lastAzumith > bufferPacket[i].blocks[j].azimuth)
                {
                    if (lidarRotationStartAngle <= bufferPacket[i].blocks[j].azimuth)
                    {
                        // ROS_ERROR("rotation, %d, %d, %d", lastAzumith, bufferPacket[i].blocks[j].azimuth, lidarRotationStartAngle);
                        currentBlockEnd = j;
                        hasAframe = 1;
                        currentPacketEnd = i;
                        break;
                    }

                }
                else if (lastAzumith < lidarRotationStartAngle && lidarRotationStartAngle <= bufferPacket[i].blocks[j].azimuth)
                {
                    // ROS_ERROR("%d, %d, %d", lastAzumith, bufferPacket[i].blocks[j].azimuth, lidarRotationStartAngle);
                    currentBlockEnd = j;
                    hasAframe = 1;
                    currentPacketEnd = i;
                    break;
                }
                lastAzumith = bufferPacket[i].blocks[j].azimuth;
            }
        }
    }

    if(hasAframe)
    {

        int first = 0;
        int j = 0;
        for (int k = 0; k < (currentPacketEnd + 1); ++k)
        {
            if(k == 0)
                j = lastBlockEnd;
            else
                j = 0;

            

            // if > 500ms 
            if(bufferPacket[k].timestamp < 500000 && gps2.used == 0)
            {
                if(gps1 > gps2.gps)
                {
                    ROS_ERROR("Oops , You give me a wrong timestamp I think...");
                }
                gps1 = gps2.gps;
                gps2.used =1;
            }
            else
            {
                if(bufferPacket[k].timestamp < lastTimestamp)
                {
                    int gap = (int)lastTimestamp - (int)bufferPacket[k].timestamp;
                    // avoid the fake jump... wrong udp order
                    if(gap > (10 * 1000)) // 10ms
                    {
                        // Oh , there is a round. But gps2 is not changed , So there is no gps packet!!!
                        // We need to add the offset.
                        
                        gps1 += ((lastTimestamp-20) /1000000) +  1; // 20us offset , avoid the timestamp of 1000002...
                        ROS_ERROR("There is a round , But gps packet!!! , Change gps1 by manual!!! %d %d %d " , gps1 , lastTimestamp , bufferPacket[k].timestamp);
                    }
                    
                }
            }
            int timestamp = bufferPacket[k].timestamp;


            // int gap = timestamp - lastTimestamp;
            // gap = gap <0 ? gap + 1000000 : gap;
            // if(gap > 600)
            // {
            //     ROS_ERROR("gap too large %d-%d=%d"  , timestamp ,lastTimestamp , gap);
            // }
            // ROS_ERROR("timestamp : %d %lf %d"  , timestamp ,ros::Time::now().toSec() , gps1);
            // ROS_ERROR("timestamp of this packe t : %lf" ,(double)gps1 + (((double)bufferPacket[k].timestamp)/1000000));
            for (; j < BLOCKS_PER_PACKET; ++j)
            {
                /* code */
                if (currentBlockEnd == j && k == (currentPacketEnd))
                {
                    break;
                }
                double stamp = 0.0;
                toPointClouds(&bufferPacket[k] , j, pc ,(double)gps1 + (((double)bufferPacket[k].timestamp)/1000000) , stamp);
                if(!first && stamp != 0.0)
                {
                    firstStamp = stamp;
                    first = 1;
                }
                
            } 
            lastTimestamp = bufferPacket[k].timestamp;
        }
        memcpy(&bufferPacket[0] , &bufferPacket[currentPacketEnd] , sizeof(raw_packet_t) * (bufferPacketSize - currentPacketEnd));
        bufferPacketSize = bufferPacketSize - currentPacketEnd;
        lastBlockEnd = currentBlockEnd;
        return 1;
    }
    else
    {
        return 0;
    }
}

int RawData::unpack(const pandar_msgs::PandarScan::ConstPtr &scanMsg, PPointCloud &pc , time_t& gps1 , 
    gps_struct_t &gps2 , double& firstStamp, int& lidarRotationStartAngle)
{
    currentPacketStart = bufferPacketSize == 0 ? 0 :bufferPacketSize -1 ;
    for (int i = 0; i < scanMsg->packets.size(); ++i)
    {
        /* code */
        parseRawData(&bufferPacket[bufferPacketSize++], &scanMsg->packets[i].data[0], scanMsg->packets[i].data.size());
        bufferPacket[bufferPacketSize - 1].recv_time = scanMsg->packets[i].stamp.toSec();

        // int agap = (int)bufferPacket[bufferPacketSize - 1].blocks[0].azimuth - lastAzumith;
        // agap = agap < 0 ? agap + 36000 : agap;
        // if(agap > 130)
        // {
        //     ROS_ERROR("azimuth gap is too large %d-%d=%d" , bufferPacket[bufferPacketSize - 1].blocks[0].azimuth , lastAzumith,agap);
        // }
        // lastAzumith = bufferPacket[bufferPacketSize - 1].blocks[0].azimuth;
    }
    
    // ROS_ERROR("currentPacketStart %d bufferPacketSize %d " , currentPacketStart , bufferPacketSize);
    int hasAframe = 0;
    int currentBlockEnd = 0;
    int currentPacketEnd = 0;
    if(bufferPacketSize > 1)
    {
        int lastAzumith = -1;
        for(int i = currentPacketStart ; i < bufferPacketSize ; i++)
        {
            if(hasAframe)
            {
                break;
            }

            int j = 0;
            if (i == currentPacketStart)
            {
                /* code */
                j = lastBlockEnd;
            }
            else
            {
                j = 0;
            }
            for (; j < BLOCKS_PER_PACKET; ++j)
            {
                /* code */
                if(lastAzumith == -1)
                {
                    lastAzumith = bufferPacket[i].blocks[j].azimuth;
                    continue;
                }


                if(lastAzumith > bufferPacket[i].blocks[j].azimuth)
                {
                    if (lidarRotationStartAngle <= bufferPacket[i].blocks[j].azimuth)
                    {
                        // ROS_ERROR("rotation, %d, %d, %d", lastAzumith, bufferPacket[i].blocks[j].azimuth, lidarRotationStartAngle);
                        currentBlockEnd = j;
                        hasAframe = 1;
                        currentPacketEnd = i;
                        break;
                    }

                }
                else if (lastAzumith < lidarRotationStartAngle && lidarRotationStartAngle <= bufferPacket[i].blocks[j].azimuth)
                {
                    // ROS_ERROR("%d, %d, %d", lastAzumith, bufferPacket[i].blocks[j].azimuth, lidarRotationStartAngle);
                    currentBlockEnd = j;
                    hasAframe = 1;
                    currentPacketEnd = i;
                    break;
                }
                lastAzumith = bufferPacket[i].blocks[j].azimuth;
            }
        }
    }

    if(hasAframe)
    {
#if 0
        for(int i = 0 ; i < LASER_COUNT ; i++)
        {
            if(PandarEnableList[i] == 1)
            {
                int j = 0;
                for (int k = 0; k < (currentPacketEnd + 1); ++k)
                {
                    if(k == 0)
                        j = lastBlockEnd;
                    else
                        j = 0;
                    
                    for (; j < BLOCKS_PER_PACKET; ++j)
                    {
                        /* code */
                        if (currentBlockEnd == j && k == (currentPacketEnd))
                        {
                            break;
                        }
                        toPointClouds(&bufferPacket[k] , i , j, pc);
                        
                    } 
                }
            }
        }
#else
        int first = 0;
        int j = 0;
        for (int k = 0; k < (currentPacketEnd + 1); ++k)
        {
            if(k == 0)
                j = lastBlockEnd;
            else
                j = 0;

            

            // if > 500ms 
            if(bufferPacket[k].timestamp < 500000 && gps2.used == 0)
            {
                if(gps1 > gps2.gps)
                {
                    ROS_ERROR("Oops , You give me a wrong timestamp I think...");
                }
                gps1 = gps2.gps;
                gps2.used =1;
            }
            else
            {
                if(bufferPacket[k].timestamp < lastTimestamp)
                {
                    int gap = (int)lastTimestamp - (int)bufferPacket[k].timestamp;
                    // avoid the fake jump... wrong udp order
                    if(gap > (10 * 1000)) // 10ms
                    {
                        // Oh , there is a round. But gps2 is not changed , So there is no gps packet!!!
                        // We need to add the offset.
                        
                        gps1 += ((lastTimestamp-20) /1000000) +  1; // 20us offset , avoid the timestamp of 1000002...
                        // ROS_ERROR("There is a round , But gps packet!!! , Change gps1 by manual!!! %d %d %d " , gps1 , lastTimestamp , bufferPacket[k].timestamp);
                    }
                    
                }
            }
            int timestamp = bufferPacket[k].timestamp;


            // int gap = timestamp - lastTimestamp;
            // gap = gap <0 ? gap + 1000000 : gap;
            // if(gap > 600)
            // {
            //     ROS_ERROR("gap too large %d-%d=%d"  , timestamp ,lastTimestamp , gap);
            // }
            // ROS_ERROR("timestamp : %d %lf %d"  , timestamp ,ros::Time::now().toSec() , gps1);
            // ROS_ERROR("timestamp of this packe t : %lf" ,(double)gps1 + (((double)bufferPacket[k].timestamp)/1000000));
            for (; j < BLOCKS_PER_PACKET; ++j)
            {
                /* code */
                if (currentBlockEnd == j && k == (currentPacketEnd))
                {
                    break;
                }
                double stamp = 0.0;
                toPointClouds(&bufferPacket[k] , j, pc ,(double)gps1 + (((double)bufferPacket[k].timestamp)/1000000) , stamp);
                if(!first && stamp != 0.0)
                {
                    firstStamp = stamp;
                    first = 1;
                }
                
            } 
            lastTimestamp = bufferPacket[k].timestamp;
        }
#endif
        memcpy(&bufferPacket[0] , &bufferPacket[currentPacketEnd] , sizeof(raw_packet_t) * (bufferPacketSize - currentPacketEnd));
        bufferPacketSize = bufferPacketSize - currentPacketEnd;
        lastBlockEnd = currentBlockEnd;

        // for(int i = 0 ; i < LASER_COUNT ; i++)
        // {
        //     if(PandarEnableList[i] == 1)
        //     {
        //         pc.height++;
        //     }
        // }

        // pc.width /= pc.height;
        // if(pc.width > 1900 || pc.width < 1700)
        // {
        //     ROS_INFO("This fram ");
        // }

        return 1;
    }
    else
    {
        return 0;
    }


#if 0
    raw_packet_t *packets = new raw_packet_t[scanMsg->packets.size()];

    int lastAzumith = -1;
    int currentPacketEnd = 0;
    int currentBlockEnd = 0;
    raw_packet_t *currentPacket = NULL;

    int currentAvariableBlock = 0;

    int foundTheGap = 0;
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
        
        parseRawData(&packets[i], &scanMsg->packets[i].data[0], scanMsg->packets[i].data.size());

        if(foundTheGap)
            continue;

        for(size_t j = 0 ; j < BLOCKS_PER_PACKET ; j++)
        {
            if(lastAzumith == -1)
            {
                lastAzumith = packets[i].blocks[j].azimuth;
                continue;
            }

            if(lastAzumith > packets[i].blocks[j].azimuth)
            {
                // a frame;
                if(i != (scanMsg->packets.size() - 1))
                {
                    if(j ==  (BLOCKS_PER_PACKET-1))
                    {
                        currentPacketEnd = i+1;
                        currentBlockEnd = 0;
                    }
                    else
                    {
                        currentPacketEnd = i;
                        currentBlockEnd = j+1;
                    }

                    currentPacket = packets;
                }
                else
                {
                    if(j ==  (BLOCKS_PER_PACKET-1))
                    {
                        currentPacket = NULL;
                        currentPacketEnd = 0;
                        currentBlockEnd = 0;
                    }
                    else
                    {
                        currentPacketEnd = i;
                        currentBlockEnd = j+1;
                        currentPacket = packets;
                    }
                }
                foundTheGap = 1;
                break;
            }

            currentAvariableBlock++;
        }
    }

    // ROS_ERROR("dddd  %d %d " , currentPacketEnd , currentBlockEnd);
#if 1
    if(lastPacket)
    {
        int first = 1;
        int j = 0;

        for(int i = 0 ; i < LASER_COUNT ; i++)
        {
            if(PandarEnableList[i] == 1)
            {
                for (int k = lastPacketEnd; k < lastPacketCounter; ++k)
                {
                    if(k == lastPacketEnd)
                        j = lastBlockEnd;
                    else
                        j = 0;
                    
                    for (; j < BLOCKS_PER_PACKET; ++j)
                    {
                        /* code */
                        toPointClouds(&lastPacket[k] , i , j, pc);
                        pc.width++;
                    } 
                }
            }
        }
    }
#endif

    // // if(lastPacket && packets[0].blocks[0].azimuth < lastPacket[lastPacketCounter-1].blocks[BLOCKS_PER_PACKET -1].azimuth)
    // // {
    // // }
    // else
    {
        for(int i = 0 ; i < LASER_COUNT ; i++)
        {
            if(PandarEnableList[i] != 1)
            {
                continue;
            }
            for (int k = 0; k <= currentPacketEnd; ++k)
            {
                for (int j = 0; j < BLOCKS_PER_PACKET; ++j)
                {
                    if(k == currentPacketEnd && j == currentBlockEnd)
                        break;
                    /* code */
                    toPointClouds(&packets[k] , i , j, pc);
                    pc.width++;
                }
                
            }
        }
    }


#if 1

    for(int i = 0 ; i < LASER_COUNT ; i++)
    {
        if(PandarEnableList[i] == 1)
        {
            pc.height++;
        }
    }

    pc.width /= pc.height;
    if(pc.width > 1900 || pc.width < 1700)
    {
        ROS_ERROR("SSS %d %d  , %d %d " , currentPacketEnd , currentBlockEnd , lastPacketEnd ,lastBlockEnd);
    }

    if(lastPacket)
    {
        delete [] lastPacket;
        lastPacket = NULL;
    }

    if(currentPacket)
    {
        lastPacket = currentPacket;
        lastPacketEnd = currentPacketEnd;
        lastBlockEnd = currentBlockEnd;
        lastPacketCounter = scanMsg->packets.size();
    }

#else
    lastPacket = NULL;
    delete [] packets;
#endif
#endif
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const pandar_msgs::PandarPacket &pkt, PPointCloud &pc)
{
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

	raw_packet_t packet;
	parseRawData(&packet, &pkt.data[0], pkt.data.size());
	toPointClouds(&packet, pc);
}

} // namespace pandar_rawdata
