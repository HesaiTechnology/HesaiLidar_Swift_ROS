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

    @author Jack O'Quin
    @author Jesse Vera
    @author Yang Sheng

*/

#include "transform.h"

#include <pcl_conversions/pcl_conversions.h>


namespace pandar_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new pandar_rawdata::RawData()),
    m_spConver(new Convert(node, private_nh,"transform"))
  {
    m_input.reset(new pandar_pointcloud::InputSocket(private_nh, 2368));
    ROS_WARN(" Transform::Transform");
    private_nh.getParam("frame_id", config_.frame_id);
    // Read calibration.
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    // output_ =
    //   node.advertise<sensor_msgs::PointCloud2>("transform_pandar_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
      TransformNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<pandar_pointcloud::TransformNodeConfig>::
      CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);
    
    // subscribe to PandarScan packets using transform filter
    
    pandar_scan_.subscribe(node, "pandar_packets", 10);
    tf_filter_ =
      new tf::MessageFilter<pandar_msgs::PandarScan>(pandar_scan_,
                                                         listener_,
                                                         config_.frame_id, 10);
    tf_filter_->registerCallback(boost::bind(&Transform::processScan, this, _1));
    ROS_WARN(" Transform::processScan config[%s]",config_.frame_id.c_str());
    ROS_WARN(" Transform::Transform finisher");
    m_driverReadThread = new boost::thread(boost::bind(&Transform::driverReadThread, this));
    
  }
  
  void Transform::reconfigure_callback(
      pandar_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range, 
                         config.view_direction, config.view_width);
    config_.frame_id = tf::resolve(tf_prefix_, config.frame_id);
    ROS_INFO_STREAM("Target frame ID: " << config_.frame_id);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg)
  {
    // process each packet provided by the driver
    // ROS_WARN(" Transform::processScan");
    for (size_t next = 0; next < scanMsg->packets.size(); ++next) {
        m_packetBuffer.push(scanMsg->packets[next]);
        
        }
  }

  void Transform::driverReadThread(){
    pandar_msgs::PandarPacket raw_packet;
    struct tm t;
    int64_t last_pkt_ts = 0;
    int64_t last_time = 0;
    int64_t current_time = 0;
    int64_t pkt_ts = 0;
    int sleep_count = 0;
    while(1){
      if(m_packetBuffer.size() > 0){
        raw_packet = m_packetBuffer.front();
        m_packetBuffer.pop();
        PandarPacket packet;
        while(m_spConver->getIsSocketTimeout()){
          usleep(10000);
        }
        packet.stamp = raw_packet.stamp;
        packet.size = raw_packet.size;
        memcpy(&packet.data[0], &raw_packet.data[0], raw_packet.size);
        if(m_input->checkPacketSize(&packet)){
          m_spConver->pushLiDARData(raw_packet);
        }
        


        uint32_t unix_second = 0;      
        if(raw_packet.data[m_input->m_iUtcIindex] != 0){
          struct tm t = {0};
          t.tm_year  = raw_packet.data[m_input->m_iUtcIindex];
          if (t.tm_year >= 200) {
            t.tm_year -= 100;
          }
          t.tm_mon   = raw_packet.data[m_input->m_iUtcIindex + 1] - 1;
          t.tm_mday  = raw_packet.data[m_input->m_iUtcIindex + 2];
          t.tm_hour  = raw_packet.data[m_input->m_iUtcIindex + 3];
          t.tm_min   = raw_packet.data[m_input->m_iUtcIindex + 4];
          t.tm_sec   = raw_packet.data[m_input->m_iUtcIindex + 5];
          t.tm_isdst = 0;

          unix_second = mktime(&t);
        }
        else{
          uint32_t utc_time_big = *(uint32_t*)(&raw_packet.data[m_input->m_iUtcIindex] + 2);
          unix_second = ((utc_time_big >> 24) & 0xff) |
                        ((utc_time_big >> 8) & 0xff00) |
                        ((utc_time_big << 8) & 0xff0000) |
                        ((utc_time_big << 24));
        }
        pkt_ts = unix_second * 1000000 + ((raw_packet.data[m_input->m_iTimestampIndex]& 0xff) | \
            (raw_packet.data[m_input->m_iTimestampIndex + 1]& 0xff) << 8 | \
            ((raw_packet.data[m_input->m_iTimestampIndex + 2]& 0xff) << 16) | \
            ((raw_packet.data[m_input->m_iTimestampIndex + 3]& 0xff) << 24)); 
        struct timeval sys_time;
        gettimeofday(&sys_time, NULL);
        current_time = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

        if (0 == last_pkt_ts) {
          last_pkt_ts = pkt_ts;
          last_time = current_time;
        } else {
          int64_t sleep_time = (pkt_ts - last_pkt_ts) - \
              (current_time - last_time);
              // ROS_WARN("pkt time: %u,use time: %u,sleep time: %d",pkt_ts - last_pkt_ts,current_time - last_time, sleep_time);
          if(((pkt_ts - last_pkt_ts) % 1000000) > 10000 && (sleep_count == 0)){
              sleep_count += 1;
              m_spConver->setIsSocketTimeout(true);
            }
            else{
              if(sleep_count != 1)
                m_spConver->setIsSocketTimeout(false);
              sleep_count = 0;
              
            }
          if (sleep_time > 0) {
            struct timeval waitTime;
            waitTime.tv_sec = sleep_time / 1000000;
            waitTime.tv_usec = sleep_time % 1000000;
            
            

            int err;

            do {
              err = select(0, NULL, NULL, NULL, &waitTime);
            } while (err < 0 && errno != EINTR);
          }

          last_pkt_ts = pkt_ts;
          last_time = current_time;
          last_time += sleep_time;
        }
      }
      else{
        // ROS_WARN("%d   %d",m_spConver->getIsSocketTimeout(), m_packetBuffer.size());
        usleep(1000);
      }
    }
  }
} // namespace pandar_pointcloud
