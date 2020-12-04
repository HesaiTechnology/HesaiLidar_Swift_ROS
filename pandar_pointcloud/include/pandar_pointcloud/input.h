/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Pandar128 3D LIDAR data input classes
 *
 *    These classes provide raw Pandar128 LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     pandar::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     pandar::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     pandar::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include <pandar_msgs/PandarPacket.h>



namespace pandar_pointcloud
{
  static uint16_t DATA_PORT_NUMBER = 8080;     // default data port
  static uint16_t POSITION_PORT_NUMBER = 8308; // default position port

  #define PANDAR128_SEQUENCE_NUMBER_OFFSET (831) 
  /** @brief pandar input base class */
  class Input
  {
  public:
    Input(ros::NodeHandle private_nh, uint16_t port);
    virtual ~Input() {}

    /** @brief Read one pandar packet.
     *
     * @param pkt points to pandarPacket message
     *
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    virtual int getPacket(pandar_msgs::PandarPacket *pkt) = 0;

  protected:
    ros::NodeHandle private_nh_;
    uint16_t port_;
    std::string devip_str_;
  };

  /** @brief Live pandar input from socket. */
  class InputSocket: public Input
  {
  public:
    InputSocket(ros::NodeHandle private_nh,
                uint16_t port = DATA_PORT_NUMBER);
    virtual ~InputSocket();

    virtual int getPacket(pandar_msgs::PandarPacket *pkt);
    void setDeviceIP( const std::string& ip );
  private:

  private:
    int sockfd_;
    in_addr devip_;
    uint32_t m_u32Sequencenum;
    uint8_t seqnub1;
    uint8_t seqnub2;
    uint8_t seqnub3;
    uint8_t seqnub4;

  };


  /** @brief pandar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap, pandar's DSR software,
   * ethereal, wireshark, tcpdump, or the \ref vdump_command.
   */
  class InputPCAP: public Input
  {
  public:
    InputPCAP(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER,
              double packet_rate = 0.0,
              std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0);
    virtual ~InputPCAP();

    virtual int getPacket(pandar_msgs::PandarPacket *pkt);
    void setDeviceIP( const std::string& ip );

  private:
    ros::Rate packet_rate_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
    int ts_index;
    int utc_index;
    int gap;
    int64_t last_pkt_ts;
    int count;
    int64_t last_time;
    int64_t current_time;
    int64_t pkt_ts;
  };

} // pandar_driver namespace

#endif // __PANDAR_INPUT_H
