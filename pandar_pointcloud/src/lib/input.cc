/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Pandar128 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <pandar_pointcloud/input.h>
#include <pandar_pointcloud/platUtil.h>
#include <poll.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <string>

namespace pandar_pointcloud {
static const size_t packet_size = sizeof(pandar_msgs::PandarPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(ros::NodeHandle private_nh, uint16_t port)
    : private_nh_(private_nh), port_(port) {
  private_nh.param("device_ip", devip_str_, std::string(""));
  if (!devip_str_.empty())
    ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
}

bool Input::checkPacketSize(PandarPacket *pkt) {
  if(pkt->size < 100)
  return false;
  uint8_t laserNum = pkt->data[6];
  uint8_t blockNum = pkt->data[7];
  uint8_t flags = pkt->data[11];

  bool hasSeqNum = (flags & 1); 
  bool hasImu = (flags & 2);
  bool hasFunctionSafety = (flags & 4);
  bool hasSignature = (flags & 8);
  bool hasConfidence = (flags & 0x10);

  uint32_t size = 12 + 
            (hasConfidence ? 4 * laserNum * blockNum : 3 * laserNum * blockNum) + 
            2 * blockNum + 4 +
            (hasFunctionSafety ? 17 : 0) + 
            26 + 
            (hasImu ? 22 : 0) + 
            (hasSeqNum ? 4 : 0) + 4 +
            (hasSignature ? 32 : 0);
  if(pkt->size == size){
    if(size == 893 || size == 861){
      return true;
    }
    else{
      ROS_WARN("Don't support to parse packet with size %d", size);
      return false;
    }
  }
  else{
    ROS_WARN("Packet size mismatch.caculated size:%d, packet size:%d", size, pkt->size);
    return false;
  }
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number
 */
InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port)
    : Input(private_nh, port) {
  sockfd_ = -1;
  m_u32Sequencenum = 0;
  seqnub1 = 0;
  seqnub2 = 0;
  seqnub3 = 0;
  seqnub4 = 0;

  if (!devip_str_.empty()) {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  // connect to Pandar UDP port
  ROS_INFO_STREAM("Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO: ROS_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }

  int getchecksum = 0;
  socklen_t option_int = sizeof(int);
  int get_error =
      getsockopt(sockfd_, SOL_SOCKET, SO_NO_CHECK, &getchecksum, &option_int);
  int nochecksum = 1;
  int set_error = setsockopt(sockfd_, SOL_SOCKET, SO_NO_CHECK, &nochecksum,
                             sizeof(nochecksum));
  ROS_DEBUG("Pandar socket fd is %d\n", sockfd_);
}

/** @brief destructor */
InputSocket::~InputSocket(void) { (void)close(sockfd_); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputSocket::getPacket(PandarPacket *pkt) {
  // double time1 = ros::Time::now().toSec();

  uint64_t startTime = 0;
  uint64_t endTime = 0;
  uint64_t midTime = 0;
  uint32_t seqnub;
  timespec time;
  memset(&time, 0, sizeof(time));

  int isgps = 0;
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 20;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  clock_gettime(CLOCK_REALTIME, &time);
  startTime = time.tv_nsec / 1000 + time.tv_sec * 1000000;

  int retval = poll(fds, 1, POLL_TIMEOUT);

  uint64_t midTime2;
  clock_gettime(CLOCK_REALTIME, &time);
  midTime2 = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  if (midTime2 - startTime > 1000) {
    // ROS_WARN("InputSocket::getPacket poll time:[%u]",midTime2-startTime);
  }

  //  ROS_WARN("InputSocket::getPacket retval[%d]",retval);
  if (retval < 0)  // poll() error?
  {
    if (errno != EINTR) ROS_WARN("poll() error: %s", strerror(errno));
    return 1;
  }
  if (retval == 0)  // poll() timeout?
  {
    ROS_WARN("Pandar poll() timeout");
    return 1;
  }
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
      (fds[0].revents & POLLNVAL))  // device error?
  {
    ROS_WARN("poll() reports Pandar error");
    return 1;
  }
  // } while ((fds[0].revents & POLLIN) == 0);

  ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], 10000, 0,
                            (sockaddr *)&sender_address, &sender_address_len);
  pkt->size = nbytes;
  if (pkt->size == 512) {
    // ROS_ERROR("GPS");
    return 2;
  }
  else if(!checkPacketSize(pkt)){
    return 1;  // Packet size not match
  }

  static uint32_t dropped = 0, u32StartSeq = 0;
  static uint32_t startTick = GetTickCount();
  if(pkt->data[11]& 1){    //Packet has UDP sequence number
    uint32_t *pSeq = (uint32_t *)&pkt->data[PANDAR128_SEQUENCE_NUMBER_OFFSET];
    seqnub = *pSeq;

    if (m_u32Sequencenum == 0) {
      m_u32Sequencenum = seqnub;
      u32StartSeq = m_u32Sequencenum;
    } else {
      uint32_t diff = seqnub - m_u32Sequencenum;
      if (diff > 1) {
        ROS_WARN("seq diff: %x ", diff);
        dropped += diff - 1;
      }
    }
    m_u32Sequencenum = seqnub;

    uint32_t endTick = GetTickCount();

    if (endTick - startTick >= 1000 && dropped > 0) {
      ROS_WARN("!!!!!!!!!! dropped: %d, %d, percent, %f", dropped,
              m_u32Sequencenum - u32StartSeq,
              float(dropped) / float(m_u32Sequencenum - u32StartSeq) * 100.0);
      dropped = 0;
      u32StartSeq = m_u32Sequencenum;
      startTick = endTick;
    }
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  // double time2 = ros::Time::now().toSec();
  // pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);
  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number
 *  @param packet_rate expected device packet frequency (Hz)
 *  @param filename PCAP dump file name
 */
InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port,
                     double packet_rate, std::string filename, bool read_once,
                     bool read_fast, double repeat_delay)
    : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename) {
  pcap_ = NULL;
  empty_ = true;

  // get parameters using private node handle
  private_nh.param("read_once", read_once_, false);
  private_nh.param("read_fast", read_fast_, false);
  private_nh.param("repeat_delay", repeat_delay_, 0.0);

  if (read_once_) ROS_INFO("Read input file only once.");
  if (read_fast_) ROS_INFO("Read input file as quickly as possible.");
  if (repeat_delay_ > 0.0)
    ROS_WARN("Delay %.3f seconds before repeating input file.", repeat_delay_);

  // Open the PCAP dump file
  ROS_WARN("Opening PCAP file \"%s\"", filename_.c_str());
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
    ROS_FATAL("Error opening Pandar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1,
               PCAP_NETMASK_UNKNOWN);
  ts_index = 826;
  utc_index = 820;
  gap = 1000;
  last_pkt_ts = 0;
  count;
  last_time = 0;
  current_time = 0;
  pkt_ts = 0;
}

/** destructor */
InputPCAP::~InputPCAP(void) { pcap_close(pcap_); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputPCAP::getPacket(PandarPacket *pkt) {
  pcap_pkthdr *pktHeader;
  const unsigned char *packetBuf;
  struct tm t;

  if(pcap_next_ex(pcap_, &pktHeader, &packetBuf) >= 0) {
    const uint8_t *packet = packetBuf + 42;
    memcpy(&pkt->data[0], packetBuf + 42, pktHeader->caplen -42);
    pkt->size = pktHeader->caplen -42;
    count++;
    if (pktHeader->caplen == (512 + 42)) {
      // ROS_ERROR("GPS");
      return 2;
    }
    else if(!checkPacketSize(pkt)){
      return 1;  // Packet size not match
    }
    if (count >= gap) {
      count = 0;

      t.tm_year  = packet[utc_index];
      t.tm_mon   = packet[utc_index+1] - 1;
      t.tm_mday  = packet[utc_index+2];
      t.tm_hour  = packet[utc_index+3];
      t.tm_min   = packet[utc_index+4];
      t.tm_sec   = packet[utc_index+5];
      t.tm_isdst = 0;

      pkt_ts = mktime(&t) * 1000000 + ((packet[ts_index]& 0xff) | \
          (packet[ts_index+1]& 0xff) << 8 | \
          ((packet[ts_index+2]& 0xff) << 16) | \
          ((packet[ts_index+3]& 0xff) << 24));
      struct timeval sys_time;
      gettimeofday(&sys_time, NULL);
      current_time = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

      if (0 == last_pkt_ts) {
        last_pkt_ts = pkt_ts;
        last_time = current_time;
      } else {
        int64_t sleep_time = (pkt_ts - last_pkt_ts) - \
            (current_time - last_time);
            // ROS_WARN("pkt time: %u,use time: %u,sleep time: %u",pkt_ts - last_pkt_ts,current_time - last_time, sleep_time);

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

    pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
                                      // synchronization required
    // if (pcapFile != NULL) {
    //   pcap_close(pcapFile);
    //   pcapFile = NULL;
    // }
    return 0;
  }
  return 1;
}

}  // pandar namespace
