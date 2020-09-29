/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Pandar40 3D LIDAR:
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
int InputSocket::getPacket(pandar_msgs::PandarPacket *pkt) {
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

  static uint32_t dropped = 0, u32StartSeq = 0;
  static uint32_t startTick = GetTickCount();

  uint32_t *pSeq = (uint32_t *)&pkt->data[nbytes - 4];
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

  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  // double time2 = ros::Time::now().toSec();
  // pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);
  if (isgps) {
    return 2;
  }
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
}

/** destructor */
InputPCAP::~InputPCAP(void) { pcap_close(pcap_); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputPCAP::getPacket(pandar_msgs::PandarPacket *pkt) {
  struct pcap_pkthdr *header;
  const u_char *pkt_data;

  while (true) {  // ROS_WARN("InputPCAP::getPacket");
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
      // Skip packets not for the correct port and from the
      // selected IP address.
      // if (!devip_str_.empty() &&
      //     (0 == pcap_offline_filter(&pcap_packet_filter_,
      //                               header, pkt_data)))
      //   {ROS_WARN("devip_str_ &&
      //   pcap_offline_filter[%s]",devip_str_.c_str());
      //     continue;
      //   }

      // Keep the reader from blowing through the file.
      if (read_fast_ == false) packet_rate_.sleep();

      memcpy(&pkt->data[0], pkt_data + 42, packet_size);
      pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
                                      // synchronization required
      empty_ = false;
      if (header->caplen == (512 + 42)) {
        // ROS_ERROR("GPS");
        return 2;
      }

      else if (header->caplen == (812 + 42)) {
        return 0;  // success
      }

      // Wrong data , It's not the packet of LiDAR I think.
      continue;
    }

    if (empty_)  // no data in file?
    {
      // ROS_WARN("Error %d reading Pandar packet: %s",
      //  res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_) {
      ROS_WARN("end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0) {
      ROS_WARN("end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    ROS_DEBUG("replaying Pandar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again
}

}  // pandar namespace
