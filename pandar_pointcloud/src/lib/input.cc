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
static const size_t packet_size = sizeof(PandarPacket().data);

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
  m_iTimestampIndex = 0;
  m_iUtcIindex = 0;
  m_iSequenceNumberIndex = 0;
  m_bGetUdpVersion = false;
}

bool Input::checkPacketSize(PandarPacket *pkt) {
  if(pkt->size < 500){
	  return false;
  }
  
  if (pkt->data[0] != 0xEE && pkt->data[1] != 0xFF) {    
    ROS_WARN("Packet with invaild delimiter %x %x %d", pkt->data[0], pkt->data[1], pkt->size);
    return false;
  }
  if (pkt->data[2] != 4 || (pkt->data[3] != 1 && pkt->data[3] != 3)) {    
    ROS_WARN("Packet with invaild lidar type");
    return false;
  }
  uint8_t laserNum = pkt->data[6];
  uint8_t blockNum = pkt->data[7];
  uint8_t flags = pkt->data[11];

  uint8_t UDPMinorVersion = pkt->data[3];

  bool hasSeqNum = (flags & 1); 
  bool hasImu = (flags & 2);
  bool hasFunctionSafety = (flags & 4);
  bool hasSignature = (flags & 8);
  bool hasConfidence = (flags & 0x10);

  if(UDPMinorVersion == UDP_VERSION_MINOR_1){
	m_iTimestampIndex = PANDAR_AT128_HEAD_SIZE +
                      PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * laserNum * blockNum + 
                      PANDAR_AT128_AZIMUTH_SIZE * blockNum +
                      PANDAR_AT128_TAIL_RESERVED1_SIZE + 
                      PANDAR_AT128_TAIL_RESERVED2_SIZE +
                      PANDAR_AT128_SHUTDOWN_FLAG_SIZE +
                      PANDAR_AT128_TAIL_RESERVED3_SIZE +
                      PANDAR_AT128_MOTOR_SPEED_SIZE;
	m_iUtcIindex = m_iTimestampIndex + PANDAR_AT128_TS_SIZE +
                  PANDAR_AT128_RETURN_MODE_SIZE +
                  PANDAR_AT128_FACTORY_INFO;
	m_iSequenceNumberIndex = m_iUtcIindex + PANDAR_AT128_UTC_SIZE;
  }
  else{
		m_iTimestampIndex = PANDAR_AT128_HEAD_SIZE +
                        (hasConfidence ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * laserNum * blockNum : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE * laserNum * blockNum)+ 
                        PANDAR_AT128_AZIMUTH_SIZE * blockNum +
                        PANDAR_AT128_FINE_AZIMUTH_SIZE * blockNum +
                        (UDPMinorVersion == 3 ? PANDAR_AT128_CRC_SIZE : 0) +
                        (hasFunctionSafety ? PANDAR_AT128_FUNCTION_SAFETY_SIZE : 0) + 
                        PANDAR_AT128_TAIL_RESERVED1_SIZE + 
                        PANDAR_AT128_TAIL_RESERVED2_SIZE +
                        PANDAR_AT128_SHUTDOWN_FLAG_SIZE +
                        PANDAR_AT128_TAIL_RESERVED3_SIZE +
                        PANDAR_AT128_TAIL_RESERVED4_SIZE +
                        PANDAR_AT128_MOTOR_SPEED_SIZE;
		m_iUtcIindex = m_iTimestampIndex + PANDAR_AT128_TS_SIZE +
                    PANDAR_AT128_RETURN_MODE_SIZE +
                    PANDAR_AT128_FACTORY_INFO;
		m_iSequenceNumberIndex = m_iUtcIindex + PANDAR_AT128_UTC_SIZE;

  }

  uint32_t size = m_iSequenceNumberIndex + (hasSeqNum ? PANDAR_AT128_SEQ_NUM_SIZE  : 0) +
                (UDPMinorVersion == 3 ? PANDAR_AT128_CRC_SIZE : 0)+
                (hasSignature ? PANDAR_AT128_SIGNATURE_SIZE : 0);
  if(pkt->size == size){
    return true;
  }
  else{
    ROS_WARN("Packet size mismatch.caculated size:%d, packet size:%d", size, pkt->size);
    return false;
  }
}

void Input::setUdpVersion(uint8_t major, uint8_t minor) {
	if(UDP_VERSION_MAJOR_1 == major) {
		if(UDP_VERSION_MINOR_3 == minor) {
			m_sUdpVresion = UDP_VERSION_1_3;
			m_iTimestampIndex = udpVersion13[TIMESTAMP_INDEX];
			m_iUtcIindex = udpVersion13[UTC_INDEX];
			m_iSequenceNumberIndex = udpVersion13[SEQUENCE_NUMBER_INDEX];
			// m_iPacketSize = udpVersion13[PACKET_SIZE];
			m_bGetUdpVersion = true;
			return;
		}
		if(UDP_VERSION_MINOR_4 == minor) {
			m_sUdpVresion = UDP_VERSION_1_4;
			m_iTimestampIndex = udpVersion14[TIMESTAMP_INDEX];
			m_iUtcIindex = udpVersion14[UTC_INDEX];
			m_iSequenceNumberIndex = udpVersion14[SEQUENCE_NUMBER_INDEX];
			// m_iPacketSize = udpVersion14[PACKET_SIZE];
			m_bGetUdpVersion = true;
			return;
		}
		printf("error udp version minor: %d\n", minor);
	}
	else if(UDP_VERSION_MAJOR_4 == major){
		if(UDP_VERSION_MINOR_1 == minor) {
			m_sUdpVresion = UDP_VERSION_4_1;
			m_bGetUdpVersion = true;
			printf("UDP version is : %d.%d\n", major, minor);
			return;
		}
		if(UDP_VERSION_MINOR_3 == minor) {
			m_sUdpVresion = UDP_VERSION_4_3;
			m_bGetUdpVersion = true;
			printf("UDP version is : %d.%d\n", major, minor);
			return;
		}
		printf("error udp version minor: %d\n", minor);

	}
	else{
		printf("error udp version major: %d\n", major);
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
InputSocket::InputSocket(ros::NodeHandle private_nh, std::string host_ip, uint16_t port, uint16_t gpsport, std::string multicast_ip)
    : Input(private_nh, port) {
	m_iSockfd = -1;
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
	m_iSockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if (m_iSockfd == -1) {
		perror("socket");  // TODO: ROS_ERROR errno
		return;
	}

	sockaddr_in my_addr;                   // my address information
	memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
	my_addr.sin_family = AF_INET;          // host byte order
	my_addr.sin_port = htons(port);        // port in network byte order
	my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

	if (bind(m_iSockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
		perror("bind");  // TODO: ROS_ERROR errno
		return;
	}

	if (fcntl(m_iSockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		perror("non-block");
		return;
	}

	int getchecksum = 0;
	socklen_t option_int = sizeof(int);
	int get_error =
		getsockopt(m_iSockfd, SOL_SOCKET, SO_NO_CHECK, &getchecksum, &option_int);
	int nochecksum = 1;
	int set_error = setsockopt(m_iSockfd, SOL_SOCKET, SO_NO_CHECK, &nochecksum,
								sizeof(nochecksum));
	int nRecvBuf = 400000000;
	setsockopt(m_iSockfd, SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBuf, sizeof(int));
	int curRcvBufSize = -1;
	socklen_t optlen = sizeof(curRcvBufSize);
	if (getsockopt(m_iSockfd, SOL_SOCKET, SO_RCVBUF, &curRcvBufSize, &optlen) < 0)
	{
		ROS_WARN("getsockopt error=%d(%s)!!!\n", errno, strerror(errno));
	}
	ROS_WARN("OS current udp socket recv buff size is: %d\n", curRcvBufSize);
	ROS_WARN("Pandar socket fd is %d\n", m_iSockfd);
	m_iSocktNumber = 1;
	if(multicast_ip != ""){
    struct ip_mreq mreq;                    
    mreq.imr_multiaddr.s_addr=inet_addr(multicast_ip.c_str());
    mreq.imr_interface.s_addr = host_ip == "" ? htons(INADDR_ANY) : inet_addr(host_ip.c_str());
    int ret = setsockopt(m_iSockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq));
    if (ret < 0) {
      printf("Multicast IP error,set correct multicast ip address or keep it empty %s %s\n", multicast_ip.c_str(), host_ip.c_str());
    } 
    else {
      printf("Recive data from multicast ip address %s %s\n", multicast_ip.c_str(), host_ip.c_str());
    }
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void) { (void)close(m_iSockfd); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
PacketType InputSocket::getPacket(PandarPacket *pkt, bool &isTimeout, bool& skipSleep) {
	uint64_t startTime = 0;
	uint64_t endTime = 0;
	uint64_t midTime = 0;
	timespec time;
	memset(&time, 0, sizeof(time));

	struct pollfd fds[m_iSocktNumber];
	if(m_iSocktNumber == 2) {
		fds[0].fd = m_iSockGpsfd;
		fds[0].events = POLLIN;

		fds[1].fd = m_iSockfd;
		fds[1].events = POLLIN;
	} 
	else if(m_iSocktNumber == 1) {
		fds[0].fd = m_iSockfd;
		fds[0].events = POLLIN;
	}
	static const int POLL_TIMEOUT = 1;  // one second (in msec)
	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);
	if(skipSleep){
		int retval = poll(fds, m_iSocktNumber, POLL_TIMEOUT);
		if(retval < 0) { // poll() error?
			if(errno != EINTR) printf("poll() error: %s\n", strerror(errno));
			return ERROR_PACKET;
		}
		if(retval == 0) { // poll() timeout?
			// printf("timeout\n");
			isTimeout = true;
			return ERROR_PACKET;
		}
		if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||(fds[0].revents & POLLNVAL)) { // device error?
			printf("poll() reports Pandar error\n");
			return ERROR_PACKET;
		}
	}
	ssize_t nbytes;
	nbytes = recvfrom(m_iSockfd, &pkt->data[0], 10000, 0, (sockaddr *)&sender_address, &sender_address_len);
	pkt->size = nbytes;
	skipSleep = false;
	if(nbytes == -1){
		skipSleep = true;
		return ERROR_PACKET;
	}
	if (pkt->size == 512) {
		// ROS_ERROR("GPS");
		return GPS_PACKET;
	}
	if (pkt->size == FAULT_MESSAGE_PCAKET_SIZE) {
		return FAULT_MESSAGE_PACKET;
	}
	if (pkt->size == LOG_REPORT_PCAKET_SIZE) {
		return LOG_REPORT_PACKET;
	}
	if(!m_bGetUdpVersion) {
		return POINTCLOUD_PACKET;
	}
	else if(!checkPacketSize(pkt)){
		return ERROR_PACKET;  // Packet size not match
	}
	isTimeout = false;
	calcPacketLoss(pkt);
	return POINTCLOUD_PACKET;
}

void InputSocket::calcPacketLoss(PandarPacket *pkt) {
	if(m_bGetUdpVersion) {
		if(m_sUdpVresion == UDP_VERSION_1_4 && !(pkt->data[11] & 1))
			return;
		static uint32_t dropped = 0, u32StartSeq = 0;
		static uint32_t startTick = GetTickCount();
		uint32_t *pSeq = (uint32_t *)&pkt->data[m_iSequenceNumberIndex];
		uint32_t seqnub = *pSeq;
		// printf("index: %d,%d, seq: %d\n", nbytes, m_iSequenceNumberIndex, seqnub);

		if(m_u32Sequencenum == 0) {
			m_u32Sequencenum = seqnub;
			u32StartSeq = m_u32Sequencenum;
		} 
		else {
			uint32_t diff = seqnub - m_u32Sequencenum;
			if(diff > 1) {
				ROS_WARN("seq diff: %x  current seq: %x  last seq: %x", diff, seqnub, m_u32Sequencenum);
				dropped += diff - 1;
			}
		}
		m_u32Sequencenum = seqnub;
		uint32_t endTick = GetTickCount();
		if(endTick - startTick >= 1000 && dropped > 0 && (m_u32Sequencenum - u32StartSeq) > 0) {
			ROS_WARN("dropped: %u, received: %u, percentage: %f", dropped, m_u32Sequencenum - u32StartSeq,
					float(dropped) / float(m_u32Sequencenum - u32StartSeq) * 100.0);	
			dropped = 0;
			u32StartSeq = m_u32Sequencenum;
			startTick = endTick;
		}
	}
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
  m_pcapt = NULL;
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
  if ((m_pcapt = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
    ROS_FATAL("Error opening Pandar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(m_pcapt, &pcap_packet_filter_, filter.str().c_str(), 1,
               PCAP_NETMASK_UNKNOWN);
  m_iTimeGap = 0;
	m_i64LastPktTimestamp = 0;
	m_iPktCount = 0;
	m_i64LastTime = 0;
	m_i64CurrentTime = 0;
	m_i64PktTimestamp = 0;
}

/** destructor */
InputPCAP::~InputPCAP(void) { pcap_close(m_pcapt); }


PacketType InputPCAP::getPacket(PandarPacket *pkt, bool &isTimeout, bool& skipSleep) {
	skipSleep = false;
	pcap_pkthdr *pktHeader;
	const unsigned char *packetBuf;

	if(pcap_next_ex(m_pcapt, &pktHeader, &packetBuf) >= 0) {
		const uint8_t *packet = packetBuf + 42;
		memcpy(&pkt->data[0], packetBuf + 42, packet_size);
		pkt->size = pktHeader->caplen - 42;
		m_iPktCount++;
		
		if (pktHeader->caplen == (512 + 42)) {
			return GPS_PACKET;
		}
		else if (pktHeader->caplen == (FAULT_MESSAGE_PCAKET_SIZE + 42)) {
			return FAULT_MESSAGE_PACKET;
		}
		else if (pktHeader->caplen == (LOG_REPORT_PCAKET_SIZE + 42)) {
			return LOG_REPORT_PACKET;
		}	
		else if(!checkPacketSize(pkt)){
			return ERROR_PACKET;  // error packet
		}
		if( (m_iPktCount >= m_iTimeGap)) {
			sleep(packet, isTimeout);
		}
		pkt->stamp = ros::Time::now();  // time_offset not considered here, as no synchronization required
		return POINTCLOUD_PACKET;  // success
	}
	return PCAP_END_PACKET;
}

void InputPCAP::sleep(const uint8_t *packet, bool &isTimeout) {
	static int sleep_count = 0;
	m_iPktCount = 0;
	uint32_t unix_second = 0;      
	if(packet[m_iUtcIindex] != 0){
		struct tm t = {0};
		t.tm_year  = packet[m_iUtcIindex];
		if (t.tm_year >= 200) {
			t.tm_year -= 100;
		}
		t.tm_mon   = packet[m_iUtcIindex+1] - 1;
		t.tm_mday  = packet[m_iUtcIindex+2];
		t.tm_hour  = packet[m_iUtcIindex+3];
		t.tm_min   = packet[m_iUtcIindex+4];
		t.tm_sec   = packet[m_iUtcIindex+5];
		t.tm_isdst = 0;

		unix_second = mktime(&t);
	}
	else{
	
		uint32_t utc_time_big = *(uint32_t*)(&packet[m_iUtcIindex] + 2);
		unix_second = ((utc_time_big >> 24) & 0xff) |
						((utc_time_big >> 8) & 0xff00) |
						((utc_time_big << 8) & 0xff0000) |
						((utc_time_big << 24));            
	}
	m_i64PktTimestamp = unix_second * 1000000 + ((packet[m_iTimestampIndex]& 0xff) | \
		(packet[m_iTimestampIndex+1]& 0xff) << 8 | \
		((packet[m_iTimestampIndex+2]& 0xff) << 16) | \
		((packet[m_iTimestampIndex+3]& 0xff) << 24));	
	struct timeval sys_time;
	gettimeofday(&sys_time, NULL);
	m_i64CurrentTime = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

	if(0 == m_i64LastPktTimestamp) {
		m_i64LastPktTimestamp = m_i64PktTimestamp;
		m_i64LastTime = m_i64CurrentTime;
	} 
	else {
		int64_t sleep_time = (m_i64PktTimestamp - m_i64LastPktTimestamp) - (m_i64CurrentTime - m_i64LastTime);
		// printf("pkt time: %u,use time: %u,sleep time: %u",pkt_ts - m_i64LastPktTimestamp,m_i64CurrentTime - m_i64LastTime, sleep_time);
		if(((m_i64PktTimestamp - m_i64LastPktTimestamp) % 1000000) > 10000 && (sleep_count == 0)){
			sleep_count += 1;
			isTimeout  = true;
			return ;
		}
		else{
			if(sleep_count != 1)
			isTimeout  = false;
			sleep_count = 0;
			
		}
		if(sleep_time > 0) {
			struct timeval waitTime;
			waitTime.tv_sec = sleep_time / 1000000;
			waitTime.tv_usec = sleep_time % 1000000;
			
			int err;
			do {
				err = select(0, NULL, NULL, NULL, &waitTime);
			} while (err < 0 && errno != EINTR);
		}
		m_i64LastPktTimestamp = m_i64PktTimestamp;
		m_i64LastTime = m_i64CurrentTime;
		m_i64LastTime += sleep_time;
	}
}


}  // pandar namespace
