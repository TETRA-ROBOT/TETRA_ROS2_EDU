#pragma once
#ifndef __UDP_H__
#define __UDP_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file udp.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief define UDP Functions
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <algorithm>
#include <arpa/inet.h>
#include <cassert>
#include <cstdint> 
#include <iostream>
#include <iterator>
#include <netinet/in.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>     // close
#include <vector>

#define MAX_BUF_SIZE (65000)

/**
 * @class kanavi_udp
 * @brief Provides UDP socket communication functionality including multicast support for Kanavi sensors.
 *
 * This class handles socket setup, data transmission, and reception over UDP.
 * It supports both unicast and multicast communication modes.
 */
class KanaviUDP
{
public:
/**
 * @brief Constructor for initializing UDP communication with local and multicast IP.
 * 
 * @param localIP Local IP address to bind.
 * @param port UDP port number.
 * @param multicastIP Multicast IP address to join.
 */
	KanaviUDP(const std::string& localIP, const int& port, const std::string& multicastIP);
	KanaviUDP(const std::string& localIP, const int& port);
	~KanaviUDP();

/**
 * @brief Receives a UDP packet and returns the raw data.
 * 
 * @return Vector of received bytes.
 */
	std::vector<uint8_t> GetData();

/**
 * @brief Sends a UDP packet to the configured address (Not Used).
 * 
 * @param data Byte buffer to send.
 */
	void SendData(std::vector<uint8_t> data);

/**
 * @brief Establishes the UDP socket connection.
 * 
 * @return 0 if successful, or error code otherwise.
 */
	int Connect();
	
/**
 * @brief Closes the UDP socket connection.
 * 
 * @return 0 if successful, or error code otherwise.
 */
	int Disconnect();

private:
	/* data */

	//SECTION -- FUNCS.
/**
 * @brief Initializes the UDP socket with given IP, port, and optional multicast settings.
 * 
 * @param ip Local IP address to bind the socket.
 * @param port Port number for UDP communication.
 * @param multicastIP (Optional) Multicast group IP address. Default is "224.0.0.5".
 * @param multiChecked Whether to enable multicast reception.
 * @return 0 if successful, or error code otherwise.
 */
	int init(const std::string& ip, const int& port, std::string multicastIP = "224.0.0.5", bool multiChecked = false);

/**
 * @brief Checks and logs the current receive buffer size for the UDP socket.
 */
	void checkUdpBufSize();

	//!SECTION --------

	//SECTION -- VARS.
	struct sockaddr_in mUdpAddr;
	struct sockaddr_in mSenderAddr;
	int mUdpSocket;

	struct ip_mreq mMultiAddr;

	uint8_t mUdpBuf[MAX_BUF_SIZE];

	//!SECTION --------
};

#endif // __UDP_H__