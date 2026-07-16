#pragma once
#ifndef __KANAVI_LIDAR_H__
#define __KANAVI_LIDAR_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file kanavi_lidar.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief define processing Function for kanavi-mobility LiDAR VL-series Raw data
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <cstdint>
#include <memory>
#include <iostream>
#include <vector>

#include "common.h"
#include "kanavi_datagram.h"

/**
 * @class KanaviLidar
 * @brief Handles LiDAR data parsing and processing for different Kanavi LiDAR models (R2, R4, R270).
 *
 * This class is responsible for receiving raw LiDAR data packets, validating them,
 * and converting them into structured data formats such as KanaviDatagram.
 * It supports model-specific parsing logic and provides access to processed results.
 */

class KanaviLidar
{
public:
/**
 * @brief Constructor for kanavi_lidar class with specific model.
 * @param model_ Integer model identifier (e.g., R2, R4, R270).
 */
	KanaviLidar(int model);
	~KanaviLidar();

/**
 * @brief Processes the input data and performs parsing and validation.
 * @param data The input LiDAR data to process.
 * @return Status code after processing.
 */
	int Process(const std::vector<uint8_t>& data);

/**
 * @brief Checks whether the LiDAR data processing is complete.
 * @return true if the full packet has been processed.
 */
	bool IsProcessEnd() const;

/**
 * @brief Initializes the process end flag.
 */
	void InitProcessEnd();

/**
 * @brief Retrieves the parsed datagram result.
 * @return Parsed LiDAR data in KanaviDatagram format.
 */
	KanaviDatagram GetDatagram();

private:
// FUNCTIONS----
/**
 * @brief Classifies the incoming raw LiDAR data.
 * @param data The raw data buffer received from the LiDAR sensor using UDP.
 * @return Integer representing the classification result or model type.
 */
	int classification(const std::vector<uint8_t>& data);

/**
 * @brief Parses the input LiDAR data according to the internal model.
 * @param data Raw data to parse.
 */
	void parse(const std::vector<uint8_t>& data);

/**
 * @brief Parses R2-model LiDAR data into structured format.
 * @param input Raw input data.
 * @param output Output structure to hold parsed data.
 */
	void r2(const std::vector<uint8_t>& input, KanaviDatagram* output);
/**
 * @brief Parses R4-model LiDAR data into structured format.
 * @param input Raw input data.
 * @param output Output structure to hold parsed data.
 */
	void r4(const std::vector<uint8_t>& input, KanaviDatagram* output);
/**
 * @brief Parses R270-model LiDAR data into structured format.
 * @param input Raw input data.
 * @param output Output structure to hold parsed data.
 */
	void r270(const std::vector<uint8_t>& input, KanaviDatagram* output);
/**
 * @brief Parses the length section of the data for a given channel.
 * @param input Raw input data.
 * @param output Output datagram structure.
 * @param ch Channel index to parse.
 */
	void parseLength(const std::vector<uint8_t>& input, KanaviDatagram* output, int ch);
// !FUNTCIONS---

	/* data */
	std::unique_ptr<KanaviDatagram> mPtrdatagram;
	std::vector<uint8_t> mVecTempBuf;	// for complete packet
	bool mbCheckOnGoing;

	int mCheckedModel;
	uint8_t mCheckedChannel;
	size_t mTotalSize;

	bool mbCheckParseEnd;
	bool mbCheckedLidarInputed;
};


#endif // __KANAVI_LIDAR_H__