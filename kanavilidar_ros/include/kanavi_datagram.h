#pragma once
#ifndef __KANAVI_DATAGRAM_H__
#define __KANAVI_DATAGRAM_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file kanavi_datagram.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief Datagram class definition for processing raw data from KANAVI LiDAR VL-series
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <cstdint>
#include <iostream>
#include <vector>

#include "common.h"

/**
 * @class KanaviDatagram
 * @brief Datagram class for processing and storing raw data from KANAVI LiDAR
 * 
 * This class is used to process and store raw data received from the LiDAR sensor.
 * It supports data processing for different LiDAR models (R2, R4, R270) and
 * contains all necessary information for point cloud generation.
 */
class KanaviDatagram
{
public:
/**
 * @brief Constructor
 * @param model LiDAR model identifier (R2, R4, R270)
 */
	KanaviDatagram(int model);
	~KanaviDatagram();

// Basic data accessors
/**
 * @brief Get LiDAR model identifier
 * @return int LiDAR model identifier
 */
    int GetModel() const;
    
    
/**
 * @brief Get vertical field of view
 * @return double Vertical FoV in degrees
 */
    double GetVerticalFov() const;

/**
 * @brief Get vertical resolution
 * @return double Vertical resolution in degrees
 */
    double GetVerticalResolution() const;

/**
 * @brief Get horizontal field of view
 * @return double Horizontal FoV in degrees
 */
    double GetHorizontalFov() const;
    
/**
 * @brief Get horizontal resolution
 * @return double Horizontal resolution in degrees
 */
    double GetHorizontalResolution() const;
    
/**
 * @brief Check if data input is complete
 * @return bool True if data input is complete
 */
    bool IsCheckedEnd() const;
    
/**
 * @brief Set data input completion status
 * @param checked Data input completion status
 */
    void SetCheckedEnd(bool);
    
/**
 * @brief Get LiDAR sensor IP address
 * @return const std::string& LiDAR sensor IP address
 */
    const std::string& GetLidarIP() const;
    
/**
 * @brief Set LiDAR sensor IP address
 * @param ip LiDAR sensor IP address
 */
    void SetLidarIP(const std::string& ip);

// Buffer accessors
/**
 * @brief Get raw data buffer (const version)
 * @return const std::vector<std::vector<uint8_t>>& Raw data buffer
 */
    const std::vector<std::vector<uint8_t>>& GetRawBuffer() const;
    
/**
 * @brief Get raw data buffer
 * @return std::vector<std::vector<uint8_t>>& Raw data buffer
 */
    std::vector<std::vector<uint8_t>>& GetRawBuffer();
    
/**
 * @brief Get distance data buffer (const version)
 * @return const std::vector<std::vector<float>>& Distance data buffer
 */
    const std::vector<std::vector<float>>& GetLengthBuffer() const;
    
/**
 * @brief Get distance data buffer
 * @return std::vector<std::vector<float>>& Distance data buffer
 */
    std::vector<std::vector<float>>& GetLengthBuffer();
    
/**
 * @brief Get channel points size
 * @return size_t channel points size
 */
    size_t GetChannelPointsSize() const;

    
/**
 * @brief Get number of channels
 * @return uint8_t Number of channels
 */
    uint8_t GetChannelCount() const;
    
/**
 * @brief Get active channels list (const version)
 * @return const std::vector<bool>& List of active channels
 */
    const std::vector<bool>& GetActiveChannels() const;
    
/**
 * @brief Get active channels list
 * @return std::vector<bool>& List of active channels
 */
    std::vector<bool>& GetActiveChannels();

private:
/// LiDAR model identifier
	int mModel;
/// Vertical field of view in degrees
	double mVerticalFOV;
/// Vertical resolution in degrees
	double mVerticalResolution;
/// Horizontal field of view in degrees
	double mHorizontalFOV;
/// Horizontal resolution in degrees
	double mHorizontalResolution;
/// Data input completion status
	bool mbCheckEnd;
/// LiDAR sensor IP address
	std::string mLidarIP;
/// Raw data buffer (data per channel)
	std::vector< std::vector<uint8_t> > mVecRawBuf;
/// Distance data buffer (distance values per channel)
	std::vector< std::vector<float> > mVecLenBuf;
/// Number of points per channel (horizontal resolution)
	size_t mChannelPointsSize;  
/// Number of channels
	uint8_t mChannelCount;
/// List of active channels
	std::vector<bool> mVecChannelActive;
};

#endif // __KANAVI_DATAGRAM_H__