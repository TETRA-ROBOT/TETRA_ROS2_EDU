#pragma once
#ifndef __KANAVI_NODE_H__
#define __KANAVI_NODE_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file kanavi_node.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief Header for Kanavi LiDAR ROS1 Node
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <chrono>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>

#include "argv_parser.hpp"
#include "kanavi_datagram.h"
#include "kanavi_lidar.h"	// for LiDAR data processing
#include "udp.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @class kanavi_node
 * @brief ROS1-compatible LiDAR interface for Kanavi sensors.
 *
 * This class handles UDP communication with the LiDAR sensor, parses the incoming data,
 * converts it into a ROS-compatible PointCloud2 message, and publishes it to a topic.
 */
class KanaviNode
{
public:
	KanaviNode(const std::string& node, int& argc, char** argv);
	~KanaviNode();

/**
 * @brief Main loop to receive, process, and publish LiDAR point cloud in ROS1.
 */
	void Run();

private:

	//SECTION - FUNCS.

/**
 * @brief Receives LiDAR data from the UDP socket and processes it.
 */
	std::vector<uint8_t> receiveDatagram();

/**
 * @brief Ends the ROS1 node operation and releases resources.
 */
	void endProcess();

/**
 * @brief Configures ROS1 log parameters such as log level and output behavior.
 */
	void setLogParameters();

/**
 * @brief Converts raw datagram into an internal point cloud representation.
 * @param datagram Parsed datagram from LiDAR sensor.
 */
	void length2PointCloud(KanaviDatagram datagram);

/**
 * @brief Calculates angular resolution and spacing for a specific LiDAR model.
 * @param model LiDAR model identifier.
 */
	void CalculateAngular(int model);

/**
 * @brief Converts a KanaviDatagram into a PCL-compatible point cloud.
 * @param datagram Parsed KanaviDatagram.
 * @param cloud Output point cloud.
 */
	void generatePointCloud(const KanaviDatagram& datagram, PointCloudT& cloud);

/**
 * @brief Converts a length measurement and trigonometric values to a 3D point.
 * @param len Distance measurement.
 * @param vSin Vertical sine.
 * @param vCos Vertical cosine.
 * @param hSin Horizontal sine.
 * @param hCos Horizontal cosine.
 * @return Computed 3D point(XYZRGB).
 */
	PointT length2Point(float len, float vSin, float vCos, float hSin, float hCos);

/**
 * @brief Converts HSV color to RGB color.
 * @param fR Pointer to resulting red value.
 * @param fG Pointer to resulting green value.
 * @param fB Pointer to resulting blue value.
 * @param fH Hue component.
 * @param fS Saturation component.
 * @param fV Value component.
 */
	void hsv2Rgb(float* fR, float* fG, float* fB, float fH, float fS, float fV);
	
/**
 * @brief Converts a PCL point cloud to ROS1 PointCloud2 message format.
 * @param cloud Input point cloud (XYZRGB).
 * @param frame Coordinate frame ID.
 * @return ROS1 PointCloud2 message.
 */
	sensor_msgs::PointCloud2 cloud2CloudMSG(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const std::string& frame);

/**
 * @brief Rotates the point cloud around the Z-axis by a given angle.
 * @param cloud Input/output point cloud.
 * @param angle Rotation angle in radians.
 */
	void rotateAxisZ(PointCloudT::Ptr cloud, float angle);

//!SETCION

	/* data */
	//SECTION - Variables

	// argv parse Class
	std::unique_ptr<ArgvParser> mArgv;
	
	// Network
	std::string mLocalIP;
	int mPort;
	std::string mMulticastIP;

	// ROS
	std::string mTopicName;
	std::string mFixedName;
	ros::NodeHandle mNodeHnadle;
	ros::Publisher mPtrPublisher;

	// timer for RECV
	ros::Timer mPtrTimer;

	// flags
	bool mbCheckedMulticast;
	bool mbCheckedDebug;
	bool mbCheckedTimestamp;

	// UDP network
	std::unique_ptr<KanaviUDP> mPtrUDP;

	// LiDAR data processing Class
	std::unique_ptr<KanaviLidar> mPtrProcess;

	// sin, cos value for calculate Angle
	std::vector<float> mVecVerticalSin;
	std::vector<float> mVecVerticalCos;
	std::vector<float> mVecHorizontalSin;
	std::vector<float> mVecHorizontalCos;

	// pcl point cloud 
	PointCloudT::Ptr mPtrPointCloud;

	// rotate Angle
	float mRotateAngle;

//!SECTION	

};
#endif // __KANAVI_NODE_H__