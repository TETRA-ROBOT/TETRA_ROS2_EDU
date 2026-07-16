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
 * @brief Header for Kanavi LiDAR ROS2 Node
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <chrono>
#include <pcl/common/common.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "argv_parser.hpp"
#include "kanavi_datagram.h"
#include "kanavi_lidar.h"
#include "udp.h"	

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @class kanavi_node
 * @brief ROS2 node wrapper for Kanavi LiDAR sensor integration.
 *
 * This class manages receiving LiDAR data via UDP, parsing it into structured
 * data formats, converting it into PCL point clouds, and publishing to ROS topics.
 */
class KanaviNode
: public rclcpp::Node
{
public:
/**
 * @brief Constructor for kanavi_node, sets up the ROS2 node.
 * @param node_ Node name.
 * @param argc_ Argument count.
 * @param argv_ Argument values.
 */
	KanaviNode(const std::string& node, int& argc, char** argv);
	~KanaviNode();

/**
 * @brief Calculates angular resolution and alignment based on LiDAR model.
 * @param model LiDAR model type (e.g., R2, R4, R270).
 */
	void CalculateAngular(int model);

private:

//SECTION - FUNCS.

/**
 * @brief Receives UDP data from the LiDAR and initiates parsing.
 */
	void receiveData();

/**
 * @brief Finalizes the node process and cleans up resources.
 */
	void endProcess();

/**
 * @brief Sets up the logging parameters for the ROS2 node.
 */
	void setLogParameters();

/**
 * @brief Converts raw datagram into an internal point cloud representation.
 * @param datagram Parsed datagram from LiDAR sensor.
 */
	void length2PointCloud(KanaviDatagram datagram);

/**
 * @brief Converts a KanaviDatagram into a PCL-compatible point cloud.
 * @param datagram Parsed KanaviDatagram.
 * @param cloud_ Output point cloud.
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
 * @brief Rotates the point cloud around the Z-axis by a given angle.
 * @param cloud Input/output point cloud.
 * @param angle Rotation angle in radians.
 */
	void rotateAxisZ(PointCloudT::Ptr cloud, float angle);

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
 * @brief Publishes the given point cloud to a ROS2 topic.
 * @param cloud Point cloud to publish.
 */
	void publishPointCloud(PointCloudT::Ptr cloud);
	
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
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPtrPublisher;
	
	// timer for RECV
	rclcpp::TimerBase::SharedPtr mPtrTimer;

	// flags
	bool mbCheckedMulticast;
	bool mbCheckedDebug;
	bool mbCheckedTimestamp;

	// rotate angle
	float mRotateAngle;

	// sin, cos value for calculate Angle
	std::vector<float> mVecVerticalSin;
	std::vector<float> mVecVerticalCos;
	std::vector<float> mVecHorizontalSin;
	std::vector<float> mVecHorizontalCos;

	// pcl point cloud 
	PointCloudT::Ptr mPtrPointCloud;

	// UDP network
	std::unique_ptr<KanaviUDP> mPtrUDP;

	// LiDAR Processor
	std::unique_ptr<KanaviLidar> mPtrProcess;

//!SECTION	

};
#endif // __KANAVI_NODE_H__