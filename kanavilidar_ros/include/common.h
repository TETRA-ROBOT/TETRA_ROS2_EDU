#pragma once
#ifndef __COMMON_H__
#define __COMMON_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

#include <iostream>
#include <string.h>

namespace kanavi
{
	namespace ros
	{
		constexpr const char* PARAMETER_DEBUG               = "-d";
		constexpr const char* PARAMETER_HELP                = "-h";
		constexpr const char* PARAMETER_IP                  = "-i";
		constexpr const char* PARAMETER_MULTICAST           = "-m";
		constexpr const char* PARAMETER_PORT                = "-p";
		constexpr const char* PARAMETER_TIMESTAMP           = "-t";

		constexpr const char* PARAMETER_FIXED               = "-fix";
		constexpr const char* PARAMETER_TOPIC               = "-topic";
	}

	namespace common
	{
		constexpr const char* ROS_FIXED_NAME                = "map";
		constexpr const char* ROS_TOPIC_NAME                = "kanavi_lidar_msg";

		constexpr const char* DEFAULT_LIDAR_IP              = "192.168.123.200";
		constexpr const char* DEFAULT_LOCAL_IP              = "192.168.123.100";
		constexpr const char* DEFAULT_MULTICAST_IP          = "224.0.0.5";
		constexpr const int	  DEFAULT_PORT_NUM              = 5000;

		namespace specification
		{
			constexpr float		BASE_ZERO_ANGLE			= -45.f;

			namespace R2
			{
				constexpr double	HORIZONTAL_FOV			= 120.f;
				constexpr double	HORIZONTAL_RESOLUTION	= 0.25f;
				constexpr int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FOV/HORIZONTAL_RESOLUTION);
				constexpr double	VERTICAL_FOV			= 3.0f;
				constexpr double	VERTICAL_RESOLUTION		= 1.5f;
				constexpr int		VERTICAL_CHANNEL		= static_cast<int>(VERTICAL_FOV / VERTICAL_RESOLUTION);
				constexpr size_t	RAW_TOTAL_SIZE			= 969;
			}
			namespace R4
			{
				constexpr double	HORIZONTAL_FOV			= 100.f;
				constexpr double	HORIZONTAL_RESOLUTION	= 0.25f;
				constexpr int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FOV/HORIZONTAL_RESOLUTION);
				constexpr double	VERTICAL_FOV			= 4.8f;
				constexpr double	VERTICAL_RESOLUTION		= 1.2f;
				constexpr int		VERTICAL_CHANNEL		= static_cast<int>(VERTICAL_FOV / VERTICAL_RESOLUTION);
				constexpr size_t	RAW_TOTAL_SIZE			= 809;
			}
			namespace R270
			{
				constexpr double	HORIZONTAL_FOV			= 270.f;
				constexpr double	HORIZONTAL_RESOLUTION	= 0.25f;
				constexpr int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FOV/HORIZONTAL_RESOLUTION);
				constexpr double	VERTICAL_FOV			= 1.f;
				constexpr double	VERTICAL_RESOLUTION		= 1.f;
				constexpr int		VERTICAL_CHANNEL		= static_cast<int>(VERTICAL_FOV / VERTICAL_RESOLUTION);
				constexpr size_t	RAW_TOTAL_SIZE			= 2169;
			}
		}

		namespace protocol
		{
			constexpr int HEADER           = 0xFA;

			enum eModel
			{
				R2		                   = 0x03,
				R4		                   = 0x06,
				R270	                   = 0x07
			};

			namespace command
			{
				enum eMode
				{
					CONFIG_SET             = 0xCF,
					DISTANCE_DATA          = 0xDD,
					FIRMWARE_BOOTLOAD      = 0xFB,
					NAK                    = 0xF0,
				};

				enum eChannel
				{
					CHANNEL_0              = 0xC0,
					CHANNEL_1              = 0xC1,
					CHANNEL_2              = 0xC2,
					CHANNEL_3              = 0xC3
				};
	        }

			namespace position
			{
				constexpr int HEADER       = 0;
				constexpr int PRODUCTLINE  = 1;
				constexpr int ID           = 2;
				constexpr int COMMAND      = 3;

				enum class eCommand
				{
					MODE                   = 3,
					PARAMETER              = 4
				};

				const int DATALENGTH       = 5;
				const int RAWDATA_START    = 7;
				const int CHECKSUM         = 1; /*total_size - this*/

			}
		}
	} // namespace common
} // namespace kanavi

#endif // __COMMON_H__