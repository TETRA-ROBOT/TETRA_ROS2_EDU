#pragma once	
#ifndef __ARGV_PARSER_H__
#define __ARGV_PARSER_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file argv_parser.hpp
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief for parse intput ARGV
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <string>

#include "common.h"

/**
 * @brief define user structure for argv
 * 
 */
struct ArgvContainer
{
	std::string localIP = kanavi::common::DEFAULT_LOCAL_IP;		// local IP address
	std::string multicastIP = kanavi::common::DEFAULT_MULTICAST_IP;	// multicast IP address
	std::string topicName = kanavi::common::ROS_TOPIC_NAME;		// ROS Node topic Name
	std::string fixedName = kanavi::common::ROS_FIXED_NAME;		// ROS Node Fixed Name
	int port = kanavi::common::DEFAULT_PORT_NUM;					// port number
	bool checkedMulticast = false;		// checked multicast
	bool checkedDebug = false;			// checked debug log output
	bool checkedTimestamp = false;		// checked timestamp
};


/**
 * @class argv_parser
 * @brief Parses command-line arguments to extract program configurations and options.
 */
class ArgvParser
{
public:
	ArgvParser(const int& argc, char** argv)
	{
		parseArgv(argc, argv);
	}
	~ArgvParser(){}

/**
 * @brief Returns the parsed command-line parameters.
 * @return argvContainer An object containing extracted arguments such as IP address, port, topic name, etc.
 */
	ArgvContainer GetParameters();

private:
	// funcs.
/**
 * @brief Parses command-line arguments.
 * @return void
 * @param &argc_ The number of command-line arguments.
 * @param **argv_ The array of argument strings passed to the program.
 */
	void parseArgv(const int& argc, char** argv);
	
	/* data */
	ArgvContainer mArgvResult;

	// Vars.

};

/**
 * @brief Parses command-line arguments.
 * @return void
 * @param &argc_ The number of command-line arguments.
 * @param **argv_ The array of argument strings passed to the program.
 */
inline void ArgvParser::parseArgv(const int& argc, char** argv)
{
	if(0 == argc)
	{
		printf("Active Default Mode");
		mArgvResult.checkedMulticast = true;
	}

	for(int i=0; i<argc; i++)
	{
		if(!strcmp(argv[i], kanavi::ros::PARAMETER_IP))		        // check ARGV - IP & port num.
		{
			mArgvResult.localIP = argv[i+1];
			mArgvResult.port = atoi(argv[i+2]);
		}
		else if(!strcmp(argv[i], kanavi::ros::PARAMETER_MULTICAST))	// check ARGV - udp multicast ip
		{
			mArgvResult.checkedMulticast = true;
			mArgvResult.multicastIP = argv[i+1];
		}
		else if(!strcmp(argv[i], kanavi::ros::PARAMETER_FIXED))		// check ARGV - ROS Fixed name
		{
			mArgvResult.fixedName = argv[i+1];
		}
		else if(!strcmp(argv[i], kanavi::ros::PARAMETER_TOPIC))		// check ARGV - ROS topic name
		{
			mArgvResult.topicName = argv[i+1];
		}
		else if(!strcmp(argv[i], kanavi::ros::PARAMETER_DEBUG))		// check ARGV - debug log output
		{
			mArgvResult.checkedDebug = true;
		}
		else if(!strcmp(argv[i], kanavi::ros::PARAMETER_TIMESTAMP))	// check ARGV - timestamp for debug log
		{
			mArgvResult.checkedTimestamp = true;
		}
	}

}

/**
 * @brief Returns the parsed command-line parameters.
 * @return argvContainer An object containing extracted arguments such as IP address, port, topic name, etc.
 */
inline ArgvContainer ArgvParser::GetParameters()
{
	return mArgvResult;
}

#endif // __ARGV_PARSER_H__