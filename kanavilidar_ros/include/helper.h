#pragma once

#include <cstring>
#include <cstdio>
#include "common.h"

// 공통으로 사용되는 help 메시지 출력 함수
inline void printHelpMessage() 
{
    printf("Usage: kanavi_vl [OPTION]...\n\n"
           "Network options:\n"
           "  %-7s [ip] [port]    set network information\n"
           "  %-7s [ip]           set multicast IP\n\n"
           "ROS options:\n"
           "  %-7s [name]         set fixed frame name for rviz\n"
           "  %-7s [name]         set topic name for rviz\n\n"
           "Debug options:\n"
           "  %-7s                enable debug log output\n"
           "  %-7s                enable timestamp debug log output\n\n",
           kanavi::ros::PARAMETER_IP,
           kanavi::ros::PARAMETER_MULTICAST,
           kanavi::ros::PARAMETER_FIXED,
           kanavi::ros::PARAMETER_TOPIC,
           kanavi::ros::PARAMETER_DEBUG,
           kanavi::ros::PARAMETER_TIMESTAMP);
}

// ROS1 전용 help 체크 함수
inline bool checkHelpOptionROS1(int argc, char** argv) 
{
    for (int i = 0; i < argc; i++) 
    {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") || !strcmp(argv[i], "-help")) {
            printHelpMessage();
            return true;
        }
    }
    return false;
}

// ROS2 전용 help 체크 함수
inline bool checkHelpOptionROS2(int argc, char** argv) 
{
    for (int i = 0; i < argc; i++) 
    {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") || !strcmp(argv[i], "-help")) {
            printHelpMessage();
            return true;
        }
    }
    return false;
}