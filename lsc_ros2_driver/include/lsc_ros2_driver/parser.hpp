/*
* Copyright (c) 2022, Autonics Co.,Ltd.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*   * Neither the name of the Autonics Co.,Ltd nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef PARSER_HPP
#define PARSER_HPP

#include <vector>

// #define PRINT_PARSED_DATA

#define PI  3.14159265358979323846
#define DEG2RAD PI / 180
#define RAD2DEG 180 / PI

#define NUM_OF_CMD_TYPE 7
#define NUM_OF_CMD 8

#define NUMBER_OF_PARAM 13

enum CmdListNum
{
    NONE = 0,
    SET_ACCESS_LEVEL,
    SENSOR_SCAN_INFO,
    SENSOR_START,
    SENSOR_STOP,
    SCAN_DATA,
    FIRST_CONNECT_DUMMY_SEND,
    SCAN_DATA_CONFIG
};


struct ScanInfo
{
    int32_t angle_start;
    int32_t angle_end;
    uint16_t fw_ver;
    std::string model_name;
};

struct ScanMea
{
    uint16_t scan_counter;
    uint16_t scan_freq;
    uint16_t meas_freq;
    int32_t angle_begin;
    uint16_t angle_resol;
    uint16_t amnt_of_data;
    uint16_t active_field_num;

    std::vector<float> ranges;
    std::vector<float> rssi;
};

struct ScanDataConfig
{
    int32_t angle_start;
    int32_t angle_end;
    uint8_t rssi_activate;
    uint16_t scan_interval;
    uint8_t fieldset_output_activate;
};

struct Lsc_t
{
    ScanMea scan_mea;
    ScanInfo scan_info;
    ScanDataConfig scan_data_config;
};




class Parser
{
  public:
    virtual int parsingMsg(std::vector<unsigned char> raw_msg, std::shared_ptr<Lsc_t> parsed_data) = 0;
    virtual std::string getResponse() = 0;
};

class AsciiParser : public Parser
{
  public:
    AsciiParser();
    ~AsciiParser();
    int parsingMsg(std::vector<unsigned char> raw_msg, std::shared_ptr<Lsc_t> parsed_data);
    std::string getResponse();

  private:
    std::string command_;
    
    std::vector<unsigned char> raw_msg; 
    std::shared_ptr<Lsc_t> parsed_data;
};


class UdpParser
{
  public:
    UdpParser();
    ~UdpParser();
    int parsingMsg(std::vector<unsigned char> raw_msg);
    std::string getResponse();
    
    struct UdpInfo
    {
        unsigned char IpAddr[4];
        unsigned char BroadcastVersion[4];
        unsigned char HWVersion[4];
        unsigned char SWVersion[4];
        unsigned char FPGAVersion[4];
        unsigned char SubnetMask[4];
        unsigned char GateWay[4];
        unsigned char Port[4];
        unsigned char MAC[8];
        unsigned char ModelName[32];
        unsigned char InUse;
    };

    struct UdpSet
    {
        unsigned char MAC[8];
        unsigned char OldIp[4];
        unsigned char NewIp[4];
        unsigned char SubnetMask[4];
        unsigned char GateWay[4];
        unsigned char Port[4];
    };

    unsigned char makeCheckSum(unsigned char* pData, uint16_t nLength);
    bool VerifyCheckSum(unsigned char* pData, uint16_t nLength);
    int makeUdpCmd(int cmd, unsigned char* sendbuf, std::shared_ptr<UdpSet> info);

    std::shared_ptr<UdpInfo> NetworkInfo;
    std::shared_ptr<UdpSet> NetworkChange;
    std::vector<unsigned char> raw_msg; 
    std::string recv_addr;

  private:
    std::string command_;    
};


#endif
