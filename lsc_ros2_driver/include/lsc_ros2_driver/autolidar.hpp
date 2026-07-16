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


#ifndef AUTOLIDAR_HPP
#define AUTOLIDAR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "socket.hpp"
#include "parser.hpp"

#define USE_DIAGNOSTICS

#define RESPONS_WAIT_CNT 20 /// x100ms

#define RECONNECT_TRY_COUNT 999

#ifdef USE_DIAGNOSTICS
  #include "diagnostic_updater/diagnostic_updater.hpp"
  #include "diagnostic_updater/publisher.hpp"
#endif

class Autolidar : public rclcpp::Node
{
  public:
    explicit Autolidar();
    virtual ~Autolidar();

    int init();
    int get_response(std::string cmd);
    int8_t send_command(std::string cmd);

    void setCommType(std::shared_ptr<Communication> comm_type);
    void setParser(std::shared_ptr<Parser> parser);


  private:
    void run();
    void publish_laserscan(std::shared_ptr<sensor_msgs::msg::LaserScan> msg, std::shared_ptr<Lsc_t> lsc);

    unsigned char itohascii(unsigned char input);
    int makeCommand(std::vector<unsigned char>& v_cmd_frame, std::string cmd);

    template<typename T>
    bool declearParam(std::string name, T value);

    #ifdef USE_DIAGNOSTICS
      void diagnostic_update(void);
      void check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat);

      diagnostic_updater::Updater diagnostic_updater_laser_;
      std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> scan_pub_freq_;

      double min_freq_;
      double max_freq_;
    #endif

    int login(std::string password);
    std::string searchMatchedAddr(std::string std_addr, std::vector<std::string> ip_list);
    std::string searchClientip(std::string baseAddr);
    int addrChange();
    int setScanAngle();
    void lsc_init();

    std::shared_ptr<Communication> comm_;
    std::shared_ptr<Parser> parser_;
    std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg_;
    std::shared_ptr<Lsc_t> lsc_;

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr param_callback_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_topic_publisher_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::string ip_addr, frame_id, pub_topic, password, prev_addr, new_addr;
    uint16_t port_number;
    int diagnostics_windows_time;
    float angle_min, angle_max, angle_offset;
    double range_min, range_max, diagnostics_tolerance;
    bool  intensities, ip_change;

    bool diagnostic_param_setting;


    enum IPCHANGE_STAT{
        NONE = 0,
        RQ_INFO,// cmd send
        GET_INFO,// recv info
        INFO_CHANGE,// change cmd send
        GET_RESP// recv ack
    };
};


#endif
