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



#ifndef SOCKET_HPP
#define SOCKET_HPP

#ifdef _MSC_VER
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif
#include <array>
#include <vector>
#include <queue>

#include <string>


#define CONNECTION_TRY_COUNT 999

#define RS_TIMEOUT_S    3
#define RS_TIMEOUT_NS   0

#define STX 0x02
#define ETX 0x03

//#define PRINT_DEBUG

class Communication
{
  public:
    virtual int CommConnect(std::string addr, uint16_t port) = 0;
    virtual int CommRead(unsigned char* buffer, int buf_size) = 0;
    virtual int CommWrite(unsigned char* buffer, int buf_size) = 0;
    virtual void CommDisconnect() = 0;

    virtual bool IsConnected() = 0;
    std::queue<std::vector<unsigned char>> recvQueue;
};


class tcpComm : public Communication
{
  public:
    tcpComm();
    ~tcpComm();

    int CommConnect(std::string addr, uint16_t port);
    int CommRead(unsigned char* buffer, int buf_size);
    int CommWrite(unsigned char* buffer, int buf_size);
    void CommDisconnect();

    bool IsConnected(void);


    friend void* readCallback(void* arg);
#ifdef _MSC_VER
    SOCKET getServerSocket(void);
#else
    int getServerSocket(void);
#endif

    bool getthreadRunning(void);
    bool getRcvTimeout(void);
    bool getRcvError(void);

    void setConnected(bool flag);
    void setThreadRunning(bool flag);
    void setRcvTimeout(bool flag);
    void setRcvError(bool flag);

  private:
    void putBufToMsg(unsigned char* buf, uint16_t size);

    sockaddr_in m_server_addr_;
    std::thread thread_read_;
    uint16_t port_num_;

#ifdef _MSC_VER
    SOCKET m_server_sock_;
#else
    int m_server_sock_;
#endif
    bool thread_running_;
    bool m_connected_;
    bool rcv_timeout_;
    bool rcv_error_;
};


class udpComm : public Communication
{
  public:
    udpComm();
    ~udpComm();

    int CommConnect(std::string addr, uint16_t port);
    int CommRead(unsigned char* buffer, int buf_size);
    int CommWrite(unsigned char* buffer, int buf_size);
    void CommDisconnect();

    bool IsConnected(void);


    friend void* udpReadCallback(void* arg);
#ifdef _MSC_VER
    SOCKET getClientSocket(void);
#else
    int getClientSocket(void);
#endif

    bool getthreadRunning(void);
    bool getRcvTimeout(void);
    bool getRcvError(void);

    void setConnected(bool flag);
    void setThreadRunning(bool flag);
    void setRcvTimeout(bool flag);
    void setRcvError(bool flag);

    void pushAddr(std::string recvaddr);
    std::string popAddr(void);

  private:
    sockaddr_in m_server_addr_;
    std::thread thread_read_;
    uint16_t port_num_;

#ifdef _MSC_VER
    SOCKET m_client_sock_;
#else
    int m_client_sock_;
#endif
    bool thread_running_;
    bool m_connected_;
    bool rcv_timeout_;
    bool rcv_error_;

    char recv_ipaddr[30];
    std::queue<std::string> queue_addr_;
};

#endif
