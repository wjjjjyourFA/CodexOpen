/*
 * @Author: HuangWei
 * @Date: 2023-06-14 17:17:28
 * @LastEditors: HuangWei
 * @LastEditTime: 2023-07-05 16:21:00
 * @FilePath: /radar/ars548_process/src/udp_interface.h
 * @Description: 
 * 
 * Copyright (c) 2023 by JOJO, All Rights Reserved. 
 */
#ifndef UDP_INTERFACE_H
#define UDP_INTERFACE_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <iostream>

using namespace std;

// ============ UDP 组播接收 ============
class UdpInterface
{
public:
    UdpInterface();
    ~UdpInterface();

    int initUdpMulticastServer(const string& _group_ip, int _group_port);
    int initUdpMulticastServer(const string& _group_ip, int _group_port, const string& _host_ip);

    int initUdpUnicastClient(const string &dest_ip, int dest_port, int local_port);
    int initUdpUnicastClient(const string &local_ip, int local_port);

    void receiveFromRadar(char* data, int &len);
    void receiveFromRadar(struct sockaddr_in* addr, char* data, int &len);

    int sendToRadar(char* data, int len);
    int sendToRadar(const string& dest_ip, int dest_port, char* data, int len);

private:
    int socket_server_fd;
    struct sockaddr_in group_addr;  // group address
    socklen_t addr_len;
    struct sockaddr_in addr_clie{};

    int socket_client_fd;
    struct sockaddr_in addr_serv;
};

#endif 
