/*
 * @Author: HuangWei
 * @Date: 2024-04-08 15:32:09
 * @LastEditors: HuangWei
 * @LastEditTime: 2024-04-08 15:32:09
 * @FilePath: /chcnav_ros_0.48/src/hc_msg_parser_node/device_connector/device_connector.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by JOJO, All Rights Reserved. 
 */
#ifndef __HC_DEVICE_CONNECTOR_HPP__
#define __HC_DEVICE_CONNECTOR_HPP__

// 这是一个不同的解析器的套件头
class hc__device_connector
{
public:
    /**
     * @brief Establish a connection with the device
     *
     * @return int < 0 failed. @c 0 success. @c 1 already opened.
     */
    virtual int connect(void) = 0;

    /**
     * @brief Send information to the device
     *
     * @param data data block
     * @param len length of data
     * @return int @c nums of bytes written. @c -1 means write fail.
     */
    virtual int write(char *data, unsigned int len) = 0;

    /**
     * @brief Read information from the device
     *
     * @param data Space for storing data
     * @param maxsize max size of the data memory
     * @return int @c nums of read. @c -1 means failed.
     */
    virtual int read(char *data, unsigned int maxsize) = 0;

    /**
     * @brief Disconnect from the device
     *
     * @return int @c 0 means succeed. @c -1 failed.
     */
    virtual int disconnect(void) = 0;
};

#endif