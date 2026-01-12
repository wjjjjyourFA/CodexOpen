#ifndef SERIAL_BASE_H
#define SERIAL_BASE_H

#include <stdio.h> /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <fcntl.h> /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h> /*错误号定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>

int SerialOpen(const char *port);
void SerialClose(int fd);
int SerialSet(int fd, int speed, int flow_ctrl, int databits, int stopbits,
              int parity);
int SerialRecv(int fd, char *rcv_buf, int data_len);
int SerialSend(int fd, const void *const send_buf, int data_len);

#endif  // SERIAL_BASE_H
