#pragma once

#include <thread>

#include "ros/ros.h"
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "chcnav/hcrawimub.h"
#include "hc_cgi_protocol.h"

#include "config/config_manager.h"
#include "config/runtime_config.h"

using namespace jojo::drivers;

unsigned int g_leaps = 18;

class GnssChannel {
 public:
  GnssChannel(ros::NodeHandle &parent_nh, const std::string &topic_ns);
  ~GnssChannel();

  /**
   * @brief 处理华测协议的回调函数
   *
   * @param msg 接收到的数据
   * */
  void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg);

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher gs_devpvt_pub;
  ros::Publisher gs_devimu_pub;

  ros::CallbackQueue *callback_queue;
  ros::AsyncSpinner *spinner;

  // 处理各个协议的函数
  void msg_deal__hcrawgnsspvatb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawimuib(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawodob(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawrtcmpb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawrtcmsb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawrtcmb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawimub(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawimuvb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawgsvb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcrawnmeab(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcinspvatb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcinspvatzcb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcpinfoltsb(const chcnav::hc_sentence::ConstPtr &msg);
  void msg_deal__hcpinfonzb(const chcnav::hc_sentence::ConstPtr &msg);
};