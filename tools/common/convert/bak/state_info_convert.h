#ifndef STATE_INFO_CONVERT_H
#define STATE_INFO_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/state_info.hpp"
#include "dds/msg/StateInfo.hpp"

class ROS2_To_DDS_StateInfo : public SubscribeROS2Msg<ros2::msg::StateInfo>,
	public PublishDDSMsg<dds::msg::StateInfo>
{
public:
	ROS2_To_DDS_StateInfo(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::StateInfo>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::StateInfo>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::StateInfo &ros2_msg)
		{
			dds::msg::StateInfo dds_msg;
			dds_msg.uc_id() = ros2_msg.uc_id;
			dds_msg.uc_state() = ros2_msg.uc_state;
			dds_msg.us_err_code() = ros2_msg.us_err_code;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_StateInfo : public SubscribeDDSMsg<dds::msg::StateInfo>, 
	public PublishROS2Msg<ros2::msg::StateInfo>
{
public:
	DDS_To_ROS2_StateInfo(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::StateInfo>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::StateInfo>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::StateInfo &dds_msg)
		{
			ros2::msg::StateInfo ros2_msg;
			ros2_msg.uc_id = dds_msg.uc_id();
			ros2_msg.uc_state = dds_msg.uc_state();
			ros2_msg.us_err_code = dds_msg.us_err_code();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // STATE_INFO_CONVERT_H