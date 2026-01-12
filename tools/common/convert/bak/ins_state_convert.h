#ifndef INS_STATE_CONVERT_H
#define INS_STATE_CONVERT_H

#include <publish_msg_base/publish_msg_wrapper.h>
#include <subscribe_msg_base/subscribe_msg_wrapper.h>
#include "ros2/msg/ins_state.hpp"
#include "dds/msg/InsState.hpp"

class ROS2_To_DDS_InsState : public SubscribeROS2Msg<ros2::msg::InsState>,
	public PublishDDSMsg<dds::msg::InsState>
{
public:
	ROS2_To_DDS_InsState(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeROS2Msg<ros2::msg::InsState>(rcl_node, sub_topic_name, sub_topic_name),
		PublishDDSMsg<dds::msg::InsState>(domain_id, pub_topic_name, zcc_flag)
	{
		auto func = [this](const ros2::msg::InsState &ros2_msg)
		{
			dds::msg::InsState dds_msg;
			dds_msg.ui_msg_id() = ros2_msg.ui_msg_id;
			dds_msg.ui_msg_data_length() = ros2_msg.ui_msg_data_length;
			dds_msg.ul_msg_time() = ros2_msg.ul_msg_time;

			{int size = ros2_msg.st_ins_state.size();
			dds_msg.st_ins_state().resize(size);
			for(int i = 0; i < size; ++i)
			{
			dds_msg.st_ins_state()[i].uc_id() = ros2_msg.st_ins_state[i].uc_id;
			dds_msg.st_ins_state()[i].uc_state() = ros2_msg.st_ins_state[i].uc_state;
			dds_msg.st_ins_state()[i].us_err_code() = ros2_msg.st_ins_state[i].us_err_code;
			}}
			dds_msg.us_check_sum() = ros2_msg.us_check_sum;

			this->Publish(dds_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};

class DDS_To_ROS2_InsState : public SubscribeDDSMsg<dds::msg::InsState>, 
	public PublishROS2Msg<ros2::msg::InsState>
{
public:
	DDS_To_ROS2_InsState(const std::shared_ptr<rclcpp::Node> &rcl_node,
		std::string sub_topic_name,
		uint32_t domain_id,
		std::string pub_topic_name,
		bool zcc_flag = false)
		: SubscribeDDSMsg<dds::msg::InsState>(sub_topic_name, sub_topic_name, domain_id, zcc_flag),
		PublishROS2Msg<ros2::msg::InsState>(rcl_node, pub_topic_name)
	{
		auto func = [this](const dds::msg::InsState &dds_msg)
		{
			ros2::msg::InsState ros2_msg;
			ros2_msg.ui_msg_id = dds_msg.ui_msg_id();
			ros2_msg.ui_msg_data_length = dds_msg.ui_msg_data_length();
			ros2_msg.ul_msg_time = dds_msg.ul_msg_time();

			{int size = dds_msg.st_ins_state().size();
			ros2_msg.st_ins_state.resize(size);
			for(int i = 0; i < size; ++i)
			{
			ros2_msg.st_ins_state[i].uc_id = dds_msg.st_ins_state()[i].uc_id();
			ros2_msg.st_ins_state[i].uc_state = dds_msg.st_ins_state()[i].uc_state();
			ros2_msg.st_ins_state[i].us_err_code = dds_msg.st_ins_state()[i].us_err_code();
}}
			ros2_msg.us_check_sum = dds_msg.us_check_sum();

			this->Publish(ros2_msg);
		};

		this->SetOptionalCallbackFunc(func);
	}
};
#endif // INS_STATE_CONVERT_H